/*
 * DWM3001CDK (nRF52833) 포트 — 플래시에서 혼자 도는 앵커용.
 *
 * SPI, DW3110 리셋/깨우기, 드라이버 연결(dw3000_probe_interf), 송신 전력 변환은
 * drone-hw-bench `task6-qorvo-3to1-20261123` 브랜치(커밋 b934e69)
 * task6_mixed/anchor_3001/qorvo_1to1/port.c 를 그대로 옮겼다. 그 코드로 1대1 거리 측정이 동작했다.
 * 바꾼 점: RAM 실행용 start()/SWD 메일박스 대신 플래시 부팅(gcc_startup_nrf52833.S -> main),
 *          외부 크리스털(HFXO) 시작, LED, UART 로그, 난수, 감시 타이머 추가.
 */
#include <stddef.h>
#include "nrf.h"
#include "deca_interface.h"
#include "deca_probe_interface.h"
#include "dw3000/dw3000_deca_regs.h"
#include "port.h"

/* DWM3001CDK 배선 (Qorvo SDK DWM3001CDK uwb_stack_llhw.cmake 와 같음) */
enum { SCK = 3, MOSI = 8, MISO = 29, RESET = 25, CS = 6 };   /* SCK/MOSI/MISO/RESET = P0, CS = P1 */
enum { UART_TX_PIN = 19 };                                     /* P0.19 -> J-Link OB VCOM */
static const uint8_t led_pins[4] = { 4, 5, 22, 14 };           /* P0: D9, D10, D11, D12 (낮으면 켜짐) */

#define REG(address) (*(volatile uint32_t *)(uintptr_t)(address))
#define P0(offset) REG(0x50000000u + (offset))
#define P1(offset) REG(0x50000300u + (offset))
#define SPI(offset) REG(0x40003000u + (offset))    /* SPIM0 */
#define TIMER2(offset) REG(0x4000A000u + (offset))
#define PIN_CONFIG(pin) (0x700u + 4u * (pin))

static uint8_t tx[1032];
static uint8_t rx[1032];
static uint32_t dw_device_id;

/* ------------------------------------------------------------------ 타이머 (TIMER2, 1 MHz) */
static void timer_init(void)
{
    TIMER2(0x004) = 1; /* STOP */
    TIMER2(0x504) = 0; /* Timer mode */
    TIMER2(0x508) = 3; /* 32 bits */
    TIMER2(0x510) = 4; /* 16 MHz / 16 = 1 MHz */
    TIMER2(0x00C) = 1; /* CLEAR */
    TIMER2(0x000) = 1; /* START */
}

static uint32_t timer_us(void)
{
    TIMER2(0x040) = 1; /* CAPTURE[0] */
    return TIMER2(0x540); /* CC[0] */
}

uint32_t cdk_time_us(void)
{
    return timer_us();
}

static void delay_us(uint32_t us)
{
    const uint32_t start = timer_us();
    while ((uint32_t)(timer_us() - start) < us) { }
}

void deca_usleep(unsigned long us)
{
    delay_us(us);
}

void Sleep(unsigned int milliseconds)
{
    while (milliseconds-- > 0u) {
        delay_us(1000);
        port_wdt_feed();
    }
}

void deca_sleep(unsigned int milliseconds)
{
    Sleep(milliseconds);
}

/* 인터럽트를 쓰지 않고 폴링만 한다 */
decaIrqStatus_t decamutexon(void)
{
    return 0;
}

void decamutexoff(decaIrqStatus_t state)
{
    (void)state;
}

/* ------------------------------------------------------------------ SPI (SPIM0) */
static void spi_slow(void)
{
    SPI(0x524) = 0x20000000u; /* 2 MHz */
}

void port_set_dw_ic_spi_fastrate(void)
{
    SPI(0x524) = 0x80000000u; /* 8 MHz */
}

static void spi_init(void)
{
    SPI(0x500) = 0;
    P0(0x50C) = (1u << SCK) | (1u << MOSI);
    P0(PIN_CONFIG(SCK)) = 3;
    P0(PIN_CONFIG(MOSI)) = 3;
    P0(PIN_CONFIG(MISO)) = 0;
    P1(0x508) = 1u << CS;
    P1(PIN_CONFIG(CS)) = 3;
    SPI(0x508) = SCK;
    SPI(0x50C) = MOSI;
    SPI(0x510) = MISO;
    SPI(0x554) = 0; /* MSB first, mode 0 */
    spi_slow();
    SPI(0x500) = 7; /* SPIM EasyDMA */
}

void reset_DWIC(void)
{
    P0(0x50C) = 1u << RESET;
    P0(PIN_CONFIG(RESET)) = 1;
    Sleep(2);
    P0(PIN_CONFIG(RESET)) = 0; /* RSTn 을 입력으로 풀어 줌 (High 로 밀지 않음) */
    Sleep(5);
}

static void wakeup_dw(void)
{
    P1(0x50C) = 1u << CS;
    delay_us(600);
    P1(0x508) = 1u << CS;
    Sleep(2);
}

static int32_t transfer(uint16_t header_len, const uint8_t *header,
                        uint16_t data_len, const uint8_t *out, uint8_t *in,
                        const uint8_t *crc)
{
    const uint32_t length = (uint32_t)header_len + data_len + (crc != NULL);
    if (length > sizeof(tx)) {
        return DWT_ERROR;
    }
    memcpy(tx, header, header_len);   /* EasyDMA 는 RAM 만 읽으므로 항상 RAM 버퍼로 복사 */
    if (data_len != 0u) {
        if (out != NULL) {
            memcpy(tx + header_len, out, data_len);
        } else {
            memset(tx + header_len, 0, data_len);
        }
    }
    if (crc != NULL) {
        tx[length - 1u] = *crc;
    }

    SPI(0x118) = 0; /* EVENTS_END */
    SPI(0x120) = 0; /* EVENTS_STOPPED */
    SPI(0x534) = (uintptr_t)rx;
    SPI(0x538) = in != NULL ? length : 0;
    SPI(0x544) = (uintptr_t)tx;
    SPI(0x548) = length;
    P1(0x50C) = 1u << CS;
    SPI(0x010) = 1; /* TASKS_START */
    const uint32_t start = timer_us();
    while (SPI(0x118) == 0u) {
        if ((uint32_t)(timer_us() - start) >= 10000u) {
            SPI(0x014) = 1; /* TASKS_STOP */
            P1(0x508) = 1u << CS;
            return DWT_ERROR;
        }
    }
    P1(0x508) = 1u << CS;
    if (in != NULL) {
        memcpy(in, rx + header_len, data_len);
    }
    return DWT_SUCCESS;
}

static int32_t spi_read(uint16_t hlen, uint8_t *header, uint16_t len, uint8_t *data)
{
    return transfer(hlen, header, len, NULL, data, NULL);
}

static int32_t spi_write(uint16_t hlen, const uint8_t *header, uint16_t len, const uint8_t *data)
{
    return transfer(hlen, header, len, data, NULL, NULL);
}

static int32_t spi_write_crc(uint16_t hlen, const uint8_t *header, uint16_t len,
                             const uint8_t *data, uint8_t crc)
{
    return transfer(hlen, header, len, data, NULL, &crc);
}

static struct dwt_spi_s spi = {
    .readfromspi = spi_read,
    .writetospi = spi_write,
    .writetospiwithcrc = spi_write_crc,
    .setslowrate = spi_slow,
    .setfastrate = port_set_dw_ic_spi_fastrate,
};

static const struct dwt_driver_s *drivers[] = { &dw3000_driver };

const struct dwt_probe_s dw3000_probe_interf = {
    .spi = &spi,
    .wakeup_device_with_io = wakeup_dw,
    .driver_list = (struct dwt_driver_s **)drivers,
    .dw_driver_num = 1,
};

uint32_t port_dw_device_id(void)
{
    return dw_device_id;
}

/* ------------------------------------------------------------------ LED */
void port_led(enum port_led led, bool on)
{
    const uint32_t bit = 1u << led_pins[led];
    if (on) {
        NRF_P0->OUTCLR = bit;
    } else {
        NRF_P0->OUTSET = bit;
    }
}

void port_led_toggle(enum port_led led)
{
    const uint32_t bit = 1u << led_pins[led];
    port_led(led, (NRF_P0->OUT & bit) != 0u);
}

static void led_init(void)
{
    for (unsigned i = 0; i < 4; ++i) {
        NRF_P0->OUTSET = 1u << led_pins[i];
        NRF_P0->PIN_CNF[led_pins[i]] = 1u; /* 출력 */
    }
}

/* ------------------------------------------------------------------ UART 로그 (UARTE0, TX 만) */
static char log_buf[192];
static bool log_busy;
static uint32_t log_drop;

static void uart_init(void)
{
    NRF_P0->OUTSET = 1u << UART_TX_PIN;
    NRF_P0->PIN_CNF[UART_TX_PIN] = 1u;
    NRF_UARTE0->PSEL.TXD = UART_TX_PIN;
    NRF_UARTE0->PSEL.RXD = 0xFFFFFFFFu;
    NRF_UARTE0->PSEL.RTS = 0xFFFFFFFFu;
    NRF_UARTE0->PSEL.CTS = 0xFFFFFFFFu;
    NRF_UARTE0->BAUDRATE = UARTE_BAUDRATE_BAUDRATE_Baud115200;
    NRF_UARTE0->CONFIG = 0;
    NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Enabled;
}

bool port_log(const char *line)
{
    if (log_busy) {
        if (NRF_UARTE0->EVENTS_ENDTX == 0u) {
            ++log_drop;
            return false;
        }
        NRF_UARTE0->EVENTS_ENDTX = 0;
        NRF_UARTE0->TASKS_STOPTX = 1;
        log_busy = false;
    }
    size_t n = strlen(line);
    if (n > sizeof(log_buf) - 2u) {
        n = sizeof(log_buf) - 2u;
    }
    memcpy(log_buf, line, n);
    log_buf[n++] = '\r';
    log_buf[n++] = '\n';
    NRF_UARTE0->EVENTS_ENDTX = 0;
    NRF_UARTE0->TXD.PTR = (uint32_t)(uintptr_t)log_buf;
    NRF_UARTE0->TXD.MAXCNT = n;
    NRF_UARTE0->TASKS_STARTTX = 1;
    log_busy = true;
    return true;
}

uint32_t port_log_dropped(void)
{
    return log_drop;
}

void port_log_flush(uint32_t max_us)
{
    const uint32_t start = timer_us();
    while (log_busy && NRF_UARTE0->EVENTS_ENDTX == 0u && (uint32_t)(timer_us() - start) < max_us) {
        port_wdt_feed();
    }
}

/* ------------------------------------------------------------------ 난수, 감시 타이머, 클록 */
uint32_t port_random32(void)
{
    uint32_t value = 0;
    NRF_RNG->CONFIG = 1; /* 치우침 보정 */
    NRF_RNG->TASKS_START = 1;
    for (unsigned i = 0; i < 4; ++i) {
        NRF_RNG->EVENTS_VALRDY = 0;
        const uint32_t start = timer_us();
        while (NRF_RNG->EVENTS_VALRDY == 0u && (uint32_t)(timer_us() - start) < 1000u) { }
        value = (value << 8) | (NRF_RNG->VALUE & 0xFFu);
    }
    NRF_RNG->TASKS_STOP = 1;
    return value ^ timer_us();
}

void port_wdt_feed(void)
{
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

static void wdt_init(void)
{
    if (NRF_WDT->RUNSTATUS) {
        return; /* 이미 켜져 있으면 (감시 타이머 재시작 뒤) 설정을 바꿀 수 없음 */
    }
    NRF_WDT->CONFIG = WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos; /* 디버거로 멈추면 같이 멈춤 */
    NRF_WDT->CRV = 2u * 32768u;                                    /* 2 s */
    NRF_WDT->RREN = WDT_RREN_RR0_Enabled;
    NRF_WDT->TASKS_START = 1;
}

static void hfxo_start(void)
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    const uint32_t start = timer_us();
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0u && (uint32_t)(timer_us() - start) < 5000u) { }
}

void port_init(void)
{
    __disable_irq(); /* 인터럽트는 쓰지 않음 */
    SysTick->CTRL = 0;
    timer_init();
    hfxo_start();      /* UART 보드레이트 정확도용. DW3110 시간은 자체 38.4 MHz 크리스털 사용 */
    wdt_init();
    led_init();
    uart_init();
    spi_init();
    reset_DWIC();
    uint8_t header = 0;
    uint8_t id[4] = { 0 };
    if (spi_read(1, &header, sizeof(id), id) == DWT_SUCCESS) {
        dw_device_id = (uint32_t)id[0] | ((uint32_t)id[1] << 8) |
                       ((uint32_t)id[2] << 16) | ((uint32_t)id[3] << 24);
    }
}
