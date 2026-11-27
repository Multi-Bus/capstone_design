/* 비행용 태그: BLE 중계 보드(nucleo/n6_bridge)가 하던 일을 이 펌웨어가 같이 한다.
 *
 *   노트북 --BLE--> XIAO --USART2 115200 (PA2 TX, PF6 RX)--> [이 보드] --UART7 921600 (PH4 TX, PE7 RX)--> 비행 제어기
 *
 * n6_bridge 는 주 반복문에서 UART 를 계속 들여다봤다. 이 펌웨어의 주 반복문은 무선을 다루느라 그럴 수 없어서
 * (UART7 의 수신 버퍼 8바이트는 87 µs 면 찬다) 수신과 송신을 인터럽트로 한다. 인터럽트는 바이트를 고리 버퍼에
 * 넣고 빼는 일만 하고, 메시지를 푸는 일(fcl.c)은 프레임의 조용한 구간에 주 반복문이 한다.
 *
 * 핀과 UART 설정은 n6_bridge/FSBL/Src/stm32n6xx_hal_msp.c, main.c 와 같다(USART2: AF7, UART7: AF8, 8N1, FIFO 켬,
 * 클록은 PCLK1). CubeMX 로 만든 것이 아니라 손으로 옮겼다.
 *
 * 기록 줄 (fcl.c 의 C, E 줄에 더해)
 *   U,<프레임>,<FC 수신 바이트>,<FC 송신>,<BLE 수신>,<BLE 송신>,<FC 수신 넘침>,<BLE 송신 못 넣음>,<FC 수신 오류>,<BLE 수신 오류>
 *      10초마다. 넘침과 오류가 늘면 중계가 바이트를 잃고 있는 것이다.
 */
#include <stdio.h>
#include <string.h>
#include <port.h>
#include "fcl.h"
#include "flight.h"

enum { FC_RX = 4096, FC_TX = 1024, BLE_RX = 1024, BLE_TX = 4096, CHUNK = 96, STATS_MS = 10000, UART_IRQ_PRIORITY = 2 };

struct ring {
    uint8_t *data;
    uint32_t size;             /* 2의 거듭제곱 */
    volatile uint32_t head, tail;
};

struct link {
    UART_HandleTypeDef handle;
    struct ring rx, tx;
    volatile uint32_t rx_bytes, tx_bytes, rx_overflow, errors;
    uint32_t tx_refused;
};

static uint8_t fc_rx_data[FC_RX], fc_tx_data[FC_TX], ble_rx_data[BLE_RX], ble_tx_data[BLE_TX];
static struct link fc = { .rx = { fc_rx_data, FC_RX, 0, 0 }, .tx = { fc_tx_data, FC_TX, 0, 0 } };
static struct link ble = { .rx = { ble_rx_data, BLE_RX, 0, 0 }, .tx = { ble_tx_data, BLE_TX, 0, 0 } };
static bool ready;
static uint32_t last_tick_ms, last_stats_ms, last_frame;
static char stats_line[120];
static bool stats_pending;

#define USART_ERRORS (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)
#define USART_ERROR_CLEAR (USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF)

/* 인터럽트: 받은 바이트를 고리에 넣고, 보낼 바이트를 고리에서 꺼낸다 */
static void link_isr(struct link *l)
{
    USART_TypeDef *u = l->handle.Instance;
    if ((u->ISR & USART_ERRORS) != 0U) {
        ++l->errors;
        u->ICR = USART_ERROR_CLEAR;
    }
    while ((u->ISR & USART_ISR_RXNE_RXFNE) != 0U) {
        const uint8_t byte = (uint8_t)u->RDR;
        if (l->rx.head - l->rx.tail >= l->rx.size) {
            ++l->rx_overflow;
        } else {
            l->rx.data[l->rx.head & (l->rx.size - 1U)] = byte;
            l->rx.head = l->rx.head + 1U;
            ++l->rx_bytes;
        }
    }
    while ((u->CR1 & USART_CR1_TXEIE_TXFNFIE) != 0U && (u->ISR & USART_ISR_TXE_TXFNF) != 0U) {
        if (l->tx.head == l->tx.tail) {
            u->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
            break;
        }
        u->TDR = l->tx.data[l->tx.tail & (l->tx.size - 1U)];
        l->tx.tail = l->tx.tail + 1U;
        ++l->tx_bytes;
    }
}

void USART2_IRQHandler(void) { link_isr(&ble); }
void UART7_IRQHandler(void) { link_isr(&fc); }

/* 메시지 하나를 통째로 송신 고리에 넣는다. 자리가 모자라면 넣지 않는다 */
static bool link_send(struct link *l, const uint8_t *data, unsigned length)
{
    if (!ready || l->tx.size - (l->tx.head - l->tx.tail) < length) {
        ++l->tx_refused;
        return false;
    }
    uint32_t head = l->tx.head;
    for (unsigned i = 0; i < length; ++i) {
        l->tx.data[head++ & (l->tx.size - 1U)] = data[i];
    }
    l->tx.head = head;
    __disable_irq();
    l->handle.Instance->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    __enable_irq();
    return true;
}

static unsigned link_receive(struct link *l, uint8_t *out, unsigned max)
{
    unsigned n = 0;
    while (n < max && l->rx.tail != l->rx.head) {
        out[n++] = l->rx.data[l->rx.tail & (l->rx.size - 1U)];
        l->rx.tail = l->rx.tail + 1U;
    }
    return n;
}

static bool to_fc(const uint8_t *data, unsigned length) { return link_send(&fc, data, length); }
static bool to_ble(const uint8_t *data, unsigned length) { return link_send(&ble, data, length); }

static bool uart_start(struct link *l, USART_TypeDef *instance, uint32_t baud, IRQn_Type irq)
{
    UART_HandleTypeDef *h = &l->handle;
    h->Instance = instance;
    h->Init.BaudRate = baud;
    h->Init.WordLength = UART_WORDLENGTH_8B;
    h->Init.StopBits = UART_STOPBITS_1;
    h->Init.Parity = UART_PARITY_NONE;
    h->Init.Mode = UART_MODE_TX_RX;
    h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    h->Init.OverSampling = UART_OVERSAMPLING_16;
    h->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    h->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    h->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(h) != HAL_OK || HAL_UARTEx_SetTxFifoThreshold(h, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_SetRxFifoThreshold(h, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_EnableFifoMode(h) != HAL_OK) {
        return false;
    }
    instance->ICR = USART_ERROR_CLEAR;
    instance->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
    HAL_NVIC_SetPriority(irq, UART_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(irq);
    return true;
}

void flight_init(uint8_t tag_id, uint32_t frame_us)
{
    static const struct fcl_io io = { to_fc, to_ble };
    RCC_PeriphCLKInitTypeDef clock = { 0 };
    GPIO_InitTypeDef pin = { 0 };
    fcl_init(tag_id, frame_us, &io);

    clock.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    clock.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    bool ok = HAL_RCCEx_PeriphCLKConfig(&clock) == HAL_OK;
    clock.PeriphClockSelection = RCC_PERIPHCLK_UART7;
    clock.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
    ok = ok && HAL_RCCEx_PeriphCLKConfig(&clock) == HAL_OK;
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_UART7_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    pin.Mode = GPIO_MODE_AF_PP;
    pin.Pull = GPIO_PULLUP;
    pin.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    pin.Alternate = GPIO_AF7_USART2;
    pin.Pin = GPIO_PIN_2;                /* PA2 = USART2_TX -> XIAO D7 */
    HAL_GPIO_Init(GPIOA, &pin);
    pin.Pin = GPIO_PIN_6;                /* PF6 = USART2_RX <- XIAO D6 */
    HAL_GPIO_Init(GPIOF, &pin);
    pin.Alternate = GPIO_AF8_UART7;
    pin.Pin = GPIO_PIN_4;                /* PH4 = UART7_TX -> 비행 제어기 RX */
    HAL_GPIO_Init(GPIOH, &pin);
    pin.Pin = GPIO_PIN_7;                /* PE7 = UART7_RX <- 비행 제어기 TX */
    HAL_GPIO_Init(GPIOE, &pin);

    ok = ok && uart_start(&ble, USART2, 115200U, USART2_IRQn);
    ok = ok && uart_start(&fc, UART7, 921600U, UART7_IRQn);
    ready = ok;
    last_tick_ms = last_stats_ms = HAL_GetTick();
    snprintf(stats_line, sizeof(stats_line), "E,0,flight_uart,%d,%lu\r\n", (int)ok,
             (unsigned long)(fc.handle.Instance != NULL ? fc.handle.Instance->BRR : 0U));
    stats_pending = true;
}

bool flight_service(uint32_t now_us)
{
    uint8_t chunk[CHUNK];
    const uint32_t now_ms = HAL_GetTick();
    (void)now_us;
    if (!ready) {
        return false;
    }
    unsigned n = link_receive(&fc, chunk, sizeof(chunk));
    if (n != 0) {
        fcl_from_fc(chunk, n, now_ms);
    }
    const unsigned m = link_receive(&ble, chunk, sizeof(chunk));
    if (m != 0) {
        fcl_from_ble(chunk, m, now_ms);
    }
    if (now_ms != last_tick_ms) {
        last_tick_ms = now_ms;
        fcl_tick(now_ms);
        if (!stats_pending && (uint32_t)(now_ms - last_stats_ms) >= STATS_MS) {
            last_stats_ms = now_ms;
            snprintf(stats_line, sizeof(stats_line), "U,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
                     (unsigned long)last_frame, (unsigned long)fc.rx_bytes, (unsigned long)fc.tx_bytes,
                     (unsigned long)ble.rx_bytes, (unsigned long)ble.tx_bytes, (unsigned long)fc.rx_overflow,
                     (unsigned long)ble.tx_refused, (unsigned long)fc.errors, (unsigned long)ble.errors);
            stats_pending = true;
        }
    }
    return n != 0 || m != 0;
}

void flight_control(uint32_t frame, const struct uwbf_control *ctl, bool log_ok)
{
    last_frame = frame;
    if (ready) {
        fcl_control(frame, ctl, log_ok, HAL_GetTick());
    }
}

unsigned flight_take_line(char *out, unsigned size)
{
    if (stats_pending) {
        const unsigned n = (unsigned)strlen(stats_line);
        stats_pending = false;
        if (n < size) {
            memcpy(out, stats_line, n);
            return n;
        }
    }
    return fcl_take_line(out, size);
}
