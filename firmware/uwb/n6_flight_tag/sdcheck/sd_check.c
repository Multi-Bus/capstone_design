/* 캐리어 보드의 microSD(SDMMC2, 4비트) 확인용. NUCLEO-N657X0-Q 의 RAM 에서 돌린다. 외부 플래시는 건드리지 않는다.
 *
 * 배선(캐리어 넷리스트): CLK PC2, CMD PC3, D0 PC4, D1 PC5, D2 PC0, D3 PE4, 카드 감지 PB8(꽂히면 Low),
 *                       카드 전원 켜기 PB9(High 면 켜짐). 핀과 초기화 순서는 ST 의 STM32N6570-DK SD 예제와 같다.
 *
 * 하는 일 (결과는 ST-LINK VCP 115200 으로 나온다)
 *   1) 카드 감지 핀을 읽고, 카드 전원을 껐다 켠다
 *   2) 카드를 초기화하고 종류와 크기를 읽는다 (12.5 MHz, 4비트)
 *   3) 0번 섹터를 읽어 파티션과 파일 시스템 종류를 본다
 *   4) 1 MiB 를 읽어 속도를 잰다. 25 MHz 로도 한 번 더 읽는다
 *   5) 쓰기 지연: 카드 끝의 섹터들을 읽어서 **같은 내용을 그대로 다시 쓴다**(내용은 바뀌지 않는다).
 *      한 섹터씩 256번, 8섹터씩 64번. 한 번에 걸린 시간의 분포를 낸다. SDCHECK_WRITE 0 으로 빌드하면 건너뛴다.
 *      먼저 그 구역을 읽어 보고, 비어 있지 않으면(0x00 이나 0xFF 가 아닌 값이 있으면) 쓰지 않고 건너뛴다.
 *      그래도 쓰려면 SDCHECK_WRITE 2 로 빌드한다.
 *   6) 카드 전원을 끄면 초기화가 실패하고, 다시 켜면 되는지 본다
 *
 * 줄마다 SD,<항목>,<값...> 꼴이고 마지막 줄은 SD,DONE,pass=<통과 수>,fail=<실패 수> 이다.
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <port.h>

#ifndef SDCHECK_WRITE
#define SDCHECK_WRITE 1
#endif
#define SD_EN_PORT GPIOB
#define SD_EN_PIN GPIO_PIN_9
#define SD_DET_PORT GPIOB
#define SD_DET_PIN GPIO_PIN_8

enum { BLOCK = 512, READ_CHUNK = 64, READ_TOTAL = 2048, REWRITE_SINGLE = 256, REWRITE_MULTI = 64, MULTI = 8 };

static SD_HandleTypeDef hsd;
static uint8_t buffer[READ_CHUNK * BLOCK] __attribute__((aligned(32)));
#if SDCHECK_WRITE
static uint8_t check[MULTI * BLOCK] __attribute__((aligned(32)));
#endif
static unsigned passed, failed;

static void say(const char *format, ...)
{
    char line[160];
    va_list args;
    va_start(args, format);
    vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    test_run_info((unsigned char *)line);
}

static void verdict(const char *name, int ok)
{
    say("SD,%s,%s", name, ok ? "OK" : "FAIL");
    if (ok) {
        ++passed;
    } else {
        ++failed;
    }
}

/* CPU 사이클 카운터로 µs 를 잰다 (port_init 이 켜 둔다). 7초 안쪽의 간격만 */
static uint32_t cycles(void)
{
    return DWT->CYCCNT;
}

static uint32_t us_since(uint32_t start)
{
    return (DWT->CYCCNT - start) / (SystemCoreClock / 1000000U);
}

void HAL_SD_MspInit(SD_HandleTypeDef *handle)
{
    GPIO_InitTypeDef gpio = { 0 };
    (void)handle;
    HAL_PWREx_EnableVddIO5(); /* PC0, PC2~PC5, PE4 가 속한 전원 영역 */
    __HAL_RCC_SDMMC2_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF11_SDMMC2;
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5; /* D2, CLK, CMD, D0, D1 */
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_4; /* D3 */
    HAL_GPIO_Init(GPIOE, &gpio);
}

/* ST 의 DK 예제(stm32n6570_discovery_sd.c)와 같은 순서: SDMMC2 를 CPU 와 같은 보안 영역에 두고 리셋한다 */
static void sdmmc2_isolation(void)
{
    volatile uint32_t *rimc = (volatile uint32_t *)0x54024C1CUL;   /* RIFSC_RIMC_ATTR, SDMMC2 */
    volatile uint32_t *seccfgr1 = (volatile uint32_t *)0x54024014UL; /* RIFSC_RISC_SECCFGR1 */
    *rimc = 0x00000310UL;
    (void)*rimc;
    __HAL_RCC_SDMMC2_CLK_ENABLE();
    __HAL_RCC_SDMMC2_FORCE_RESET();
    __HAL_RCC_SDMMC2_RELEASE_RESET();
    *seccfgr1 |= 0x00400000UL;
    (void)*seccfgr1;
}

static void card_power(int on)
{
    HAL_GPIO_WritePin(SD_EN_PORT, SD_EN_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int wait_ready(uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - start > timeout_ms) {
            return 0;
        }
    }
    return 1;
}

static int card_init(uint32_t clock_hz, uint32_t *actual_hz)
{
    const uint32_t kernel = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC2);
    uint32_t div = (kernel + 2U * clock_hz - 1U) / (2U * clock_hz);
    if (div < 1U) {
        div = 1U;
    }
    if (hsd.Instance != NULL) {
        (void)HAL_SD_DeInit(&hsd); /* 처음에는 아직 초기화한 적이 없다 */
    }
    sdmmc2_isolation();
    memset(&hsd, 0, sizeof(hsd));
    hsd.Instance = SDMMC2;
    hsd.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsd.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDMMC_BUS_WIDE_1B;
    /* 인터럽트나 DMA 없이 읽으므로, 수신 버퍼가 차면 칩이 클록을 멈추게 한다. 끄면 여러 섹터를 읽을 때
     * 수신 넘침(오류 0x20)이 난다 (2026-11-24 이 보드에서 확인) */
    hsd.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
    hsd.Init.ClockDiv = div;
    *actual_hz = kernel / (2U * div);
    if (HAL_SD_Init(&hsd) != HAL_OK) {
        say("SD,init_error,step=init,code=0x%08lX,kernel_hz=%lu", (unsigned long)hsd.ErrorCode, (unsigned long)kernel);
        return 0;
    }
    if (HAL_SD_ConfigWideBusOperation(&hsd, SDMMC_BUS_WIDE_4B) != HAL_OK) {
        say("SD,init_error,step=4bit,code=0x%08lX", (unsigned long)hsd.ErrorCode);
        return 0;
    }
    /* HAL 이 카드 종류에 따라 클록을 다시 정했을 수 있다. 실제 분주값을 읽는다 */
    div = hsd.Instance->CLKCR & SDMMC_CLKCR_CLKDIV;
    *actual_hz = div ? kernel / (2U * div) : kernel;
    return wait_ready(1000);
}

static int read_speed(uint32_t first, uint32_t *kb_per_s)
{
    const uint32_t start = cycles();
    for (uint32_t done = 0; done < READ_TOTAL; done += READ_CHUNK) {
        if (HAL_SD_ReadBlocks(&hsd, buffer, first + done, READ_CHUNK, 2000) != HAL_OK || !wait_ready(2000)) {
            say("SD,read_error,block=%lu,code=0x%08lX", (unsigned long)(first + done), (unsigned long)hsd.ErrorCode);
            return 0;
        }
    }
    const uint32_t us = us_since(start);
    *kb_per_s = us ? (uint32_t)(((uint64_t)READ_TOTAL * BLOCK * 1000000ULL / us) / 1024ULL) : 0;
    return 1;
}

#if SDCHECK_WRITE
/* 인터럽트로 쓰기: 쓰기를 걸어 놓고 돌아온 뒤, 끝났다는 표시만 본다 (기록 펌웨어가 쓸 방식) */
static volatile uint8_t it_done, it_error;

void SDMMC2_IRQHandler(void)
{
    HAL_SD_IRQHandler(&hsd);
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *handle)
{
    (void)handle;
    it_done = 1;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *handle)
{
    (void)handle;
    it_error = 1;
}

/* 같은 내용을 다시 쓰되 단계별로 시간을 잰다.
 *   call  = 쓰기 함수를 부른 때부터 돌아올 때까지 (이 동안 다른 일을 못 한다)
 *   done  = 부른 때부터 자료 전송이 끝났다는 표시까지
 *   ready = 부른 때부터 카드가 다음 명령을 받을 수 있을 때까지
 *   stall = 기다리는 동안 주 루프가 가장 오래 멈춘 시간 (인터럽트 처리에 든 시간) */
static int timing_test(const char *name, uint32_t first, unsigned count, unsigned blocks, int use_it)
{
    uint32_t max_call = 0, max_done = 0, max_ready = 0, max_stall = 0;
    uint64_t sum_call = 0, sum_done = 0, sum_ready = 0;
    for (unsigned i = 0; i < count; ++i) {
        const uint32_t lba = first + i * blocks;
        if (HAL_SD_ReadBlocks(&hsd, buffer, lba, blocks, 2000) != HAL_OK || !wait_ready(2000)) {
            say("SD,%s_error,step=read,block=%lu,code=0x%08lX", name, (unsigned long)lba, (unsigned long)hsd.ErrorCode);
            return 0;
        }
        it_done = it_error = 0;
        const uint32_t start = cycles();
        const HAL_StatusTypeDef status = use_it ? HAL_SD_WriteBlocks_IT(&hsd, buffer, lba, blocks)
                                                : HAL_SD_WriteBlocks(&hsd, buffer, lba, blocks, 5000);
        const uint32_t call = us_since(start);
        if (status != HAL_OK) {
            say("SD,%s_error,step=write,block=%lu,code=0x%08lX", name, (unsigned long)lba, (unsigned long)hsd.ErrorCode);
            return 0;
        }
        uint32_t done = call, last = cycles();
        if (use_it) {
            while (!it_done && !it_error && us_since(start) < 3000000U) {
                const uint32_t gap = us_since(last);
                last = cycles();
                max_stall = gap > max_stall ? gap : max_stall;
            }
            done = us_since(start);
            if (!it_done) {
                say("SD,%s_error,step=wait,block=%lu,code=0x%08lX,error_flag=%u", name, (unsigned long)lba,
                    (unsigned long)hsd.ErrorCode, (unsigned)it_error);
                return 0;
            }
        }
        if (!wait_ready(5000)) {
            say("SD,%s_error,step=ready,block=%lu", name, (unsigned long)lba);
            return 0;
        }
        const uint32_t ready = us_since(start);
        if (HAL_SD_ReadBlocks(&hsd, check, lba, blocks, 2000) != HAL_OK || !wait_ready(2000) ||
            memcmp(buffer, check, blocks * BLOCK) != 0) {
            say("SD,%s_error,step=verify,block=%lu", name, (unsigned long)lba);
            return 0;
        }
        max_call = call > max_call ? call : max_call;
        max_done = done > max_done ? done : max_done;
        max_ready = ready > max_ready ? ready : max_ready;
        sum_call += call;
        sum_done += done;
        sum_ready += ready;
    }
    say("SD,%s,count=%u,blocks=%u,call_us=%lu/%lu,done_us=%lu/%lu,ready_us=%lu/%lu,max_stall_us=%lu  (mean/max)", name,
        count, blocks, (unsigned long)(sum_call / count), (unsigned long)max_call, (unsigned long)(sum_done / count),
        (unsigned long)max_done, (unsigned long)(sum_ready / count), (unsigned long)max_ready, (unsigned long)max_stall);
    return 1;
}

/* first 부터 blocks 섹터에 0x00, 0xFF 가 아닌 값이 든 섹터 수. 읽기 오류면 -1 */
static int32_t count_used(uint32_t first, uint32_t blocks)
{
    int32_t used = 0;
    for (uint32_t done = 0; done < blocks; done += READ_CHUNK) {
        if (HAL_SD_ReadBlocks(&hsd, buffer, first + done, READ_CHUNK, 2000) != HAL_OK || !wait_ready(2000)) {
            return -1;
        }
        for (unsigned b = 0; b < READ_CHUNK; ++b) {
            for (unsigned i = 0; i < BLOCK; ++i) {
                const uint8_t v = buffer[b * BLOCK + i];
                if (v != 0x00 && v != 0xFF) {
                    ++used;
                    break;
                }
            }
        }
    }
    return used;
}

/* first 부터 count 번, 한 번에 blocks 섹터씩: 읽고, 같은 내용을 다시 쓰고, 다시 읽어 견준다 */
static int rewrite_test(const char *name, uint32_t first, unsigned count, unsigned blocks)
{
    uint32_t min_us = 0xFFFFFFFFUL, max_us = 0, over_20ms = 0, over_100ms = 0;
    uint64_t sum_us = 0;
    for (unsigned i = 0; i < count; ++i) {
        const uint32_t lba = first + i * blocks;
        if (HAL_SD_ReadBlocks(&hsd, buffer, lba, blocks, 2000) != HAL_OK || !wait_ready(2000)) {
            say("SD,%s_error,step=read,block=%lu,code=0x%08lX", name, (unsigned long)lba, (unsigned long)hsd.ErrorCode);
            return 0;
        }
        const uint32_t start = cycles();
        if (HAL_SD_WriteBlocks(&hsd, buffer, lba, blocks, 5000) != HAL_OK || !wait_ready(5000)) {
            say("SD,%s_error,step=write,block=%lu,code=0x%08lX", name, (unsigned long)lba, (unsigned long)hsd.ErrorCode);
            return 0;
        }
        const uint32_t us = us_since(start);
        if (HAL_SD_ReadBlocks(&hsd, check, lba, blocks, 2000) != HAL_OK || !wait_ready(2000) ||
            memcmp(buffer, check, blocks * BLOCK) != 0) {
            say("SD,%s_error,step=verify,block=%lu,code=0x%08lX", name, (unsigned long)lba, (unsigned long)hsd.ErrorCode);
            return 0;
        }
        min_us = us < min_us ? us : min_us;
        max_us = us > max_us ? us : max_us;
        sum_us += us;
        over_20ms += us > 20000U;
        over_100ms += us > 100000U;
    }
    say("SD,%s,count=%u,blocks=%u,min_us=%lu,mean_us=%lu,max_us=%lu,over_20ms=%lu,over_100ms=%lu", name, count, blocks,
        (unsigned long)min_us, (unsigned long)(sum_us / count), (unsigned long)max_us, (unsigned long)over_20ms,
        (unsigned long)over_100ms);
    return 1;
}
#endif

/* This entry is called by the handoff project's TEST_DS_TWR_RESPONDER selection. */
int ds_twr_responder(void)
{
    GPIO_InitTypeDef gpio = { 0 };
    HAL_SD_CardInfoTypeDef info = { 0 };
    uint32_t hz = 0, speed = 0;

    say("SD,START,fw=sdcheck-v1,write_test=%u", (unsigned)SDCHECK_WRITE);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = SD_DET_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL; /* 캐리어에 100 kΩ 풀업이 있다 */
    HAL_GPIO_Init(SD_DET_PORT, &gpio);
    card_power(0);
    gpio.Pin = SD_EN_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SD_EN_PORT, &gpio);

    /* 1) 카드 감지, 전원 */
    const int detect_low = HAL_GPIO_ReadPin(SD_DET_PORT, SD_DET_PIN) == GPIO_PIN_RESET;
    say("SD,detect,pin=%d,%s", detect_low ? 0 : 1, detect_low ? "card_present" : "no_card_or_no_3V3_pullup");
    verdict("card_detect", detect_low);
    Sleep(300);
    say("SD,step,power_enable_high");
    card_power(1);
    Sleep(300);
    say("SD,step,power_is_on,detect_pin=%d", HAL_GPIO_ReadPin(SD_DET_PORT, SD_DET_PIN) == GPIO_PIN_SET);
    say("SD,step,kernel_hz=%lu", (unsigned long)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC2));

    /* 2) 초기화 */
    int ok = card_init(12500000U, &hz);
    verdict("init_12M5", ok);
    if (ok) {
        HAL_SD_GetCardInfo(&hsd, &info);
        say("SD,card,type=%lu,version=%lu,class=0x%lX,blocks=%lu,block_size=%lu,size_MiB=%lu,clock_hz=%lu",
            (unsigned long)info.CardType, (unsigned long)info.CardVersion, (unsigned long)info.Class,
            (unsigned long)info.LogBlockNbr, (unsigned long)info.LogBlockSize,
            (unsigned long)(((uint64_t)info.LogBlockNbr * info.LogBlockSize) >> 20), (unsigned long)hz);
    }

    /* 3) 0번 섹터 */
    if (ok) {
        ok = HAL_SD_ReadBlocks(&hsd, buffer, 0, 1, 2000) == HAL_OK && wait_ready(2000);
        verdict("read_sector0", ok);
    }
    if (ok) {
        const int signature = buffer[510] == 0x55 && buffer[511] == 0xAA;
        const uint8_t type = buffer[446 + 4];
        const uint32_t start = (uint32_t)buffer[446 + 8] | ((uint32_t)buffer[446 + 9] << 8) |
                               ((uint32_t)buffer[446 + 10] << 16) | ((uint32_t)buffer[446 + 11] << 24);
        say("SD,sector0,signature_55AA=%d,partition_type=0x%02X,partition_start=%lu", signature, type,
            (unsigned long)start);
        if (signature && start != 0 && start < info.LogBlockNbr &&
            HAL_SD_ReadBlocks(&hsd, buffer, start, 1, 2000) == HAL_OK && wait_ready(2000)) {
            char fat32[9] = { 0 }, oem[9] = { 0 };
            memcpy(oem, buffer + 3, 8);
            memcpy(fat32, buffer + 82, 8);
            for (unsigned i = 0; i < 8; ++i) {
                oem[i] = (oem[i] >= 32 && oem[i] < 127) ? oem[i] : '.';
                fat32[i] = (fat32[i] >= 32 && fat32[i] < 127) ? fat32[i] : '.';
            }
            say("SD,filesystem,oem=%s,label_at_82=%s,sectors_per_cluster=%u", oem, fat32, (unsigned)buffer[13]);
        }
    }

    /* 4) 읽기 속도 */
    if (ok) {
        ok = read_speed(8192, &speed);
        verdict("read_1MiB_12M5", ok);
        if (ok) {
            say("SD,read_speed,clock_hz=%lu,kB_per_s=%lu", (unsigned long)hz, (unsigned long)speed);
        }
    }
    if (ok) {
        int fast = card_init(25000000U, &hz) && read_speed(8192, &speed);
        verdict("read_1MiB_25M", fast);
        if (fast) {
            say("SD,read_speed,clock_hz=%lu,kB_per_s=%lu", (unsigned long)hz, (unsigned long)speed);
        }
        ok = card_init(12500000U, &hz); /* 쓰기 시험은 느린 클록으로 */
        verdict("reinit_12M5", ok);
    }

#if SDCHECK_WRITE
    /* 5) 쓰기 지연: 카드 끝 4 MiB 안의 섹터를 같은 내용으로 다시 쓴다 */
    if (ok && info.LogBlockNbr > 16384U) {
        const uint32_t base = info.LogBlockNbr - 8192U;
        const int32_t used = count_used(base, 2048U);
        say("SD,rewrite_region,first_block=%lu,blocks=2048,blocks_with_data=%ld", (unsigned long)base, (long)used);
        if (used != 0 && SDCHECK_WRITE < 2) {
            say("SD,rewrite_skipped,%s", used < 0 ? "read_error" : "region_has_data");
        } else {
            ok = rewrite_test("rewrite_1_block", base, REWRITE_SINGLE, 1);
            verdict("rewrite_single", ok);
            if (ok) {
                ok = rewrite_test("rewrite_8_blocks", base + 1024U, REWRITE_MULTI, MULTI);
                verdict("rewrite_multi", ok);
            }
            if (ok) {
                ok = timing_test("blocking_1_block", base, REWRITE_SINGLE, 1, 0);
                verdict("timing_blocking", ok);
            }
            if (ok) {
                HAL_NVIC_SetPriority(SDMMC2_IRQn, 5, 0);
                HAL_NVIC_EnableIRQ(SDMMC2_IRQn);
                ok = timing_test("interrupt_1_block", base, REWRITE_SINGLE, 1, 1);
                verdict("timing_interrupt_1", ok);
                if (ok) {
                    ok = timing_test("interrupt_8_blocks", base + 1024U, REWRITE_MULTI, MULTI, 1);
                    verdict("timing_interrupt_8", ok);
                }
                HAL_NVIC_DisableIRQ(SDMMC2_IRQn);
            }
        }
    }
#endif

    /* 6) 카드 전원 스위치 */
    card_power(0);
    Sleep(500);
    const int off_fails = !card_init(12500000U, &hz);
    verdict("init_fails_when_power_off", off_fails);
    card_power(1);
    Sleep(300);
    verdict("init_after_power_on", card_init(12500000U, &hz));

    say("SD,DONE,pass=%u,fail=%u", passed, failed);
    for (;;) {
        Sleep(1000);
    }
}
