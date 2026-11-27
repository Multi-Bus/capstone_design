#include <string.h>
#include <port.h>
#include "sdfat.h"
#include "sdlog.h"

#define SD_EN_PORT GPIOB
#define SD_EN_PIN GPIO_PIN_9
#define SD_DET_PORT GPIOB
#define SD_DET_PIN GPIO_PIN_8
#define SDLOG_FILE "UWBLOG  BIN"

enum {
    BLOCK = 512,
    RING = 64,                /* 쌓아 두는 섹터 수 (32 KB) */
    CLOCK_HZ = 12500000,
    FLUSH_MS = 500,           /* 이 시간 동안 섹터가 안 차면 줄바꿈으로 채워서 쓴다 */
    WRITE_TIMEOUT_MS = 2000,
    RETRIES = 3,
};

static SD_HandleTypeDef hsd;
static uint8_t ring[RING][BLOCK] __attribute__((aligned(32)));
static uint32_t head, tail;   /* head 섹터에 채우는 중, tail 부터 쓴다 */
static uint32_t fill;         /* head 섹터에 찬 글자 수 */
static uint32_t last_append_ms, write_start_ms, write_start_cycles;
static uint8_t retries;
static volatile uint8_t tx_done, tx_error;
static struct sdlog_stats stats;

static uint32_t us_since(uint32_t start)
{
    return (DWT->CYCCNT - start) / (SystemCoreClock / 1000000U);
}

void SDMMC2_IRQHandler(void)
{
    HAL_SD_IRQHandler(&hsd);
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *handle)
{
    (void)handle;
    tx_done = 1;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *handle)
{
    (void)handle;
    tx_error = 1;
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

/* 카드를 껐다 켜고 초기화한다 (ST 의 STM32N6570-DK 예제와 같은 순서). 수백 ms 걸린다 */
static int card_start(void)
{
    volatile uint32_t *rimc = (volatile uint32_t *)0x54024C1CUL;     /* RIFSC_RIMC_ATTR, SDMMC2 */
    volatile uint32_t *seccfgr1 = (volatile uint32_t *)0x54024014UL; /* RIFSC_RISC_SECCFGR1 */
    GPIO_InitTypeDef gpio = { 0 };
    HAL_NVIC_DisableIRQ(SDMMC2_IRQn);
    if (hsd.Instance != NULL) {
        (void)HAL_SD_DeInit(&hsd);
    }
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = SD_DET_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SD_DET_PORT, &gpio);
    HAL_GPIO_WritePin(SD_EN_PORT, SD_EN_PIN, GPIO_PIN_RESET);
    gpio.Pin = SD_EN_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SD_EN_PORT, &gpio);
    HAL_Delay(250);
    HAL_GPIO_WritePin(SD_EN_PORT, SD_EN_PIN, GPIO_PIN_SET);
    HAL_Delay(250);

    *rimc = 0x00000310UL;
    (void)*rimc;
    __HAL_RCC_SDMMC2_CLK_ENABLE();
    __HAL_RCC_SDMMC2_FORCE_RESET();
    __HAL_RCC_SDMMC2_RELEASE_RESET();
    *seccfgr1 |= 0x00400000UL;
    (void)*seccfgr1;

    const uint32_t kernel = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC2);
    uint32_t div = (kernel + 2U * CLOCK_HZ - 1U) / (2U * CLOCK_HZ);
    memset(&hsd, 0, sizeof(hsd));
    hsd.Instance = SDMMC2;
    hsd.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsd.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDMMC_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE; /* 끄면 여러 섹터를 읽을 때 수신 넘침이 난다 */
    hsd.Init.ClockDiv = div < 1U ? 1U : div;
    if (HAL_SD_Init(&hsd) != HAL_OK || HAL_SD_ConfigWideBusOperation(&hsd, SDMMC_BUS_WIDE_4B) != HAL_OK ||
        !wait_ready(1000)) {
        stats.init_error = hsd.ErrorCode ? hsd.ErrorCode : 0xFFFFFFFFUL;
        stats.card_ok = 0;
        return 0;
    }
    HAL_NVIC_SetPriority(SDMMC2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC2_IRQn);
    stats.card_ok = 1;
    stats.init_error = 0;
    return 1;
}

int sdlog_read_block(uint32_t block, uint8_t *buffer)
{
    return (HAL_SD_ReadBlocks(&hsd, buffer, block, 1, 2000) == HAL_OK && wait_ready(2000)) ? 0 : 1;
}

static void begin(uint32_t first_block, uint32_t blocks, uint32_t next)
{
    stats.first_block = first_block;
    stats.file_blocks = blocks;
    stats.next_block = next;
    head = tail = fill = 0;
    retries = 0;
    last_append_ms = HAL_GetTick();
    stats.state = next >= blocks ? SDLOG_FULL : SDLOG_IDLE;
}

int sdlog_init(void)
{
    struct sdfat_file file;
    memset(&stats, 0, sizeof(stats));
    stats.state = SDLOG_OFF;
    stats.file_result = SDFAT_READ_ERROR;
    if (!card_start()) {
        return 0;
    }
    stats.file_result = (uint8_t)sdfat_locate(sdlog_read_block, SDLOG_FILE, &file);
    if (stats.file_result != SDFAT_OK) {
        return 0;
    }
    const uint32_t end = sdfat_find_end(sdlog_read_block, &file);
    if (end == 0xFFFFFFFFUL) {
        stats.file_result = SDFAT_READ_ERROR;
        return 0;
    }
    begin(file.first_block, file.blocks, end);
    return stats.state == SDLOG_IDLE;
}

int sdlog_use_region(uint32_t first_block, uint32_t blocks)
{
    if (!stats.card_ok) {
        return 0;
    }
    begin(first_block, blocks, 0);
    return 1;
}

void sdlog_write(const char *text, unsigned length)
{
    if (stats.state == SDLOG_OFF || stats.state == SDLOG_FULL || stats.state == SDLOG_FAILED) {
        return;
    }
    last_append_ms = HAL_GetTick();
    while (length > 0) {
        if (head - tail >= RING) {
            stats.dropped_bytes += length; /* 쌓을 자리가 없다 */
            return;
        }
        const unsigned room = BLOCK - fill;
        const unsigned n = length < room ? length : room;
        memcpy(ring[head % RING] + fill, text, n);
        fill += n;
        text += n;
        length -= n;
        if (fill == BLOCK) {
            ++head;
            fill = 0;
        }
    }
}

static void write_failed(void)
{
    ++stats.errors;
    stats.state = ++retries > RETRIES ? SDLOG_FAILED : SDLOG_RECOVER;
}

void sdlog_service(int may_start)
{
    stats.queued = head - tail;
    if (stats.state == SDLOG_WRITING) {
        if (tx_done) {
            stats.state = SDLOG_WAIT_READY;
        } else if (tx_error || HAL_GetTick() - write_start_ms > WRITE_TIMEOUT_MS) {
            write_failed();
        }
        return;
    }
    if (!may_start) {
        return;
    }
    if (stats.state == SDLOG_WAIT_READY) {
        if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER) {
            const uint32_t us = us_since(write_start_cycles);
            if (us > stats.max_write_us && HAL_GetTick() - write_start_ms < 6000U) {
                stats.max_write_us = us;
            }
            ++stats.written;
            ++stats.next_block;
            ++tail;
            retries = 0;
            stats.state = stats.next_block >= stats.file_blocks ? SDLOG_FULL : SDLOG_IDLE;
        } else if (HAL_GetTick() - write_start_ms > WRITE_TIMEOUT_MS) {
            write_failed();
        }
        return;
    }
    if (stats.state == SDLOG_RECOVER) {
        /* 카드를 껐다 켠다. 수백 ms 동안 다른 일을 못 하지만, 안 하면 남은 기록을 모두 잃는다 */
        stats.state = card_start() ? SDLOG_IDLE : SDLOG_FAILED;
        return;
    }
    if (stats.state != SDLOG_IDLE) {
        return;
    }
    if (tail == head && fill > 0 && HAL_GetTick() - last_append_ms >= FLUSH_MS) {
        memset(ring[head % RING] + fill, '\n', BLOCK - fill); /* 줄 끝에서 끊긴 조각이다. 줄바꿈으로 채운다 */
        ++head;
        fill = 0;
    }
    if (tail == head) {
        return;
    }
    tx_done = tx_error = 0;
    write_start_ms = HAL_GetTick();
    write_start_cycles = DWT->CYCCNT;
    if (HAL_SD_WriteBlocks_IT(&hsd, ring[tail % RING], stats.first_block + stats.next_block, 1) != HAL_OK) {
        write_failed();
    } else {
        stats.state = SDLOG_WRITING;
    }
    const uint32_t us = us_since(write_start_cycles);
    if (us > stats.max_start_us) {
        stats.max_start_us = us;
    }
}

void sdlog_flush_blocking(uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();
    if (fill > 0 && head - tail < RING) {
        memset(ring[head % RING] + fill, '\n', BLOCK - fill);
        ++head;
        fill = 0;
    }
    while ((tail != head || stats.state == SDLOG_WRITING || stats.state == SDLOG_WAIT_READY) &&
           stats.state != SDLOG_FAILED && stats.state != SDLOG_FULL && stats.state != SDLOG_OFF &&
           HAL_GetTick() - start < timeout_ms) {
        sdlog_service(1);
    }
}

const struct sdlog_stats *sdlog_stats(void)
{
    stats.queued = head - tail;
    return &stats;
}

#ifdef SDLOG_TEST_HOOKS
/* sdcheck/sdlog_test.c 만 쓴다 */
int sdlog_test_card_blocks(uint32_t *blocks)
{
    HAL_SD_CardInfoTypeDef info = { 0 };
    if (!stats.card_ok || HAL_SD_GetCardInfo(&hsd, &info) != HAL_OK) {
        return 0;
    }
    *blocks = info.LogBlockNbr;
    return 1;
}

int sdlog_test_write_block(uint32_t block_number, const uint8_t *data)
{
    HAL_NVIC_DisableIRQ(SDMMC2_IRQn);
    const int ok = HAL_SD_WriteBlocks(&hsd, data, block_number, 1, 5000) == HAL_OK && wait_ready(5000);
    HAL_NVIC_EnableIRQ(SDMMC2_IRQn);
    return ok ? 0 : 1;
}
#endif
