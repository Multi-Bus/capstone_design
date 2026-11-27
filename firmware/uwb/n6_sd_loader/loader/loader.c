/* SD 로더 (DESIGN_KO.md). 핸드오프 FSBL 프로젝트의 응답기 예제 자리에 들어간다(prepare_loader.py).
 *
 * main.c 가 클록, GPIO, USART1 을 설정한 뒤 ds_twr_responder() 를 부른다. 여기서:
 *   1. 백업 레지스터와 리셋 원인으로 이번에 SD 를 시도할지 정한다 (5-1장)
 *   2. SD 카드의 N6APP.BIN 을 검사해 APP_LOAD_ADDR(AXISRAM1) 에 복사한다
 *   3. 안 되면 이 이미지 안에 넣어 둔 기본 펌웨어를 같은 자리에 복사한다
 *   4. 한 줄(LDR,...)을 남기고 그 펌웨어로 넘어간다. 돌아오지 않는다.
 */
#include <stdio.h>
#include <string.h>
#include "tagf_config.h"   /* TAGF_ID */
#include <port.h>
#include "app_header.h"
#include "loader_builtin.h"
#include "sdfat.h"
#include "sdlog.h"

#define LOADER_VERSION 1
#define APP_FILE "N6APP   BIN"
#define SD_MARK 0x4E365344UL  /* "N6SD": 바로 앞 부팅이 SD 펌웨어였음 */

extern UART_HandleTypeDef huart1;
extern const uint8_t loader_builtin_start[], loader_builtin_end[];

static uint8_t block[512] __attribute__((aligned(32)));

/* 펌웨어가 TF_INFO 에 남기는 부팅 이유 번호 (TAMP->BKP1R 의 아래 8비트). 순서를 바꾸지 말 것: DESIGN_KO.md 6장 */
static const char *const reasons[] = { "", "no_card", "no_fat", "no_file", "fragmented", "read_error", "bad_magic",
                                       "bad_version", "wrong_id", "bad_length", "bad_crc", "bad_vectors",
                                       "after_reset" };

static void note_boot(unsigned source, const char *reason)
{
    unsigned code = 0;
    for (unsigned i = 1; reason != NULL && i < sizeof(reasons) / sizeof(reasons[0]); ++i) {
        if (strcmp(reason, reasons[i]) == 0) {
            code = i;
        }
    }
    TAMP->BKP1R = ((uint32_t)'L' << 24) | ((uint32_t)source << 8) | code;   /* 출처 1 = SD, 2 = 기본 */
}

static const char *fat_reason(enum sdfat_result result)
{
    switch (result) {
    case SDFAT_NOT_FAT32: return "no_fat";
    case SDFAT_NO_FILE: return "no_file";
    case SDFAT_FRAGMENTED: return "fragmented";
    case SDFAT_TOO_SMALL: return "bad_length";
    default: return "read_error";
    }
}

/* SD 의 N6APP.BIN 을 APP_LOAD_ADDR 에 올린다. 성공하면 NULL, 실패하면 이유 */
static const char *load_from_sd(struct app_header *header)
{
    struct sdfat_file file;
    (void)sdlog_init();                       /* 카드를 껐다 켜고 초기화. UWBLOG.BIN 은 없어도 된다 */
    if (!sdlog_stats()->card_ok) {
        return "no_card";
    }
    const enum sdfat_result found = sdfat_locate(sdlog_read_block, APP_FILE, &file);
    if (found != SDFAT_OK) {
        return fat_reason(found);
    }
    if (sdlog_read_block(file.first_block, block) != 0) {
        return "read_error";
    }
    enum app_check check = app_check_header(block, TAGF_ID, file.blocks * 512U, header);
    if (check != APP_OK) {
        return app_check_name(check);
    }
    uint8_t *dest = (uint8_t *)APP_LOAD_ADDR;
    uint32_t done = 512U - APP_HEADER_SIZE;
    if (done > header->length) {
        done = header->length;
    }
    memcpy(dest, block + APP_HEADER_SIZE, done);
    for (uint32_t n = 1; done < header->length; ++n) {
        if (sdlog_read_block(file.first_block + n, block) != 0) {
            return "read_error";
        }
        const uint32_t take = header->length - done < 512U ? header->length - done : 512U;
        memcpy(dest + done, block, take);
        done += take;
    }
    if (app_crc32(0, dest, header->length) != header->crc) {
        return "bad_crc";
    }
    check = app_check_vectors(dest, header->length, APP_LOAD_ADDR);
    return check == APP_OK ? NULL : app_check_name(check);
}

static void sd_stop(void)
{
    HAL_NVIC_DisableIRQ(SDMMC2_IRQn);
    __HAL_RCC_SDMMC2_FORCE_RESET();
    __HAL_RCC_SDMMC2_RELEASE_RESET();
}

/* 펌웨어의 벡터 표로 넘어간다. 돌아오지 않는다 */
static void __attribute__((noreturn)) start_app(uint32_t base)
{
    const uint32_t sp = ((const uint32_t *)base)[0];
    const uint32_t entry = ((const uint32_t *)base)[1];
    while ((huart1.Instance->ISR & USART_ISR_TC) == 0U) {
    }
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->VAL = 0;
    for (unsigned i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); ++i) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }
    if (SCB->CCR & SCB_CCR_DC_Msk) {
        SCB_CleanDCache();
    }
    if (SCB->CCR & SCB_CCR_IC_Msk) {
        SCB_InvalidateICache();
    }
    SCB->VTOR = base;
    __set_MSPLIM(0);
    __DSB();
    __ISB();
    __enable_irq();                           /* 리셋 직후처럼: 펌웨어의 HAL_Init 뒤 SysTick 이 돌아야 한다 */
    __asm volatile("msr msp, %0\n\tbx %1" : : "r"(sp), "r"(entry) : "memory");
    __builtin_unreachable();
}

int ds_twr_responder(void)
{
    const uint32_t t0 = HAL_GetTick();
    const uint32_t reset_flags = RCC->RSR;
    RCC->RSR = RCC_RSR_RMVF;                  /* 다음 부팅이 이번 리셋 원인을 보지 않게 지운다 */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
    const uint32_t mark = TAMP->BKP0R;
    TAMP->BKP0R = 0;
    const int power_on = (reset_flags & (RCC_RSR_PORRSTF | RCC_RSR_BORRSTF)) != 0U;
    __HAL_RCC_AXISRAM1_MEM_CLK_ENABLE();

    HAL_Delay(20);                            /* main.c 의 115200 출력이 다 나가게 */
    huart1.Init.BaudRate = 921600;
    (void)HAL_UART_Init(&huart1);

    struct app_header header;
    const char *reason = (!power_on && mark == SD_MARK) ? "after_reset" : load_from_sd(&header);
    sd_stop();
    if (reason == NULL) {
        printf("\r\nLDR,ver=%d,id=%d,src=sd,fw=%s,len=%lu,crc=0x%08lx,ms=%lu\r\n", LOADER_VERSION, TAGF_ID,
               header.name, (unsigned long)header.length, (unsigned long)header.crc,
               (unsigned long)(HAL_GetTick() - t0));
        TAMP->BKP0R = SD_MARK;
        note_boot(1, NULL);
        start_app(APP_LOAD_ADDR);
    }
    const uint32_t length = (uint32_t)(loader_builtin_end - loader_builtin_start);
    memcpy((void *)APP_LOAD_ADDR, loader_builtin_start, length);
    const enum app_check check = app_check_vectors((const uint8_t *)APP_LOAD_ADDR, length, APP_LOAD_ADDR);
    printf("\r\nLDR,ver=%d,id=%d,src=builtin,reason=%s,fw=%s,len=%lu,ms=%lu,builtin=%s\r\n", LOADER_VERSION,
           TAGF_ID, reason, LOADER_BUILTIN_NAME, (unsigned long)length, (unsigned long)(HAL_GetTick() - t0),
           app_check_name(check));
    if (check != APP_OK) {
        for (;;) {                            /* 기본 펌웨어가 망가진 빌드: 넘어가지 않는다 */
        }
    }
    note_boot(2, reason);
    start_app(APP_LOAD_ADDR);
}
