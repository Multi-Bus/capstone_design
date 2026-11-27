/* SD 기록 모듈(tag/sdlog.c)을 보드에서 시험한다. RAM 에서 돌리고 외부 플래시는 건드리지 않는다.
 *
 *   1) 카드를 켜고 UWBLOG.BIN 을 찾는다. 결과를 찍는다.
 *   2) 파일이 없으면 카드 끝의 빈 구역(0 으로 차 있는지 먼저 확인)을 시험 자리로 쓴다. 끝나면 0 으로 되돌린다.
 *      파일이 있으면 그 파일의 아직 안 쓴 자리에 쓴다(되돌리지 않는다. 파일 안의 기록으로 남는다).
 *   3) 태그 펌웨어처럼 20 ms 마다 F 줄 하나, 여섯 번에 한 번 R 줄 세 개를 30 s 동안 만든다.
 *      패킷을 기다리는 루프를 흉내 내어, 프레임의 12~18 ms 구간에서만 쓰기를 걸게 한다.
 *      그동안 루프가 가장 오래 멈춘 시간을 구간 안과 밖으로 나눠 잰다.
 *   4) 쓴 것을 다시 읽어 만든 줄과 하나씩 견준다.
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <port.h>
#include "sdfat.h"
#include "sdlog.h"

enum { FRAMES = 1500, FRAME_US = 20000, QUIET_FROM_US = 12000, QUIET_TO_US = 18000, REGION_BLOCKS = 1024 };

static uint8_t block[512] __attribute__((aligned(32)));

static void say(const char *format, ...)
{
    char line[200];
    va_list args;
    va_start(args, format);
    vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    test_run_info((unsigned char *)line);
}

static uint32_t us_since(uint32_t start)
{
    return (DWT->CYCCNT - start) / (SystemCoreClock / 1000000U);
}

/* 프레임 frame 의 line 번째 줄 (0 = F 줄, 1~3 = R 줄). 없으면 0 */
static unsigned make_line(char *out, unsigned size, uint32_t frame, unsigned line)
{
    if (line == 0) {
        return (unsigned)snprintf(out, size, "F,%lu,%08lX,%lu,7F,%c,480,210,0\n", (unsigned long)frame,
                                  (unsigned long)(0x10000000UL + frame * 4992189UL), (unsigned long)(frame % 6U),
                                  frame % 6U == 0 ? 'R' : '-');
    }
    if (frame % 6U != 0 || line > 3) {
        return 0;
    }
    return (unsigned)snprintf(out, size, "R,%lu,%u,%lu,1,F,-20500,-21500,%08lX,2800546\n", (unsigned long)frame, line,
                              (unsigned long)(2000U + 700U * line + frame % 7U),
                              (unsigned long)(0x10000000UL + frame * 4992189UL + 149760UL * (line - 1U)));
}

int ds_twr_responder(void)
{
    char line[96];
    const struct sdlog_stats *st;
    int restore = 0;

    say("SDLOG,START,fw=sdlogtest-v1");
    const int ready = sdlog_init();
    st = sdlog_stats();
    say("SDLOG,init,ready=%d,card_ok=%u,init_error=0x%08lX,file=%s,first_block=%lu,file_blocks=%lu,next_block=%lu", ready,
        (unsigned)st->card_ok, (unsigned long)st->init_error, sdfat_result_name((enum sdfat_result)st->file_result),
        (unsigned long)st->first_block, (unsigned long)st->file_blocks, (unsigned long)st->next_block);
    if (!st->card_ok) {
        say("SDLOG,DONE,result=no_card");
        for (;;) { }
    }
    uint32_t region = st->first_block, start_block = st->next_block;
    if (!ready) {
        /* 파일이 없다: 카드 끝의 빈 구역을 쓴다 */
        HAL_SD_CardInfoTypeDef info = { 0 };
        extern int sdlog_test_card_blocks(uint32_t *blocks);
        uint32_t total = 0;
        (void)info;
        if (!sdlog_test_card_blocks(&total) || total < 65536U) {
            say("SDLOG,DONE,result=card_size_unknown");
            for (;;) { }
        }
        region = total - 16384U;
        unsigned used = 0;
        for (uint32_t i = 0; i < REGION_BLOCKS; ++i) {
            if (sdlog_read_block(region + i, block) != 0) {
                say("SDLOG,DONE,result=read_error");
                for (;;) { }
            }
            for (unsigned k = 0; k < sizeof(block); ++k) {
                if (block[k] != 0x00 && block[k] != 0xFF) {
                    ++used;
                    break;
                }
            }
        }
        say("SDLOG,test_region,first_block=%lu,blocks=%u,blocks_with_data=%u", (unsigned long)region,
            (unsigned)REGION_BLOCKS, used);
        if (used != 0) {
            say("SDLOG,DONE,result=region_has_data_not_touched");
            for (;;) { }
        }
        sdlog_use_region(region, REGION_BLOCKS);
        start_block = 0;
        restore = 1;
    }

    /* 3) 30 s 동안 줄을 만들며 기다리는 루프를 돌린다 */
    uint32_t max_gap_quiet = 0, max_gap_busy = 0, lines = 0, bytes = 0;
    const uint32_t tick0 = HAL_GetTick();
    for (uint32_t frame = 1; frame <= FRAMES; ++frame) {
        for (unsigned k = 0; k < 4; ++k) {
            const unsigned n = make_line(line, sizeof(line), frame, k);
            if (n != 0) {
                sdlog_write(line, n);
                ++lines;
                bytes += n;
            }
        }
        const uint32_t start = DWT->CYCCNT;
        uint32_t last = start, elapsed = 0;
        while (elapsed < FRAME_US) {
            const int quiet = elapsed >= QUIET_FROM_US && elapsed < QUIET_TO_US;
            sdlog_service(quiet);
            const uint32_t gap = us_since(last);
            last = DWT->CYCCNT;
            if (quiet) {
                max_gap_quiet = gap > max_gap_quiet ? gap : max_gap_quiet;
            } else {
                max_gap_busy = gap > max_gap_busy ? gap : max_gap_busy;
            }
            elapsed = us_since(start);
        }
    }
    const uint32_t run_ms = HAL_GetTick() - tick0;
    sdlog_flush_blocking(5000);
    st = sdlog_stats();
    say("SDLOG,run,ms=%lu,lines=%lu,bytes=%lu,blocks_written=%lu,dropped_bytes=%lu,errors=%lu,state=%u,queued=%lu",
        (unsigned long)run_ms, (unsigned long)lines, (unsigned long)bytes, (unsigned long)st->written,
        (unsigned long)st->dropped_bytes, (unsigned long)st->errors, (unsigned)st->state, (unsigned long)st->queued);
    say("SDLOG,timing,max_loop_gap_outside_quiet_us=%lu,max_loop_gap_in_quiet_us=%lu,max_start_call_us=%lu,"
        "max_write_us=%lu", (unsigned long)max_gap_busy, (unsigned long)max_gap_quiet,
        (unsigned long)st->max_start_us, (unsigned long)st->max_write_us);

    /* 4) 다시 읽어 견준다 */
    uint32_t frame = 1, matched = 0, mismatched = 0, pos = 0;
    unsigned k = 0, want_len = make_line(line, sizeof(line), frame, k);
    const uint32_t written = st->written;
    for (uint32_t b = 0; b < written && mismatched == 0; ++b) {
        if (sdlog_read_block(region + start_block + b, block) != 0) {
            say("SDLOG,verify_read_error,block=%lu", (unsigned long)b);
            ++mismatched;
            break;
        }
        for (unsigned i = 0; i < sizeof(block) && mismatched == 0; ++i) {
            const char c = (char)block[i];
            if (pos == 0 && c == '\n') {
                continue; /* 채워 넣은 줄바꿈 */
            }
            if (want_len == 0 || c != line[pos]) {
                ++mismatched;
                break;
            }
            if (++pos == want_len) {
                ++matched;
                pos = 0;
                do {
                    if (++k > 3) {
                        k = 0;
                        ++frame;
                    }
                    want_len = frame <= FRAMES ? make_line(line, sizeof(line), frame, k) : 0;
                } while (want_len == 0 && frame <= FRAMES);
            }
        }
    }
    say("SDLOG,verify,lines_matched=%lu,of=%lu,mismatch=%lu", (unsigned long)matched, (unsigned long)lines,
        (unsigned long)mismatched);

    if (restore) {
        extern int sdlog_test_write_block(uint32_t block_number, const uint8_t *data);
        unsigned bad = 0;
        memset(block, 0, sizeof(block));
        for (uint32_t b = 0; b < written; ++b) {
            bad += sdlog_test_write_block(region + b, block) != 0;
        }
        say("SDLOG,restore,blocks_zeroed=%lu,errors=%u", (unsigned long)written, bad);
    }
    const int pass = st->errors == 0 && st->dropped_bytes == 0 && mismatched == 0 && matched == lines;
    say("SDLOG,DONE,result=%s", pass ? "PASS" : "FAIL");
    for (;;) {
        Sleep(1000);
    }
}
