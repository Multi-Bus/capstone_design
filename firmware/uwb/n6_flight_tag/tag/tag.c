/* 3대1 DS-TWR 태그(응답기) v2. NUCLEO-N657 + DWM3000EVB.
 *
 * drone-hw-bench `task6-qorvo-3to1-20261123`(b934e69)의 qorvo_3to1/tag.c 를 고친 것이다.
 * 원본은 같은 폴더의 tag_v1_original.c.
 *
 * v1 과 같은 것
 *   - 무선 설정, 패킷 형식(frames.h, radio.h), 응답 지연 900 µs, Final 대기 500/220 µs
 *   - TWR3 줄 (14칸, 학생 capture.py 가 그대로 읽음)
 *
 * v2 에서 더한 것
 *   1) TWR3T 줄: 같은 교환의 시각을 태그 DW3000 시계 하나로 적는다. 9패킷 모두 이 시계로 잰다.
 *        TWR3T,<n>,<앵커>,<회차>,<Poll 시각>,<준비 µs>,<끝 ns>,<끝 종류>,<수신오류 누계>,<기타프레임 누계>,<늦은Poll 누계>
 *        - n: 태그가 받은 Poll 일련번호(1부터). 빠지면 USB 로그 줄이 빠진 것.
 *        - Poll 시각: Poll RMARKER 의 DW 시각 상위 32비트(hex, 1 = 4.006 ns, 17.2 s 마다 한 바퀴)
 *        - 준비 µs: Poll RMARKER 부터 펌웨어가 Poll 을 알아챌 때까지. 응답 지연(900 µs)의 여유를 본다.
 *        - 끝 ns: Poll RMARKER 부터 교환이 끝난 때까지
 *            F = Final RMARKER (무선 타임스탬프), T/E/X/N = 시간 초과·오류를 알아챈 때(DW 시스템 시각)
 *        - 끝 종류: F Final 받음, T Final 시간 초과, E 수신 오류, X 다른 프레임, N 10 ms 동안 사건 없음,
 *                   L Response 송신 실패(늦음), B 출력 중이라 늦게 알아챈 Poll(응답 안 함)
 *   2) 출력을 모았다가 무선이 FLUSH_IDLE_MS 동안 조용할 때 내보낸다.
 *      UART 가 115200 블로킹(한 줄 약 8 ms)이라, 교환 직후 바로 찍으면 다음 앵커의 Poll 을 놓칠 수 있다.
 *   3) 출력하는 동안 들어온 Poll 은 응답 시각이 이미 지났으므로 응답하지 않고 B 로 남긴다.
 *      측정 장치 탓 손실(B)을 무선 손실과 나눠 셀 수 있다.
 */
#include <stdio.h>
#include <config_options.h>
#include <deca_spi.h>
#include <port.h>
#include "deca_probe_interface.h"
#include "radio.h"
#include "tag_antenna_delay.h"

#define TAG_FW "tag3to1-v2"

extern dwt_txconfig_t txconfig_options;
extern dwt_config_t config_options;

enum {
    FLUSH_IDLE_MS = 3, /* 마지막 사건 뒤 2~3 ms(HAL 틱 1 ms) 조용하면 모아 둔 줄을 출력 */
    PENDING_MAX = 8,   /* 모아 둘 교환 수. 차면 바로 출력 */
    STALE_US = 650,    /* Poll 을 이보다 늦게 알아채면 응답하지 않음(응답 지연 900 µs) */
};
#define DTU_40BIT_MASK 0xFFFFFFFFFFULL
#define STALE_HI32 ((uint32_t)(((uint64_t)STALE_US * UWB3_DTU_PER_UUS) >> 8))

struct record {
    struct uwb3_key key;
    uint32_t n, poll_hi, prep_us, end_ns;
    uint32_t poll_status, final_status;
    uint32_t rx_errors, rx_other, stale;
    int32_t mm;
    struct uwb3_power poll_power, final_power;
    uint8_t response_sent, final_good, distance_valid;
    char end;
};

static struct record pending[PENDING_MAX];
static unsigned pending_count;
static uint32_t polls_seen, rx_errors, rx_other, stale_polls;

/* DW 시각 단위(15.65 ps, 1 µs = 63897.6) -> ns */
static uint32_t dtu_to_ns(uint64_t dtu)
{
    return (uint32_t)((dtu * 10000ULL) / 638976ULL);
}

static uint32_t hi32_to_ns(uint32_t hi)
{
    return dtu_to_ns((uint64_t)hi << 8);
}

static uint32_t wait_events(uint32_t max_ms)
{
    const uint32_t start = portGetTickCnt();
    uint32_t status;
    do {
        status = dwt_readsysstatuslo();
    } while ((status & UWB3_RX_EVENTS) == 0 &&
             (max_ms == 0 || portGetTickCnt() - start < max_ms));
    return status;
}

static void print_record(const struct record *r)
{
    char line[180];
    snprintf(line, sizeof(line), "TWR3,%lu,%lu,%u,%u,%u,%u,%ld,%d,%d,%d,%d,%08lX,%08lX",
             (unsigned long)r->key.session, (unsigned long)r->key.round, (unsigned)r->key.anchor,
             (unsigned)r->response_sent, (unsigned)r->final_good, (unsigned)r->distance_valid, (long)r->mm,
             r->poll_power.rssi, r->poll_power.first_path, r->final_power.rssi, r->final_power.first_path,
             (unsigned long)r->poll_status, (unsigned long)r->final_status);
    test_run_info((unsigned char *)line);
    snprintf(line, sizeof(line), "TWR3T,%lu,%u,%lu,%08lX,%lu,%lu,%c,%lu,%lu,%lu",
             (unsigned long)r->n, (unsigned)r->key.anchor, (unsigned long)r->key.round,
             (unsigned long)r->poll_hi, (unsigned long)r->prep_us, (unsigned long)r->end_ns, r->end,
             (unsigned long)r->rx_errors, (unsigned long)r->rx_other, (unsigned long)r->stale);
    test_run_info((unsigned char *)line);
}

/* 수신은 켜 둔 채로 출력한다. 출력 중 들어온 프레임은 칩에 남아 있다가 다음 차례에 처리된다. */
static void flush_pending(void)
{
    for (unsigned i = 0; i < pending_count; ++i) {
        print_record(&pending[i]);
    }
    pending_count = 0;
}

static void finish_record(struct record *r)
{
    r->rx_errors = rx_errors;
    r->rx_other = rx_other;
    r->stale = stale_polls;
}

static void respond(struct uwb3_key key, uint32_t poll_status)
{
    uint8_t frame[UWB3_FINAL_LEN + UWB3_FCS];
    struct uwb3_key final;
    struct record *r = &pending[pending_count++];
    const struct uwb3_power unknown = { UWB3_UNKNOWN_POWER, UWB3_UNKNOWN_POWER };

    *r = (struct record){ 0 };
    r->key = key;
    r->n = ++polls_seen;
    r->poll_status = poll_status;
    r->poll_power = unknown;
    r->final_power = unknown;
    r->end = 'N';

    const uint64_t poll_rx = uwb3_timestamp(false);
    r->poll_hi = (uint32_t)(poll_rx >> 8);
    const uint32_t age_hi = dwt_readsystimestamphi32() - r->poll_hi;
    r->prep_us = hi32_to_ns(age_hi) / 1000U;
    if (age_hi > STALE_HI32) {
        /* 출력하느라 늦게 알아챘다. 응답 시각(Poll + 900 µs)이 이미 지났거나 너무 가깝다. */
        ++stale_polls;
        r->end = 'B';
        r->end_ns = hi32_to_ns(age_hi);
        finish_record(r);
        return;
    }

    dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
    dwt_setdelayedtrxtime((uint32_t)((poll_rx + (uint64_t)UWB3_RESPONSE_DELAY * UWB3_DTU_PER_UUS) >> 8));
    dwt_setrxaftertxdelay(UWB3_FINAL_RX_DELAY);
    dwt_setrxtimeout(UWB3_FINAL_TIMEOUT);
    uwb3_encode(frame, UWB3_RESPONSE, key);
    const int sent = uwb3_send(frame, UWB3_HEADER, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    r->poll_power = uwb3_read_power();
    if (sent != DWT_SUCCESS) {
        r->end = 'L';
    } else {
        r->final_status = wait_events(10);
        const uint32_t seen_hi = dwt_readsystimestamphi32();
        r->end_ns = hi32_to_ns(seen_hi - r->poll_hi);
        r->response_sent = (r->final_status & DWT_INT_TXFRS_BIT_MASK) != 0;
        if (r->final_status & DWT_INT_RXFCG_BIT_MASK) {
            r->end = 'X';
            if (uwb3_decode(frame, uwb3_read(frame), UWB3_FINAL, &final) && uwb3_same_key(key, final)) {
                r->final_good = 1;
                r->end = 'F';
                const uint32_t response_tx = (uint32_t)uwb3_timestamp(true);
                const uint64_t final_rx = uwb3_timestamp(false);
                r->end_ns = dtu_to_ns((final_rx - poll_rx) & DTU_40BIT_MASK);
                r->distance_valid = uwb3_distance_mm(frame, (uint32_t)poll_rx, response_tx, (uint32_t)final_rx, &r->mm);
                r->final_power = uwb3_read_power();
            }
        } else if (r->final_status & (DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK)) {
            r->end = 'T';
        } else if (r->final_status & SYS_STATUS_ALL_RX_ERR) {
            r->end = 'E';
        }
    }
    dwt_forcetrxoff();
    dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
    finish_record(r);
}

/* This entry is called by the handoff project's TEST_DS_TWR_RESPONDER selection. */
int ds_twr_responder(void)
{
    reset_DWIC();
    Sleep(2);
    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS ||
        !dwt_checkidlerc() || dwt_initialise(DWT_READ_OTP_ALL) != DWT_SUCCESS ||
        dwt_configure(&config_options) != DWT_SUCCESS) {
        test_run_info((unsigned char *)"TWR3_INIT_FAILED");
        for (;;) { }
    }
    port_set_dw_ic_spi_fastrate();
    dwt_configuretxrf(&txconfig_options);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_settxantennadelay(UWB3_ANTENNA_DELAY);
    dwt_setrxantennadelay(UWB3_ANTENNA_DELAY);
    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);
    dwt_setpreambledetecttimeout(0);
    test_run_info((unsigned char *)"TWR3_READY,channel=5,tag=256");
    {
        char line[120];
        snprintf(line, sizeof(line), "TWR3_INFO,fw=" TAG_FW ",ant_dly=%u,flush_idle_ms=%u,stale_us=%u",
                 (unsigned)UWB3_ANTENNA_DELAY, (unsigned)FLUSH_IDLE_MS, (unsigned)STALE_US);
        test_run_info((unsigned char *)line);
    }

    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    for (;;) {
        const uint32_t status = wait_events(pending_count ? FLUSH_IDLE_MS : 0);
        if ((status & UWB3_RX_EVENTS) == 0) {
            flush_pending(); /* 조용함: 수신은 켜진 그대로 */
            continue;
        }
        struct uwb3_key key;
        uint8_t frame[UWB3_FINAL_LEN + UWB3_FCS];
        if (status & DWT_INT_RXFCG_BIT_MASK) {
            if (uwb3_decode(frame, uwb3_read(frame), UWB3_POLL, &key)) {
                respond(key, status);
            } else {
                ++rx_other;
            }
        } else if (status & SYS_STATUS_ALL_RX_ERR) {
            ++rx_errors;
        }
        dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
        if (pending_count == PENDING_MAX) {
            flush_pending();
        }
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}
