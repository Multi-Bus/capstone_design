/* 3대1 DS-TWR 태그(응답기), 7패킷(묶음) 방식. NUCLEO-N657 + DWM3000EVB.
 *
 * 9패킷 방식은 같은 폴더의 tag.c (v2) 에 그대로 있다. prepare_tag.py --packets 9 / 7 로 고른다.
 * 앵커 쪽 짝은 src/main7.c, 패킷 형식과 시각표는 frames.h 의 "7패킷" 부분.
 *
 *   Poll 1, 2, 3 (앵커, 간격 S) -> Response (태그, 방송 한 번) -> Final 1, 2, 3 (앵커, 간격 S)
 *
 * 규칙
 *   - Response 시각 = 처음 받은 Poll k 의 수신 시각 + (3 - k) x S + D1. 어느 Poll 을 기준으로 해도 같은 시각이다.
 *     Poll 일부를 못 받아도 같은 시각에 답하고, 받은 Poll 의 회차 번호만 Response 에 싣는다(못 받은 앵커는 0).
 *   - Final k 를 받으면 9패킷과 같은 uwb3_distance_mm 으로 거리를 계산한다. Response 송신 시각은 세 앵커에 공통이다.
 *   - 출력은 묶음이 끝난 뒤에 한다. 출력 중(약 40 ms)에 들어온 Poll 은 늦게 알아채므로 답하지 않고 B 로 남긴다.
 *   - S, D1, D2 는 태그가 정해서 Response 에 싣고 앵커가 따른다. TAG7_SWEEP 빌드는 표의 값을 차례로 바꿔 가며 잰다.
 *
 * 출력 (TWR3 줄은 9패킷과 같은 14칸, TWR3T 는 끝에 묶음 번호 한 칸이 더 붙는다)
 *   TWR3,<세션>,<회차>,<앵커>,<Response 보냄>,<Final 받음>,<거리 유효>,<mm>,<Poll RSSI>,<Poll 첫경로>,
 *        <Final RSSI>,<Final 첫경로>,<Poll 상태>,<Final 상태>
 *   TWR3T,<n>,<앵커>,<회차>,<Poll 시각>,<준비 µs>,<끝 ns>,<끝 종류>,<수신오류 누계>,<기타프레임 누계>,
 *         <늦은Poll 누계>,<묶음 번호>
 *        - 준비 µs: Poll RMARKER 부터 그 Poll 을 다 읽고 다음 일(수신 다시 켜기, 마지막 Poll 이면 Response 예약)을
 *          마친 때까지
 *        - 끝 ns: Poll RMARKER 부터 F = Final RMARKER, T/E = 묶음을 끝낸 때, L/B = 포기한 때
 *        - 끝 종류: F Final 받음, T Final 안 옴, E Final 자리에서 수신 오류, L Response 예약이 늦음(D1 이 짧음),
 *                   B 출력하느라 Poll 을 늦게 알아채서 Response 를 못 보냄
 *   TWR3B,<묶음 번호>,<기준 앵커>,<Response 시각>,<S>,<D1>,<D2>,<받은 Poll>,<Response>,<받은 Final>,<거리 유효>,
 *         <Response 여유 µs>,<되돌림 µs>,<끝 ns>,<묶음 안 수신오류>,<표 줄 번호>
 *        - Response 시각: 예약한 RMARKER 의 DW 시각 상위 32비트(hex). Poll 시각과 같은 시계다.
 *        - 받은 Poll / 받은 Final / 거리 유효: 앵커 1, 2, 3 순서로 1 또는 0 (예: 101)
 *        - Response: R 보냄, L 예약이 늦어 못 보냄, N 송신 끝 표시를 못 봄, B 출력하느라 늦게 알아채서 못 보냄
 *        - Response 여유 µs: 예약 명령을 내린 때부터 송신 시작까지 남은 시간. D1 을 줄일 여유다.
 *        - 되돌림 µs: 패킷(Poll, Final) RMARKER 부터 다음 수신을 켠 때까지 중 가장 긴 것.
 *          S - 139 보다 작아야 다음 패킷을 받는다.
 *        - 끝 ns: 기준 Poll RMARKER 부터 마지막 Final RMARKER(다 받았을 때) 또는 묶음을 끝낸 때까지
 */
#include <stdio.h>
#include <string.h>
#include "tag7_config.h" /* prepare_tag.py 가 쓴다. frames.h 의 기본값보다 먼저 와야 한다 */
#include <config_options.h>
#include <deca_spi.h>
#include <port.h>
#include "deca_probe_interface.h"
#include "radio.h"
#include "tag_antenna_delay.h"

#define TAG_FW "tag7p-v1"
#ifndef TAG7_SWEEP
#define TAG7_SWEEP 0
#endif
#ifndef TAG7_SWEEP_BUNDLES
#define TAG7_SWEEP_BUNDLES 300 /* 표의 한 줄마다 묶음 수 (한 바퀴 60 ms 면 18 s) */
#endif
#ifndef TAG7_BUILD_OPT
#define TAG7_BUILD_OPT "?"
#endif

extern dwt_txconfig_t txconfig_options;
extern dwt_config_t config_options;

enum {
    /* Response 송신 시작 이만큼 전에 Poll 듣기를 멈추고 예약한다: D1 - 260 을 아래 범위로 자른 값.
     * 길수록 예약이 늦을 걱정이 없고, 짧을수록 늦게 오는 Poll(앵커 3)을 기다릴 수 있다.
     * 예약에 드는 시간은 SPI 6번 + 칩 준비로 -O2 에서 약 110 µs (어림). */
    TX_LEAD_MAX_US = 300,
    TX_LEAD_MIN_US = 130,
    TX_LEAD_BACK_US = 260,
    END_MARGIN_US = 350,  /* 마지막 Final 의 RMARKER 예정 시각 뒤 이만큼 더 기다린다 */
    NOTICE_LATE_US = 500, /* 첫 Poll 을 이보다 늦게 알아챘는데 Response 를 못 보내면 출력 탓(B)으로 적는다 */
    POWER_READ_US = 220,  /* 신호 세기 읽기에 드는 시간(어림). 다음 패킷 전에 이만큼 남아야 읽는다 */
    RX_EARLY_US = 260,    /* Response 송신이 끝난 뒤 수신을 켜는 때: Final 프리앰블보다 약 60 µs 먼저 */
};
#define DTU_40BIT_MASK 0xFFFFFFFFFFULL

#if TAG7_SWEEP
/* S, D1, D2 (µs). 한 번에 하나씩 줄여 보고, 끝에 함께 줄인다. 0 번 줄이 기준이다. */
static const struct uwb7_timing sweep_table[] = {
    { 600, 900, 700 },
    { 600, 700, 700 }, { 600, 600, 700 }, { 600, 500, 700 }, { 600, 450, 700 }, { 600, 400, 700 }, { 600, 350, 700 },
    { 600, 900, 600 }, { 600, 900, 500 }, { 600, 900, 450 }, { 600, 900, 400 }, { 600, 900, 350 }, { 600, 900, 300 },
    { 500, 900, 700 }, { 450, 900, 700 }, { 400, 900, 700 }, { 350, 900, 700 }, { 300, 900, 700 },
    { 600, 900, 700 },
    { 500, 600, 600 }, { 450, 500, 500 }, { 400, 500, 500 }, { 400, 450, 450 }, { 400, 425, 450 },
    { 350, 450, 450 }, { 350, 400, 400 },
};
enum { SWEEP_ROWS = sizeof(sweep_table) / sizeof(sweep_table[0]) };
#endif

struct exchange {
    struct uwb3_key key;
    uint64_t poll_rx, final_rx;
    uint32_t n, prep_us, end_ns, poll_status, final_status;
    int32_t mm;
    struct uwb3_power poll_power, final_power;
    uint8_t final_frame[UWB3_FINAL_LEN + UWB3_FCS];
    uint8_t have_poll, have_final, distance_valid;
    char end;
};

struct bundle {
    struct exchange x[UWB3_ANCHORS];
    struct uwb7_timing t;
    uint32_t no, resp_hi, end_ns, rx_err, turn_us;
    int32_t lead_us;
    uint8_t ref, row;
    char resp;
};

static struct bundle bundle;
static struct uwb7_timing timing = { UWB7_SLOT_US, UWB7_RESPONSE_DELAY_US, UWB7_FINAL_DELAY_US };
static uint32_t bundle_no, polls_seen, rx_errors, rx_other, stale_polls, legacy_frames;
static uint8_t sweep_row;

/* DW 시각 단위(15.65 ps, 1 µs = 63897.6) -> ns */
static uint32_t dtu_to_ns(uint64_t dtu)
{
    return (uint32_t)((dtu * 10000ULL) / 638976ULL);
}

static uint32_t hi32_to_ns(uint32_t hi)
{
    return dtu_to_ns((uint64_t)hi << 8);
}

static uint32_t hi32_of(uint64_t timestamp)
{
    return (uint32_t)(timestamp >> 8);
}

static void rx_on(void)
{
    dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/* 다음 패킷의 프리앰블이 시작될 때(next_shr_hi)까지 시간이 넉넉하면 방금 받은 패킷의 신호 세기를 읽는다.
 * 수신을 다시 켠 뒤에 읽으므로, 읽느라 다음 패킷을 놓치지는 않는다. 못 읽으면 32767 로 남는다. */
static bool read_power_before(uint32_t next_shr_hi, uint32_t now_hi, struct uwb3_power *power)
{
    if ((int32_t)(next_shr_hi - now_hi) < (int32_t)uwb7_us_to_hi(POWER_READ_US)) {
        return false;
    }
    *power = uwb3_read_power();
    return true;
}

static uint32_t tx_lead_us(struct uwb7_timing t)
{
    const uint32_t lead = t.response_delay > TX_LEAD_BACK_US ? (uint32_t)t.response_delay - TX_LEAD_BACK_US : 0U;
    return lead > TX_LEAD_MAX_US ? TX_LEAD_MAX_US : lead < TX_LEAD_MIN_US ? TX_LEAD_MIN_US : lead;
}

/* 패킷 RMARKER 부터 지금(now_hi)까지 µs */
static uint32_t since_us(uint64_t rmarker, uint32_t now_hi)
{
    return hi32_to_ns(now_hi - hi32_of(rmarker)) / 1000U;
}

static struct exchange *record_poll(struct bundle *b, struct uwb3_key key, uint32_t status)
{
    struct exchange *x = &b->x[key.anchor - 1];
    if (x->have_poll) {
        return NULL;
    }
    x->key = key;
    x->poll_rx = uwb3_timestamp(false);
    x->n = ++polls_seen;
    x->poll_status = status;
    x->have_poll = 1;
    x->end = 'T';
    return x;
}

/* 수신 사건이 생기면 그 상태를, stop_hi 가 되면 0 을 돌려준다. */
static uint32_t wait_rx_until(uint32_t stop_hi)
{
    for (;;) {
        const uint32_t status = dwt_readsysstatuslo();
        if (status & UWB3_RX_EVENTS) {
            return status;
        }
        if ((int32_t)(dwt_readsystimestamphi32() - stop_hi) >= 0) {
            return 0;
        }
    }
}

static void give_up(struct bundle *b, char end)
{
    const uint32_t now_hi = dwt_readsystimestamphi32();
    for (unsigned i = 0; i < UWB3_ANCHORS; ++i) {
        struct exchange *x = &b->x[i];
        if (x->have_poll) {
            x->end = end;
            x->end_ns = hi32_to_ns(now_hi - hi32_of(x->poll_rx));
        }
    }
    b->resp = end;
    b->end_ns = hi32_to_ns(now_hi - hi32_of(b->x[b->ref - 1].poll_rx));
}

/* 첫 Poll 을 받은 뒤 묶음 하나를 끝까지 처리한다. 수신은 꺼진 채로 돌아온다. */
static void run_bundle(struct bundle *b, struct uwb3_key key, uint32_t status)
{
    uint8_t frame[UWB3_FINAL_LEN + UWB3_FCS];
    const struct uwb7_timing t = b->t;
    const uint32_t shr_hi = uwb7_us_to_hi(UWB7_SHR_US);
    const uint32_t slot_hi = uwb7_us_to_hi(t.slot);

    struct exchange *x = record_poll(b, key, status);
    struct exchange *last = x;
    bool power_done = false;
    b->ref = (uint8_t)key.anchor;
    b->resp_hi = hi32_of(x->poll_rx + (uint64_t)((UWB3_ANCHORS - key.anchor) * t.slot + t.response_delay) *
                                          UWB3_DTU_PER_UUS) & ~1U;
    const uint32_t stop_hi = b->resp_hi - shr_hi - uwb7_us_to_hi(tx_lead_us(t));
    uint32_t now_hi = dwt_readsystimestamphi32();
    x->prep_us = since_us(x->poll_rx, now_hi);
    /* 출력하는 동안 들어온 Poll 은 늦게 알아챈다. 그 탓에 Response 를 못 보내면 L 이 아니라 B 로 적는다. */
    const char late_kind = x->prep_us > NOTICE_LATE_US ? 'B' : 'L';
    if ((int32_t)(now_hi - (b->resp_hi - shr_hi)) >= 0) {
        stale_polls += late_kind == 'B';
        give_up(b, late_kind); /* Response 를 보낼 때가 이미 지났다 */
        return;
    }

    /* 1) 남은 Poll 을 듣는다. 앵커 3 의 Poll 이 마지막이므로 받으면 바로 다음 단계로 간다. */
    bool more = key.anchor < UWB3_ANCHORS;
    bool receiving = false;
    while (more) {
        rx_on();
        receiving = true;
        now_hi = dwt_readsystimestamphi32();
        last->prep_us = since_us(last->poll_rx, now_hi);
        if (last->prep_us > b->turn_us) {
            b->turn_us = last->prep_us;
        }
        if (!power_done) {
            power_done = read_power_before(hi32_of(last->poll_rx) + slot_hi - shr_hi, now_hi, &last->poll_power);
        }
        status = wait_rx_until(stop_hi);
        if (status == 0) {
            break;
        }
        receiving = false; /* 수신 사건이 나면 칩이 수신을 끈다 */
        if (status & DWT_INT_RXFCG_BIT_MASK) {
            if (uwb7_decode(frame, uwb3_read(frame), UWB7_POLL, &key) &&
                (x = record_poll(b, key, status)) != NULL) {
                last = x;
                power_done = false;
                more = key.anchor < UWB3_ANCHORS;
            } else {
                ++rx_other;
            }
        } else {
            ++rx_errors;
            ++b->rx_err;
        }
    }

    /* 2) Response 를 예약한다. 받은 Poll 의 회차 번호와 S, D1, D2 를 싣는다.
     *    마지막 Poll 을 받은 때부터 여기 명령까지가 D1 의 하한을 정하므로, 꼭 필요한 일만 먼저 한다. */
    struct uwb7_response response = { t, (uint16_t)b->no, { 0 } };
    unsigned last_anchor = 0, expected = 0;
    for (unsigned i = 0; i < UWB3_ANCHORS; ++i) {
        if (b->x[i].have_poll) {
            response.round[i] = b->x[i].key.round;
            last_anchor = i + 1;
            ++expected;
        }
    }
    if (receiving) {
        dwt_forcetrxoff();
    }
    uwb7_encode_response(frame, &response);
    dwt_setdelayedtrxtime(b->resp_hi);
    if (uwb3_send(frame, UWB7_RESPONSE_LEN, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        stale_polls += late_kind == 'B';
        give_up(b, late_kind);
        return;
    }
    now_hi = dwt_readsystimestamphi32();
    dwt_writesysstatuslo(UWB3_RX_EVENTS); /* Poll 의 수신 표시를 지운다. 송신 끝 표시는 건드리지 않는다 */
    const int32_t lead_hi = (int32_t)(b->resp_hi - shr_hi - now_hi);
    b->lead_us = lead_hi >= 0 ? (int32_t)(hi32_to_ns((uint32_t)lead_hi) / 1000U)
                              : -(int32_t)(hi32_to_ns((uint32_t)-lead_hi) / 1000U);
    if (!receiving) {
        last->prep_us = since_us(last->poll_rx, now_hi); /* 마지막 Poll: Response 예약을 마친 때 */
    }
    b->resp = 'N';
    const uint32_t final_shr_hi = b->resp_hi + uwb7_us_to_hi(t.final_delay) - shr_hi; /* Final 1 의 프리앰블 시작 */
    if (!power_done) {
        (void)read_power_before(final_shr_hi, now_hi, &last->poll_power);
    }

    /* 3) Final 을 듣는다. 송신이 끝나면 칩이 스스로 수신을 켠다. */
    const uint32_t end_hi = b->resp_hi +
                            uwb7_us_to_hi(t.final_delay + (last_anchor - 1U) * t.slot + END_MARGIN_US);
    last = NULL;
    power_done = false;
    for (;;) {
        status = dwt_readsysstatuslo();
        if (status & DWT_INT_TXFRS_BIT_MASK) {
            b->resp = 'R';
        }
        if (status & UWB3_RX_EVENTS) {
            if (status & DWT_INT_RXFCG_BIT_MASK) {
                x = NULL;
                if (uwb7_decode(frame, uwb3_read(frame), UWB7_FINAL, &key)) {
                    x = &b->x[key.anchor - 1];
                }
                if (x != NULL && x->have_poll && !x->have_final && uwb3_same_key(key, x->key)) {
                    x->final_rx = uwb3_timestamp(false);
                    memcpy(x->final_frame, frame, sizeof(x->final_frame));
                    x->final_status = status;
                    x->have_final = 1;
                    last = x;
                    power_done = false;
                    --expected;
                } else {
                    ++rx_other;
                }
            } else {
                /* 어느 앵커의 Final 자리였는지는 시각으로 가린다 */
                const uint32_t since_hi = dwt_readsystimestamphi32() - final_shr_hi;
                const uint32_t slot = (int32_t)since_hi > 0 ? since_hi / slot_hi : 0U;
                if (slot < UWB3_ANCHORS && b->x[slot].have_poll && !b->x[slot].have_final) {
                    b->x[slot].end = 'E';
                }
                ++rx_errors;
                ++b->rx_err;
            }
            if (expected == 0) {
                break;
            }
            rx_on();
            now_hi = dwt_readsystimestamphi32();
            if ((int32_t)(now_hi - end_hi) >= 0) {
                break;
            }
            if (last != NULL) {
                if (since_us(last->final_rx, now_hi) > b->turn_us) {
                    b->turn_us = since_us(last->final_rx, now_hi);
                }
                if (!power_done) {
                    power_done = read_power_before(hi32_of(last->final_rx) + slot_hi - shr_hi, now_hi,
                                                   &last->final_power);
                }
            }
        } else if ((int32_t)(dwt_readsystimestamphi32() - end_hi) >= 0) {
            break;
        }
    }
    const uint32_t end_seen_hi = dwt_readsystimestamphi32();
    dwt_forcetrxoff();
    dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
    if (last != NULL && !power_done) {
        last->final_power = uwb3_read_power();
    }

    /* 4) 거리. Response 송신 시각은 세 앵커에 공통이다. */
    const uint32_t response_tx = (uint32_t)uwb3_timestamp(true);
    const uint64_t ref_rx = b->x[b->ref - 1].poll_rx;
    uint64_t last_final = 0;
    for (unsigned i = 0; i < UWB3_ANCHORS; ++i) {
        x = &b->x[i];
        if (!x->have_poll) {
            continue;
        }
        if (x->have_final) {
            x->end = 'F';
            x->end_ns = dtu_to_ns((x->final_rx - x->poll_rx) & DTU_40BIT_MASK);
            x->distance_valid = uwb3_distance_mm(x->final_frame, (uint32_t)x->poll_rx, response_tx,
                                                 (uint32_t)x->final_rx, &x->mm);
            last_final = (x->final_rx - ref_rx) & DTU_40BIT_MASK;
        } else {
            x->end_ns = hi32_to_ns(end_seen_hi - hi32_of(x->poll_rx));
        }
    }
    b->end_ns = expected == 0 ? dtu_to_ns(last_final) : hi32_to_ns(end_seen_hi - hi32_of(ref_rx));
}

static void print_bundle(const struct bundle *b)
{
    char line[180];
    char polls[UWB3_ANCHORS + 1] = "000", finals[UWB3_ANCHORS + 1] = "000", valid[UWB3_ANCHORS + 1] = "000";
    for (unsigned i = 0; i < UWB3_ANCHORS; ++i) {
        const struct exchange *x = &b->x[i];
        if (!x->have_poll) {
            continue;
        }
        polls[i] = '1';
        finals[i] = x->have_final ? '1' : '0';
        valid[i] = x->distance_valid ? '1' : '0';
        snprintf(line, sizeof(line), "TWR3,%lu,%lu,%u,%u,%u,%u,%ld,%d,%d,%d,%d,%08lX,%08lX",
                 (unsigned long)x->key.session, (unsigned long)x->key.round, (unsigned)x->key.anchor,
                 (unsigned)(b->resp == 'R'), (unsigned)x->have_final, (unsigned)x->distance_valid, (long)x->mm,
                 x->poll_power.rssi, x->poll_power.first_path, x->final_power.rssi, x->final_power.first_path,
                 (unsigned long)x->poll_status, (unsigned long)x->final_status);
        test_run_info((unsigned char *)line);
        snprintf(line, sizeof(line), "TWR3T,%lu,%u,%lu,%08lX,%lu,%lu,%c,%lu,%lu,%lu,%lu",
                 (unsigned long)x->n, (unsigned)x->key.anchor, (unsigned long)x->key.round,
                 (unsigned long)hi32_of(x->poll_rx), (unsigned long)x->prep_us, (unsigned long)x->end_ns, x->end,
                 (unsigned long)rx_errors, (unsigned long)rx_other, (unsigned long)stale_polls,
                 (unsigned long)b->no);
        test_run_info((unsigned char *)line);
    }
    snprintf(line, sizeof(line), "TWR3B,%lu,%u,%08lX,%u,%u,%u,%s,%c,%s,%s,%ld,%lu,%lu,%lu,%u",
             (unsigned long)b->no, (unsigned)b->ref, (unsigned long)b->resp_hi, (unsigned)b->t.slot,
             (unsigned)b->t.response_delay, (unsigned)b->t.final_delay, polls, b->resp, finals, valid,
             (long)b->lead_us, (unsigned long)b->turn_us, (unsigned long)b->end_ns, (unsigned long)b->rx_err,
             (unsigned)b->row);
    test_run_info((unsigned char *)line);
}

/* S, D1, D2 를 바꾼다. 묶음과 묶음 사이에만 부른다. */
static void apply_timing(struct uwb7_timing t)
{
    timing = t;
    /* 드라이버의 단위는 1.0256 µs (= 40/39 µs) */
    dwt_setrxaftertxdelay(t.final_delay > RX_EARLY_US ? ((uint32_t)t.final_delay - RX_EARLY_US) * 39U / 40U : 0U);
    dwt_setrxtimeout(0);
}

/* This entry is called by the handoff project's TEST_DS_TWR_RESPONDER selection. */
int ds_twr_responder(void)
{
    char line[160];
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
#if TAG7_SWEEP
    timing = sweep_table[0];
#endif
    if (!uwb7_timing_ok(timing)) {
        test_run_info((unsigned char *)"TWR3_INIT_FAILED,bad_timing");
        for (;;) { }
    }
    test_run_info((unsigned char *)"TWR3_READY,channel=5,tag=256");
    snprintf(line, sizeof(line),
             "TWR3_INFO,fw=" TAG_FW ",packets=7,ant_dly=%u,s=%u,d1=%u,d2=%u,sweep=%u,sweep_bundles=%u,"
             "tx_lead_us=%u,opt=" TAG7_BUILD_OPT,
             (unsigned)UWB3_ANTENNA_DELAY, (unsigned)timing.slot, (unsigned)timing.response_delay,
             (unsigned)timing.final_delay, (unsigned)TAG7_SWEEP, (unsigned)TAG7_SWEEP_BUNDLES,
             (unsigned)tx_lead_us(timing));
    test_run_info((unsigned char *)line);
#if TAG7_SWEEP
    for (unsigned i = 0; i < SWEEP_ROWS; ++i) {
        snprintf(line, sizeof(line), "TWR3_ROW,%u,%u,%u,%u%s", i, (unsigned)sweep_table[i].slot,
                 (unsigned)sweep_table[i].response_delay, (unsigned)sweep_table[i].final_delay,
                 uwb7_timing_ok(sweep_table[i]) ? "" : ",skipped");
        test_run_info((unsigned char *)line);
    }
#endif

    apply_timing(timing);
    rx_on();
    for (;;) {
        uint32_t status;
        do {
            status = dwt_readsysstatuslo();
        } while ((status & UWB3_RX_EVENTS) == 0);

        struct uwb3_key key;
        uint8_t frame[UWB3_FINAL_LEN + UWB3_FCS];
        bool ran = false;
        if (status & DWT_INT_RXFCG_BIT_MASK) {
            const uint16_t length = uwb3_read(frame);
            if (uwb7_decode(frame, length, UWB7_POLL, &key)) {
                bundle = (struct bundle){ .t = timing, .no = ++bundle_no, .row = sweep_row, .resp = 'N' };
                for (unsigned i = 0; i < UWB3_ANCHORS; ++i) {
                    bundle.x[i].poll_power = (struct uwb3_power){ UWB3_UNKNOWN_POWER, UWB3_UNKNOWN_POWER };
                    bundle.x[i].final_power = bundle.x[i].poll_power;
                }
                run_bundle(&bundle, key, status);
                ran = true;
            } else if (uwb3_decode(frame, length, UWB3_POLL, &key)) {
                /* 9패킷 앵커의 Poll. 방식이 섞여 있다 */
                if (++legacy_frames == 1 || legacy_frames % 500U == 0) {
                    snprintf(line, sizeof(line), "TWR3_WARN,9packet_polls=%lu", (unsigned long)legacy_frames);
                    test_run_info((unsigned char *)line);
                }
            } else {
                ++rx_other;
            }
        } else if (status & SYS_STATUS_ALL_RX_ERR) {
            ++rx_errors;
        }
        dwt_forcetrxoff();
        if (ran) {
#if TAG7_SWEEP
            if (bundle_no % TAG7_SWEEP_BUNDLES == 0) {
                do {
                    sweep_row = (uint8_t)((sweep_row + 1U) % SWEEP_ROWS);
                } while (!uwb7_timing_ok(sweep_table[sweep_row]));
                apply_timing(sweep_table[sweep_row]);
            }
#endif
            rx_on(); /* 출력하는 동안에도 듣는다. 그 사이 들어온 패킷은 칩에 남아 있다가 다음 차례에 처리된다 */
            print_bundle(&bundle);
        } else {
            rx_on();
        }
    }
}
