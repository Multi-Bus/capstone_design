/*
 * DWM3001CDK 앵커 (3대1 DS-TWR, 혼자 도는 플래시 버전)
 *
 * 태그(NUCLEO-N657 + DWM3000EVB)의 ds_twr_responder(= drone-hw-bench qorvo_3to1/tag.c)와 짝이다.
 * 앵커가 Poll 을 보내고, 태그가 Response, 앵커가 Final 을 보내면 태그가 거리를 계산해 출력한다.
 *
 * range() 는 drone-hw-bench `task6-qorvo-3to1-20261123` 브랜치(커밋 b934e69)의
 * task6_mixed/qorvo_3to1/anchor.c 를 그대로 옮겼다 (패킷·타이밍 동일).
 * 바꾼 점: 원래는 PC 가 J-Link(SWD) 메일박스로 A1 -> A2 -> A3 를 하나씩 호출했다.
 *          보조배터리로 따로 켜는 앵커는 PC 가 없으므로, 앵커끼리 서로의 Poll 을 듣고 차례를 맞춘다.
 *
 * 차례 맞추기
 *   - 칸(slot) 하나 = ANCHOR_SLOT_US (기본 20 ms), 한 바퀴 = 3칸.
 *   - 앵커 j 의 Poll 을 들으면, 내 다음 Poll = 그 Poll 시각 + ((내번호 - j) mod 3) x 칸.
 *     => A1, A2, A3 가 20 ms 간격으로 겹치지 않고 돌아간다.
 *   - 다른 앵커가 안 들리면 혼자 한 바퀴(60 ms)마다 보내고, 0~칸/2 난수를 더해 동시 송신이 이어지지 않게 한다.
 *   - 칸 20 ms 는 태그가 한 번 교환할 때마다 115200 bps 로 거리 한 줄(약 8 ms)을 출력하는 시간을 감안한 값이다.
 *   - 세 앵커는 같은 ANCHOR_SLOT_US 로 빌드해야 한다.
 */
#include <stdio.h>
#include "nrf.h"
#include "deca_probe_interface.h"
#include "deca_spi.h"
#include "port.h"
#include "radio.h"
#include "radio_config.h"

#ifndef UWB3_ANCHOR_ID
#error Set UWB3_ANCHOR_ID (1, 2 or 3) to the address of this board.
#endif
#if UWB3_ANCHOR_ID < 1 || UWB3_ANCHOR_ID > 3
#error UWB3_ANCHOR_ID must be 1, 2 or 3.
#endif
#ifndef UWB3_ANTENNA_DELAY
#define UWB3_ANTENNA_DELAY 16385U
#endif
#ifndef ANCHOR_SLOT_US
#define ANCHOR_SLOT_US 20000U
#endif
#define FW_VERSION "anchor3to1-v1"

enum { SENT_POLL = 1, RECEIVED_RESPONSE = 2, SENT_FINAL = 4 };
enum {
    CYCLE_US = 3U * ANCHOR_SLOT_US,
    HEARD_LATENCY_US = 200U,      /* 남의 Poll 시작 ~ 내가 수신 완료를 알아챈 때 (프리앰블 128 + 프레임 약 0.2 ms) */
    PREP_US = 300U,               /* 내 차례 이만큼 전에 듣기를 멈춤 */
    SYNC_LOST_US = 3U * CYCLE_US, /* 이 시간 동안 남의 Poll 이 없으면 혼자 모드 */
    TAG_LED_US = 500000U,
    STATUS_PERIOD_US = 1000000U,
};

struct exchange {
    uint32_t stages, response_status, final_status;
    int16_t rssi, first_path;
};

/* 디버거(SWD)로도 읽을 수 있게 전역에 둔다 */
struct anchor_stats {
    uint32_t polls, responses, finals;
    uint32_t heard[4];        /* [j] = 앵커 j 의 Poll 을 들은 횟수 */
    uint32_t dup_id;          /* 내 번호와 같은 번호의 Poll (번호 중복 설정!) */
    uint32_t resyncs;
};
volatile struct anchor_stats g_stats;

static uint32_t next_tx_us;
static uint32_t last_heard_us;
static bool heard_any;

/* ------------------------------------------------------------------ 측정 한 번 (학생 anchor.c 와 같음) */
static uint32_t wait_events(uint32_t mask)
{
    const uint32_t start = cdk_time_us();
    uint32_t status;
    do {
        status = dwt_readsysstatuslo();
    } while ((status & mask) == 0 && cdk_time_us() - start < 10000U);
    return status;
}

static void range(struct uwb3_key key, struct exchange *ex)
{
    uint8_t frame[UWB3_FINAL_LEN + UWB3_FCS];
    struct uwb3_key response;
    dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
    uwb3_encode(frame, UWB3_POLL, key);
    if (uwb3_send(frame, UWB3_HEADER, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        return;
    }

    const uint32_t status = wait_events(UWB3_RX_EVENTS);
    ex->response_status = status;
    if (status & DWT_INT_TXFRS_BIT_MASK) {
        ex->stages |= SENT_POLL;
    }
    if (!(status & DWT_INT_RXFCG_BIT_MASK) ||
        !uwb3_decode(frame, uwb3_read(frame), UWB3_RESPONSE, &response) ||
        !uwb3_same_key(key, response)) {
        return;
    }
    ex->stages |= RECEIVED_RESPONSE;
    dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
    const uint64_t poll_tx = uwb3_timestamp(true);
    const uint64_t response_rx = uwb3_timestamp(false);
    const uint32_t delayed = (uint32_t)((response_rx + (uint64_t)UWB3_FINAL_DELAY * UWB3_DTU_PER_UUS) >> 8);
    dwt_setdelayedtrxtime(delayed);
    uwb3_encode(frame, UWB3_FINAL, key);
    uwb3_put32(frame + UWB3_HEADER, (uint32_t)poll_tx);
    uwb3_put32(frame + UWB3_HEADER + 4, (uint32_t)response_rx);
    uwb3_put32(frame + UWB3_HEADER + 8, ((delayed & ~1U) << 8) + UWB3_ANTENNA_DELAY);
    const int sent = uwb3_send(frame, UWB3_FINAL_LEN, DWT_START_TX_DELAYED);
    const struct uwb3_power power = uwb3_read_power();
    ex->rssi = power.rssi;
    ex->first_path = power.first_path;
    if (sent == DWT_SUCCESS) {
        ex->final_status = wait_events(DWT_INT_TXFRS_BIT_MASK);
        if (ex->final_status & DWT_INT_TXFRS_BIT_MASK) {
            ex->stages |= SENT_FINAL;
        }
    }
}

/* ------------------------------------------------------------------ 차례 맞추기 */
static void on_heard_poll(uint16_t from, uint32_t now)
{
    if (from == UWB3_ANCHOR_ID) {
        ++g_stats.dup_id;
        return;
    }
    ++g_stats.heard[from];
    if (!heard_any || (uint32_t)(now - last_heard_us) >= SYNC_LOST_US) {
        ++g_stats.resyncs;
    }
    heard_any = true;
    last_heard_us = now;
    const uint32_t offset = ((UWB3_ANCHOR_ID + 3U - from) % 3U) * ANCHOR_SLOT_US;
    next_tx_us = now - HEARD_LATENCY_US + offset;
}

static bool synced(uint32_t now)
{
    return heard_any && (uint32_t)(now - last_heard_us) < SYNC_LOST_US;
}

/* 내 차례(next_tx_us - PREP_US)가 될 때까지 수신하면서 남의 Poll 을 듣는다. next_tx_us 는 도중에 바뀔 수 있다. */
static void listen_until_my_turn(void)
{
    uint8_t frame[UWB3_FINAL_LEN + UWB3_FCS];
    dwt_setrxtimeout(0);
    dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    for (;;) {
        port_wdt_feed();
        const uint32_t now = cdk_time_us();
        if ((int32_t)(now - (next_tx_us - PREP_US)) >= 0) {
            break;
        }
        const uint32_t status = dwt_readsysstatuslo();
        if (status & DWT_INT_RXFCG_BIT_MASK) {
            struct uwb3_key key;
            const uint16_t length = uwb3_read(frame);
            if (length != 0 && uwb3_decode(frame, length, UWB3_POLL, &key)) {
                on_heard_poll(key.anchor, now);
            }
            dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        } else if (status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
    }
    dwt_forcetrxoff();
    dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
}

/* ------------------------------------------------------------------ 초기화 */
static const char *radio_init(struct radio_power_info *power)
{
    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
        return "probe";
    }
    if (!dwt_checkidlerc()) {
        return "idlerc";
    }
    if (dwt_initialise(DWT_READ_OTP_ALL) != DWT_SUCCESS) {
        return "initialise";
    }
    if (dwt_configure(&config_options) != DWT_SUCCESS) {
        return "configure";
    }
    port_set_dw_ic_spi_fastrate();
    if (radio_apply_tx_power(power) != DWT_SUCCESS) {
        return "tx_power";
    }
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_settxantennadelay(UWB3_ANTENNA_DELAY);
    dwt_setrxantennadelay(UWB3_ANTENNA_DELAY);
    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);
    dwt_setrxaftertxdelay(UWB3_RESPONSE_RX_DELAY);
    dwt_setrxtimeout(UWB3_RESPONSE_TIMEOUT);
    dwt_setpreambledetecttimeout(0);
    return NULL;
}

int main(void)
{
    char line[192];
    const uint32_t reset_reason = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS = reset_reason;
    port_init();

    uint32_t session = 0;
    while (session == 0) {
        session = port_random32();
    }
    snprintf(line, sizeof(line),
             "A%u_BOOT,fw=" FW_VERSION ",ficr=%08lX%08lX,reset=0x%05lX,dev_id=0x%08lX,session=0x%08lX,"
             "slot_us=%u,ant_dly=%u",
             (unsigned)UWB3_ANCHOR_ID, (unsigned long)NRF_FICR->DEVICEID[1], (unsigned long)NRF_FICR->DEVICEID[0],
             (unsigned long)reset_reason, (unsigned long)port_dw_device_id(), (unsigned long)session,
             (unsigned)ANCHOR_SLOT_US, (unsigned)UWB3_ANTENNA_DELAY);
    port_log(line);

    struct radio_power_info power = { 0 };
    const char *failed = radio_init(&power);
    port_log_flush(20000U);
    if (failed != NULL) {
        snprintf(line, sizeof(line), "A%u_INIT_FAILED,step=%s,dev_id=0x%08lX", (unsigned)UWB3_ANCHOR_ID, failed,
                 (unsigned long)port_dw_device_id());
        port_log(line);
        port_led(LED_D9_ERROR, true);
        for (;;) { } /* 감시 타이머가 2 s 뒤 재시작 -> 초기화 다시 시도 */
    }
    snprintf(line, sizeof(line), "A%u_READY,channel=5,tag=256,power_index=%lu,tx_power=0x%08lX,pll_bias=%lu",
             (unsigned)UWB3_ANCHOR_ID, (unsigned long)power.power_index, (unsigned long)power.tx_power,
             (unsigned long)power.pll_bias);
    port_log(line);
    port_log_flush(20000U);

    /* 처음에는 두 바퀴 동안 듣기만 해서 이미 돌고 있는 앵커에 맞춘다. 아무도 없으면 난수 시각에 시작. */
    next_tx_us = cdk_time_us() + 2U * CYCLE_US + port_random32() % ANCHOR_SLOT_US;
    uint32_t round = 0;
    uint32_t last_status_us = cdk_time_us();
    uint32_t last_response_us = cdk_time_us() - TAG_LED_US;

    for (;;) {
        listen_until_my_turn();
        uint32_t now = cdk_time_us();
        if ((int32_t)(now - (next_tx_us - PREP_US)) < 0) {
            continue; /* 듣는 도중 차례가 뒤로 밀렸음 */
        }
        while ((int32_t)(cdk_time_us() - next_tx_us) < 0) { }

        if (++round == 0) {
            round = 1;
        }
        const struct uwb3_key key = { UWB3_ANCHOR_ID, session, round };
        struct exchange ex = { 0, 0, 0, UWB3_UNKNOWN_POWER, UWB3_UNKNOWN_POWER };
        dwt_setrxaftertxdelay(UWB3_RESPONSE_RX_DELAY);
        dwt_setrxtimeout(UWB3_RESPONSE_TIMEOUT);
        range(key, &ex);
        dwt_forcetrxoff();
        dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);

        now = cdk_time_us();
        if (ex.stages & SENT_POLL) {
            ++g_stats.polls;
            port_led_toggle(LED_D10_POLL);
        }
        if (ex.stages & RECEIVED_RESPONSE) {
            ++g_stats.responses;
            last_response_us = now;
        }
        if (ex.stages & SENT_FINAL) {
            ++g_stats.finals;
        }
        const bool is_synced = synced(now);
        port_led(LED_D11_TAG, (uint32_t)(now - last_response_us) < TAG_LED_US);
        port_led(LED_D12_SYNC, is_synced);

        /* A<번호>,<회차>,<단계 1=Poll 2=Response 4=Final>,<Response RSSI>,<첫경로>,<동기 1/0>,<Response 상태> */
        snprintf(line, sizeof(line), "A%u,%lu,%lu,%d,%d,%u,%08lX", (unsigned)UWB3_ANCHOR_ID, (unsigned long)round,
                 (unsigned long)ex.stages, ex.rssi, ex.first_path, (unsigned)is_synced,
                 (unsigned long)ex.response_status);
        port_log(line);

        next_tx_us += CYCLE_US;
        if (!is_synced) {
            next_tx_us += port_random32() % (ANCHOR_SLOT_US / 2U);
        }
        if ((int32_t)(next_tx_us - now) < (int32_t)PREP_US) {
            next_tx_us = now + CYCLE_US;
        }

        if ((uint32_t)(now - last_status_us) >= STATUS_PERIOD_US) {
            last_status_us = now;
            snprintf(line, sizeof(line),
                     "A%u_STAT,polls=%lu,resp=%lu,finals=%lu,heard1=%lu,heard2=%lu,heard3=%lu,dup_id=%lu,"
                     "resyncs=%lu,synced=%u,log_drop=%lu",
                     (unsigned)UWB3_ANCHOR_ID, (unsigned long)g_stats.polls, (unsigned long)g_stats.responses,
                     (unsigned long)g_stats.finals, (unsigned long)g_stats.heard[1], (unsigned long)g_stats.heard[2],
                     (unsigned long)g_stats.heard[3], (unsigned long)g_stats.dup_id, (unsigned long)g_stats.resyncs,
                     (unsigned)is_synced, (unsigned long)port_log_dropped());
            port_log_flush(10000U); /* 바로 앞 줄이 아직 나가는 중일 수 있음 */
            port_log(line);
        }
    }
}
