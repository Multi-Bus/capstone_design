/* 빌드, 실행 (firmware/uwb/protocol/ 에서): gcc -std=c99 -Wall -Wextra -Iinclude -o test/test_uwb_protocol test/test_uwb_protocol.c src/uwb_protocol.c -lm && test/test_uwb_protocol */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "uwb_protocol.h"

static int state_eq(const uwb_state_t *a, const uwb_state_t *b) /* 패딩 때문에 memcmp 대신 필드 비교 */
{
    return a->id == b->id && a->seq == b->seq && a->t_src_us == b->t_src_us && a->pos_mm[0] == b->pos_mm[0]
        && a->pos_mm[1] == b->pos_mm[1] && a->vel_mms[0] == b->vel_mms[0] && a->vel_mms[1] == b->vel_mms[1]
        && a->eps_mm == b->eps_mm;
}

static void test_heartbeat_roundtrip(void)
{
    uwb_heartbeat_t hb = { 0 }, out;
    hb.state = (uwb_state_t){ 3, 513, 123456789u, { 15000, -2500 }, { -1200, 800 }, 150 };
    hb.n_ack = UWB_MAX_ACKS;
    for (int i = 0; i < UWB_MAX_ACKS; i++) hb.ack[i] = (uwb_ack_t){ (uint8_t)i, (uint8_t)(10 + i) };
    hb.n_bid = UWB_MAX_BIDS;
    for (int i = 0; i < UWB_MAX_BIDS; i++) hb.bid[i] = (uwb_bid_t){ (uint8_t)i, (int16_t)(-100 * i), 700, (uint8_t)(i == 2) };
    uint8_t buf[UWB_MAX_PAYLOAD];
    int n = uwb_encode_heartbeat(&hb, buf, sizeof buf);
    printf("heartbeat %d B (최대 ACK %d, 입찰 %d)\n", n, UWB_MAX_ACKS, UWB_MAX_BIDS);
    assert(n > 0 && n <= UWB_MAX_PAYLOAD);
    assert(uwb_decode_heartbeat(buf, n, &out) == n);
    assert(state_eq(&hb.state, &out.state));
    assert(out.n_ack == hb.n_ack && out.ack[7].version == 17);
    assert(out.bid[2].urgent == 1 && out.bid[2].score == -200 && out.bid[1].valid_until_frame == 700);
    assert(uwb_decode_heartbeat(buf, n - 1, &out) == -1); /* 잘린 패킷 거부 */
    assert(uwb_encode_heartbeat(&hb, buf, 10) == -1);       /* 공간 부족 */
}

static void test_state_and_schedule(void)
{
    uwb_state_t s = { 1, 2, 3, { 4, 5 }, { 6, 7 }, 8 }, o;
    uint8_t buf[64];
    int n = uwb_encode_state(&s, buf, sizeof buf);
    printf("state %d B\n", n);
    assert(n == 18 && uwb_decode_state(buf, n, &o) == n && state_eq(&s, &o));
    uwb_schedule_t sc = { 9, 1000, 2, { { 0, 3 }, { 1, 4 } } }, so;
    n = uwb_encode_schedule(&sc, buf, sizeof buf);
    assert(n > 0 && uwb_decode_schedule(buf, n, &so) == n && so.pair[1].b == 4 && so.frame == 1000);
    buf[0] = UWB_PKT_STATE;
    assert(uwb_decode_schedule(buf, n, &so) == -1); /* 종류 틀림 */
}

static uwb_tdma_cfg_t cfg6(void)
{
    /* 6대, 앵커 3, T_h = 5 프레임(100 ms), 측위 4슬롯/프레임(=10 Hz/기체), K = 3 */
    return (uwb_tdma_cfg_t){ 20000, 300, 300, 1500, 1500, 6, 3, 5, 4, 3 };
}

static void test_tdma(void)
{
    uwb_tdma_cfg_t c = cfg6();
    assert(uwb_tdma_fits(&c));
    uwb_slot_t s = uwb_tdma_slot_at(&c, 0, 100);
    assert(s.type == UWB_SLOT_BEACON);
    /* 프레임 0: heartbeat 칸 0 = 기체 0, 칸 1 = 기체 5 */
    s = uwb_tdma_slot_at(&c, 0, 300);
    assert(s.type == UWB_SLOT_HB && s.drone == 0);
    s = uwb_tdma_slot_at(&c, 0, 600);
    assert(s.type == UWB_SLOT_HB && s.drone == 5);
    /* hb_every 프레임 안에 모든 기체가 한 번씩 heartbeat */
    int seen[6] = { 0 };
    for (uint32_t f = 0; f < c.hb_every; f++)
        for (uint32_t t = 0; t < c.frame_us; t += 100) {
            s = uwb_tdma_slot_at(&c, f, t);
            if (s.type == UWB_SLOT_HB && s.start_us == t) seen[s.drone]++;
        }
    for (int i = 0; i < 6; i++) assert(seen[i] == 1);
    /* 측위: 18쌍이 18/4 프레임 안에 모두 돈다 -> 기체당 측위율 = 50 Hz * 4 / 6 / 3 */
    int pair_seen[18] = { 0 };
    for (uint32_t f = 0; f < 9; f++)
        for (uint32_t t = 0; t < c.frame_us; t += 100) {
            s = uwb_tdma_slot_at(&c, f, t);
            if (s.type == UWB_SLOT_LOC && s.start_us == t) pair_seen[s.drone * 3 + s.anchor]++;
        }
    for (int i = 0; i < 18; i++) assert(pair_seen[i] == 2);
    s = uwb_tdma_slot_at(&c, 0, 900 + 4 * 1500 + 1500 + 10);
    assert(s.type == UWB_SLOT_EX && s.drone == 1);
    s = uwb_tdma_slot_at(&c, 0, 19990);
    assert(s.type == UWB_SLOT_IDLE);
    c.loc_slot_us = 3000; /* DS-TWR 3 ms면 넘친다 */
    c.ex_slot_us = 3000;
    assert(!uwb_tdma_fits(&c));
}

static void test_schedule_pick(void)
{
    uwb_bid_t bids[UWB_MAX_DRONES][UWB_MAX_BIDS] = { { { 0 } } };
    uint8_t nb[UWB_MAX_DRONES] = { 0 };
    uint16_t age[UWB_MAX_DRONES][UWB_MAX_DRONES] = { { 0 } };
    uwb_pair_t out[UWB_MAX_K];
    /* 기체 0: 1과 점수 50. 기체 2: 1과 긴급. 기체 3: 4와 점수 80. 기체 5: 만료된 입찰 */
    bids[0][0] = (uwb_bid_t){ 1, 50, 100, 0 }; nb[0] = 1;
    bids[2][0] = (uwb_bid_t){ 1, 0, 100, 1 }; nb[2] = 1;
    bids[3][0] = (uwb_bid_t){ 4, 80, 100, 0 }; nb[3] = 1;
    bids[5][0] = (uwb_bid_t){ 0, 999, 90, 0 }; nb[5] = 1;
    age[0][5] = 900;
    int m = uwb_schedule_pick(6, 3, 100, bids, nb, age, out);
    assert(m == 3);
    assert(out[0].a == 2 && out[0].b == 1); /* 긴급 먼저 */
    assert(out[1].a == 3 && out[1].b == 4); /* 다음 점수 순. 0-1은 1이 바빠서 탈락 */
    assert(out[2].a == 0 && out[2].b == 5); /* 남는 슬롯은 오래된 쌍. 5의 만료 입찰은 무시 */
    /* 기체당 한 번 */
    int cnt[6] = { 0 };
    for (int i = 0; i < m; i++) { cnt[out[i].a]++; cnt[out[i].b]++; }
    for (int i = 0; i < 6; i++) assert(cnt[i] <= 1);
}

static void test_range_and_clock(void)
{
    uwb_state_t a = { 0, 0, 0, { 0, 0 }, { 0, 0 }, 100 }, b = { 1, 0, 0, { 3000, 4000 }, { 0, 0 }, 100 };
    assert(uwb_range_consistent(5000, &a, &b, 100));
    assert(uwb_range_consistent(5290, &a, &b, 100));
    assert(!uwb_range_consistent(5400, &a, &b, 100));
    uint64_t top = (1ULL << 40) - 10;
    assert(uwb_ts_diff(top, 5) == 15);   /* 랩어라운드 */
    assert(uwb_ts_diff(5, top) == -15);
    assert(uwb_clock_offset(1000, 400) == 600);
}

int main(void)
{
    test_heartbeat_roundtrip();
    test_state_and_schedule();
    test_tdma();
    test_schedule_pick();
    test_range_and_clock();
    puts("all ok");
    return 0;
}
