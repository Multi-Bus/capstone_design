#include "uwb_protocol.h"

#include <math.h>
#include <string.h>

/* ---- 리틀 엔디언 쓰기/읽기 ---- */
typedef struct { uint8_t *p; int n, cap, err; } wr_t;
typedef struct { const uint8_t *p; int n, len, err; } rd_t;

static void w8(wr_t *w, uint32_t v) { if (w->n >= w->cap) { w->err = 1; return; } w->p[w->n++] = (uint8_t)v; }
static void w16(wr_t *w, uint32_t v) { w8(w, v); w8(w, v >> 8); }
static void w32(wr_t *w, uint32_t v) { w16(w, v); w16(w, v >> 16); }
static uint32_t r8(rd_t *r) { if (r->n >= r->len) { r->err = 1; return 0; } return r->p[r->n++]; }
static uint32_t r16(rd_t *r) { uint32_t lo = r8(r); return lo | (r8(r) << 8); }
static uint32_t r32(rd_t *r) { uint32_t lo = r16(r); return lo | (r16(r) << 16); }

static void w_state(wr_t *w, const uwb_state_t *s)
{
    w8(w, s->id); w16(w, s->seq); w32(w, s->t_src_us);
    w16(w, (uint16_t)s->pos_mm[0]); w16(w, (uint16_t)s->pos_mm[1]);
    w16(w, (uint16_t)s->vel_mms[0]); w16(w, (uint16_t)s->vel_mms[1]);
    w16(w, s->eps_mm);
}

static void r_state(rd_t *r, uwb_state_t *s)
{
    s->id = (uint8_t)r8(r); s->seq = (uint16_t)r16(r); s->t_src_us = r32(r);
    s->pos_mm[0] = (int16_t)r16(r); s->pos_mm[1] = (int16_t)r16(r);
    s->vel_mms[0] = (int16_t)r16(r); s->vel_mms[1] = (int16_t)r16(r);
    s->eps_mm = (uint16_t)r16(r);
}

int uwb_encode_state(const uwb_state_t *s, uint8_t *buf, int cap)
{
    wr_t w = { buf, 0, cap, 0 };
    w8(&w, UWB_PKT_STATE);
    w_state(&w, s);
    return w.err ? -1 : w.n;
}

int uwb_decode_state(const uint8_t *buf, int len, uwb_state_t *s)
{
    rd_t r = { buf, 0, len, 0 };
    if (r8(&r) != UWB_PKT_STATE) return -1;
    r_state(&r, s);
    return r.err ? -1 : r.n;
}

int uwb_encode_heartbeat(const uwb_heartbeat_t *hb, uint8_t *buf, int cap)
{
    if (hb->n_ack > UWB_MAX_ACKS || hb->n_bid > UWB_MAX_BIDS) return -1;
    wr_t w = { buf, 0, cap, 0 };
    w8(&w, UWB_PKT_HEARTBEAT);
    w_state(&w, &hb->state);
    w8(&w, hb->n_ack);
    for (int i = 0; i < hb->n_ack; i++) { w8(&w, hb->ack[i].id); w8(&w, hb->ack[i].version); }
    w8(&w, hb->n_bid);
    for (int i = 0; i < hb->n_bid; i++) {
        const uwb_bid_t *b = &hb->bid[i];
        w8(&w, b->peer | (b->urgent ? 0x80 : 0)); w16(&w, (uint16_t)b->score); w16(&w, b->valid_until_frame);
    }
    return w.err ? -1 : w.n;
}

int uwb_decode_heartbeat(const uint8_t *buf, int len, uwb_heartbeat_t *hb)
{
    rd_t r = { buf, 0, len, 0 };
    memset(hb, 0, sizeof *hb);
    if (r8(&r) != UWB_PKT_HEARTBEAT) return -1;
    r_state(&r, &hb->state);
    hb->n_ack = (uint8_t)r8(&r);
    if (hb->n_ack > UWB_MAX_ACKS) return -1;
    for (int i = 0; i < hb->n_ack; i++) { hb->ack[i].id = (uint8_t)r8(&r); hb->ack[i].version = (uint8_t)r8(&r); }
    hb->n_bid = (uint8_t)r8(&r);
    if (hb->n_bid > UWB_MAX_BIDS) return -1;
    for (int i = 0; i < hb->n_bid; i++) {
        uint8_t pe = (uint8_t)r8(&r);
        hb->bid[i].peer = pe & 0x7F; hb->bid[i].urgent = pe >> 7;
        hb->bid[i].score = (int16_t)r16(&r); hb->bid[i].valid_until_frame = (uint16_t)r16(&r);
    }
    return r.err ? -1 : r.n;
}

int uwb_encode_schedule(const uwb_schedule_t *s, uint8_t *buf, int cap)
{
    if (s->n_pair > UWB_MAX_K) return -1;
    wr_t w = { buf, 0, cap, 0 };
    w8(&w, UWB_PKT_SCHEDULE); w16(&w, s->epoch); w16(&w, s->frame); w8(&w, s->n_pair);
    for (int i = 0; i < s->n_pair; i++) { w8(&w, s->pair[i].a); w8(&w, s->pair[i].b); }
    return w.err ? -1 : w.n;
}

int uwb_decode_schedule(const uint8_t *buf, int len, uwb_schedule_t *s)
{
    rd_t r = { buf, 0, len, 0 };
    memset(s, 0, sizeof *s);
    if (r8(&r) != UWB_PKT_SCHEDULE) return -1;
    s->epoch = (uint16_t)r16(&r); s->frame = (uint16_t)r16(&r); s->n_pair = (uint8_t)r8(&r);
    if (s->n_pair > UWB_MAX_K) return -1;
    for (int i = 0; i < s->n_pair; i++) { s->pair[i].a = (uint8_t)r8(&r); s->pair[i].b = (uint8_t)r8(&r); }
    return r.err ? -1 : r.n;
}

/* ---- TDMA ----
 * 프레임 배치: [비콘][heartbeat x hb_per_frame][측위 x loc_per_frame][교환 x k][남는 시간]
 * heartbeat: 기체 i는 frame % hb_every == i % hb_every 인 프레임에, i / hb_every 번째 칸.
 * 측위: (기체, 앵커) 쌍을 프레임마다 loc_per_frame개씩 돌아가며.
 */
static uint32_t hb_per_frame(const uwb_tdma_cfg_t *c) { return (c->n_drones + c->hb_every - 1) / c->hb_every; }

int uwb_tdma_fits(const uwb_tdma_cfg_t *c)
{
    uint64_t used = (uint64_t)c->beacon_us + hb_per_frame(c) * c->hb_slot_us
                  + (uint64_t)c->loc_per_frame * c->loc_slot_us + (uint64_t)c->k * c->ex_slot_us;
    return c->hb_every > 0 && c->k <= UWB_MAX_K && used <= c->frame_us;
}

uwb_slot_t uwb_tdma_slot_at(const uwb_tdma_cfg_t *c, uint32_t frame, uint32_t t_us)
{
    uwb_slot_t s = { UWB_SLOT_IDLE, 0, 0, 0 };
    uint32_t edge = c->beacon_us;
    if (t_us < edge) { s.type = UWB_SLOT_BEACON; return s; }
    uint32_t idx = (t_us - edge) / c->hb_slot_us;
    if (idx < hb_per_frame(c)) {
        uint32_t drone = idx * c->hb_every + frame % c->hb_every;
        s.start_us = edge + idx * c->hb_slot_us;
        if (drone < c->n_drones) { s.type = UWB_SLOT_HB; s.drone = (uint8_t)drone; }
        return s;
    }
    edge += hb_per_frame(c) * c->hb_slot_us;
    idx = (t_us - edge) / c->loc_slot_us;
    if (idx < c->loc_per_frame) {
        uint32_t pairs = (uint32_t)c->n_drones * c->n_anchors;
        uint32_t q = (frame * c->loc_per_frame + idx) % pairs;
        s.type = UWB_SLOT_LOC; s.drone = (uint8_t)(q / c->n_anchors); s.anchor = (uint8_t)(q % c->n_anchors);
        s.start_us = edge + idx * c->loc_slot_us;
        return s;
    }
    edge += c->loc_per_frame * c->loc_slot_us;
    idx = (t_us - edge) / c->ex_slot_us;
    if (idx < c->k) { s.type = UWB_SLOT_EX; s.drone = (uint8_t)idx; s.start_us = edge + idx * c->ex_slot_us; }
    return s;
}

/* ---- 조정자 배정 (논문 4.2: 긴급 -> 양수 점수 탐욕 -> 남는 슬롯은 오래된 레코드) ---- */
int uwb_schedule_pick(uint8_t n, uint8_t k, uint16_t frame,
                      const uwb_bid_t bids_of[][UWB_MAX_BIDS], const uint8_t n_bids_of[],
                      const uint16_t age_ms[][UWB_MAX_DRONES], uwb_pair_t out[])
{
    uint8_t busy[UWB_MAX_DRONES] = { 0 };
    int m = 0;
    if (k > UWB_MAX_K) k = UWB_MAX_K;
    for (int pass = 0; pass < 2 && m < k; pass++) { /* 0: 긴급, 1: 점수 > 0 */
        for (;;) {
            int bi = -1, bj = -1, best = 0;
            for (int i = 0; i < n; i++) {
                if (busy[i]) continue;
                for (int b = 0; b < n_bids_of[i]; b++) {
                    const uwb_bid_t *x = &bids_of[i][b];
                    if (x->peer >= n || x->peer == i || busy[x->peer]) continue;
                    if ((uint16_t)(frame - x->valid_until_frame) < 0x8000 && frame != x->valid_until_frame) continue; /* 만료 */
                    if (pass == 0 ? !x->urgent : (x->urgent || x->score <= 0)) continue;
                    int sc = pass == 0 ? 1 : x->score;
                    if (bi < 0 || sc > best) { bi = i; bj = x->peer; best = sc; }
                }
            }
            if (bi < 0 || m >= k) break;
            out[m].a = (uint8_t)bi; out[m].b = (uint8_t)bj; m++;
            busy[bi] = busy[bj] = 1;
        }
    }
    while (m < k) { /* 남는 슬롯: 가장 오래된 레코드 쌍 */
        int bi = -1, bj = -1;
        uint32_t best = 0;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                if (i == j || busy[i] || busy[j]) continue;
                uint32_t a = (uint32_t)age_ms[i][j] + age_ms[j][i];
                if (bi < 0 || a > best) { bi = i; bj = j; best = a; }
            }
        if (bi < 0) break;
        out[m].a = (uint8_t)bi; out[m].b = (uint8_t)bj; m++;
        busy[bi] = busy[bj] = 1;
    }
    return m;
}

int uwb_range_consistent(int32_t range_mm, const uwb_state_t *a, const uwb_state_t *b, uint16_t sigma_r_mm)
{
    double dx = (double)a->pos_mm[0] - b->pos_mm[0], dy = (double)a->pos_mm[1] - b->pos_mm[1];
    double err = fabs((double)range_mm - sqrt(dx * dx + dy * dy));
    return err <= (double)a->eps_mm + b->eps_mm + sigma_r_mm;
}

#define TS_MASK ((1ULL << 40) - 1)

int64_t uwb_ts_diff(uint64_t a, uint64_t b)
{
    uint64_t d = (b - a) & TS_MASK;
    return d >= (1ULL << 39) ? (int64_t)d - (int64_t)(1ULL << 40) : (int64_t)d;
}

int64_t uwb_clock_offset(uint64_t beacon_tx_coord, uint64_t beacon_rx_local)
{
    /* ponytail: 전파 시간(15 m에 50 ns)과 시계 drift는 무시. drift 보정은 U2 측정 후 필요하면 추가 */
    return uwb_ts_diff(beacon_rx_local, beacon_tx_coord);
}
