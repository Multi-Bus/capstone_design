#ifndef UWB_3TO1_FRAMES_H
#define UWB_3TO1_FRAMES_H

#include <stdbool.h>
#include <stdint.h>

enum {
    UWB3_PAN = 0xDECA, UWB3_TAG = 0x0100, UWB3_ANCHORS = 3,
    UWB3_POLL = 0x21, UWB3_RESPONSE = 0x10, UWB3_FINAL = 0x23,
    UWB3_HEADER = 18, UWB3_FINAL_LEN = UWB3_HEADER + 12, UWB3_FCS = 2,
    UWB3_RESPONSE_DELAY = 900, UWB3_FINAL_DELAY = 700,
    UWB3_RESPONSE_RX_DELAY = 700, UWB3_FINAL_RX_DELAY = 500,
    UWB3_RESPONSE_TIMEOUT = 300, UWB3_FINAL_TIMEOUT = 220,
    UWB3_DTU_PER_UUS = 63898, UWB3_UNKNOWN_POWER = 32767,
};

struct uwb3_key { uint16_t anchor; uint32_t session; uint32_t round; };

static inline void uwb3_put32(uint8_t *out, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) {
        out[i] = (uint8_t)(value >> (8 * i));
    }
}

static inline uint32_t uwb3_get32(const uint8_t *in)
{
    uint32_t value = 0;
    for (unsigned i = 0; i < 4; ++i) {
        value |= (uint32_t)in[i] << (8 * i);
    }
    return value;
}

static inline uint16_t uwb3_get16(const uint8_t *in)
{
    return (uint16_t)in[0] | ((uint16_t)in[1] << 8);
}

static inline void uwb3_encode(uint8_t *frame, uint8_t type, struct uwb3_key key)
{
    const uint16_t source = type == UWB3_RESPONSE ? UWB3_TAG : key.anchor;
    const uint16_t destination = type == UWB3_RESPONSE ? key.anchor : UWB3_TAG;
    frame[0] = 0x41;
    frame[1] = 0x88;
    frame[2] = (uint8_t)key.round;
    frame[3] = (uint8_t)UWB3_PAN;
    frame[4] = UWB3_PAN >> 8;
    frame[5] = (uint8_t)destination;
    frame[6] = destination >> 8;
    frame[7] = (uint8_t)source;
    frame[8] = source >> 8;
    frame[9] = type;
    uwb3_put32(frame + 10, key.session);
    uwb3_put32(frame + 14, key.round);
}

/* Length is the received PSDU length, including its two-byte FCS. */
static inline bool uwb3_decode(const uint8_t *frame, uint16_t length,
                               uint8_t type, struct uwb3_key *key)
{
    const bool response = type == UWB3_RESPONSE;
    const uint16_t expected = (type == UWB3_FINAL ? UWB3_FINAL_LEN : UWB3_HEADER) + UWB3_FCS;
    if (length != expected || frame[0] != 0x41 || frame[1] != 0x88 ||
        uwb3_get16(frame + 3) != UWB3_PAN || frame[9] != type ||
        uwb3_get16(frame + (response ? 7 : 5)) != UWB3_TAG) {
        return false;
    }
    struct uwb3_key candidate = { uwb3_get16(frame + (response ? 5 : 7)),
                                  uwb3_get32(frame + 10), uwb3_get32(frame + 14) };
    if (candidate.anchor < 1 || candidate.anchor > UWB3_ANCHORS ||
        candidate.session == 0 || candidate.round == 0 || frame[2] != (uint8_t)candidate.round) {
        return false;
    }
    *key = candidate;
    return true;
}

static inline bool uwb3_same_key(struct uwb3_key left, struct uwb3_key right)
{
    return left.anchor == right.anchor && left.session == right.session && left.round == right.round;
}

/* Unsigned subtraction handles rollover of the low 32 bits of the DW clock. */
static inline bool uwb3_distance_mm(const uint8_t *final, uint32_t poll_rx,
                                    uint32_t response_tx, uint32_t final_rx, int32_t *mm)
{
    const uint32_t poll_tx = uwb3_get32(final + UWB3_HEADER);
    const uint32_t response_rx = uwb3_get32(final + UWB3_HEADER + 4);
    const uint32_t final_tx = uwb3_get32(final + UWB3_HEADER + 8);
    const uint32_t ra = response_rx - poll_tx;
    const uint32_t rb = final_rx - response_tx;
    const uint32_t da = final_tx - response_rx;
    const uint32_t db = response_tx - poll_rx;
    const uint32_t limit = 3000U * UWB3_DTU_PER_UUS;
    if (ra == 0 || rb == 0 || da == 0 || db == 0 || ra > limit || rb > limit || da > limit || db > limit) {
        return false;
    }
    const double dtu = ((double)ra * rb - (double)da * db) / ((double)ra + rb + da + db);
    *mm = (int32_t)(dtu * (299702547000.0 / (499200000.0 * 128.0)));
    return true;
}

/* ------------------------------------------------------------------ 7패킷(묶음) 방식
 *
 *   Poll 1, Poll 2, Poll 3 (앵커, 간격 S) -> Response (태그, 방송 한 번) -> Final 1, 2, 3 (앵커, 간격 S)
 *
 *   앵커 1 의 Poll 을 0 으로 두면: Poll k = (k-1)S, Response = 2S + D1, Final k = 2S + D1 + D2 + (k-1)S
 *
 * 9패킷과 섞이지 않게 패킷 종류 값을 따로 쓴다. Poll 과 Final 의 모양은 9패킷과 같고 종류만 다르다.
 * Response(30바이트, Final 과 같은 길이)
 *   [0..1] 0x41 0x88  [2] 묶음 번호 아래 8비트  [3..4] PAN  [5..6] 받는 곳 0xFFFF(모두)  [7..8] 태그 주소
 *   [9] 종류  [10..11] S  [12..13] D1  [14..15] D2 (µs)  [16..17] 묶음 번호 아래 16비트
 *   [18..29] 태그가 받은 Poll 의 회차 번호 3개 (앵커 1, 2, 3 순서, 못 받았으면 0)
 * S, D1, D2 는 태그가 정해서 Response 에 싣고, 앵커는 들은 값을 따른다(Final 시각, 다음 묶음의 Poll 시각).
 * 그래서 값을 바꿔 가며 잴 때 태그만 다시 올리면 된다. 앵커의 빌드 값은 태그를 듣기 전까지만 쓴다.
 */
#ifndef UWB7_SLOT_US
#define UWB7_SLOT_US 600
#endif
#ifndef UWB7_RESPONSE_DELAY_US
#define UWB7_RESPONSE_DELAY_US 900
#endif
#ifndef UWB7_FINAL_DELAY_US
#define UWB7_FINAL_DELAY_US 700
#endif

enum {
    UWB7_POLL = 0x31, UWB7_RESPONSE = 0x30, UWB7_FINAL = 0x33,
    UWB7_BROADCAST = 0xFFFF,
    UWB7_RESPONSE_LEN = UWB3_HEADER + 4 * UWB3_ANCHORS,
    UWB7_TIME_MIN = 200,        /* S, D1, D2 로 받아들이는 가장 작은 값 (µs) */
    UWB7_INTERVAL_MAX = 2900,   /* uwb3_distance_mm 은 간격이 3000 µs 를 넘으면 버린다. 100 µs 여유 */
    UWB7_SHR_US = 139,          /* 프리앰블 128 + SFD 8 심볼 = 138.4 µs. 예약 송신 시각(RMARKER)보다 이만큼 먼저 송신이 시작됨 */
};

struct uwb7_timing { uint16_t slot, response_delay, final_delay; }; /* S, D1, D2 (µs) */

struct uwb7_response {
    struct uwb7_timing timing;
    uint16_t bundle;
    uint32_t round[UWB3_ANCHORS];
};

/* 거리 계산에 쓰는 가장 긴 간격: 앵커 1 은 Poll->Response 가 2S + D1, 앵커 3 은 Response->Final 이 D2 + 2S */
static inline bool uwb7_timing_ok(struct uwb7_timing t)
{
    return t.slot >= UWB7_TIME_MIN && t.response_delay >= UWB7_TIME_MIN && t.final_delay >= UWB7_TIME_MIN &&
           2U * t.slot + t.response_delay <= UWB7_INTERVAL_MAX && 2U * t.slot + t.final_delay <= UWB7_INTERVAL_MAX;
}

/* µs -> DW 시각 상위 32비트 단위 (1 = 256 DTU = 4.006 ns). 예약 송신 시각과 시스템 시각이 이 단위다. */
static inline uint32_t uwb7_us_to_hi(uint32_t us)
{
    return (uint32_t)(((uint64_t)us * UWB3_DTU_PER_UUS) >> 8);
}

static inline uint16_t uwb7_get16(const uint8_t *in)
{
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static inline void uwb7_put16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

/* Poll(18바이트)과 Final(30바이트, 시각 3개는 호출한 쪽이 채움). 9패킷과 같은 모양에 종류만 바꾼다. */
static inline void uwb7_encode(uint8_t *frame, uint8_t type, struct uwb3_key key)
{
    uwb3_encode(frame, UWB3_POLL, key);
    frame[9] = type;
}

/* Length is the received PSDU length, including its two-byte FCS. type 은 UWB7_POLL 또는 UWB7_FINAL. */
static inline bool uwb7_decode(const uint8_t *frame, uint16_t length, uint8_t type, struct uwb3_key *key)
{
    const uint16_t expected = (type == UWB7_FINAL ? UWB3_FINAL_LEN : UWB3_HEADER) + UWB3_FCS;
    if (length != expected || frame[0] != 0x41 || frame[1] != 0x88 ||
        uwb3_get16(frame + 3) != UWB3_PAN || frame[9] != type || uwb3_get16(frame + 5) != UWB3_TAG) {
        return false;
    }
    struct uwb3_key candidate = { uwb3_get16(frame + 7), uwb3_get32(frame + 10), uwb3_get32(frame + 14) };
    if (candidate.anchor < 1 || candidate.anchor > UWB3_ANCHORS ||
        candidate.session == 0 || candidate.round == 0 || frame[2] != (uint8_t)candidate.round) {
        return false;
    }
    *key = candidate;
    return true;
}

static inline void uwb7_encode_response(uint8_t *frame, const struct uwb7_response *response)
{
    frame[0] = 0x41;
    frame[1] = 0x88;
    frame[2] = (uint8_t)response->bundle;
    frame[3] = (uint8_t)UWB3_PAN;
    frame[4] = UWB3_PAN >> 8;
    frame[5] = (uint8_t)UWB7_BROADCAST;
    frame[6] = UWB7_BROADCAST >> 8;
    frame[7] = (uint8_t)UWB3_TAG;
    frame[8] = UWB3_TAG >> 8;
    frame[9] = UWB7_RESPONSE;
    uwb7_put16(frame + 10, response->timing.slot);
    uwb7_put16(frame + 12, response->timing.response_delay);
    uwb7_put16(frame + 14, response->timing.final_delay);
    uwb7_put16(frame + 16, response->bundle);
    for (unsigned i = 0; i < UWB3_ANCHORS; ++i) {
        uwb3_put32(frame + UWB3_HEADER + 4 * i, response->round[i]);
    }
}

static inline bool uwb7_decode_response(const uint8_t *frame, uint16_t length, struct uwb7_response *response)
{
    if (length != UWB7_RESPONSE_LEN + UWB3_FCS || frame[0] != 0x41 || frame[1] != 0x88 ||
        uwb3_get16(frame + 3) != UWB3_PAN || frame[9] != UWB7_RESPONSE ||
        uwb3_get16(frame + 5) != UWB7_BROADCAST || uwb3_get16(frame + 7) != UWB3_TAG) {
        return false;
    }
    struct uwb7_response candidate = { { uwb7_get16(frame + 10), uwb7_get16(frame + 12), uwb7_get16(frame + 14) },
                                       uwb7_get16(frame + 16), { 0 } };
    if (!uwb7_timing_ok(candidate.timing) || frame[2] != (uint8_t)candidate.bundle) {
        return false;
    }
    for (unsigned i = 0; i < UWB3_ANCHORS; ++i) {
        candidate.round[i] = uwb3_get32(frame + UWB3_HEADER + 4 * i);
    }
    *response = candidate;
    return true;
}

/* ------------------------------------------------------------------ 프레임 방식 (태그 여러 대가 차례로, 20 ms 프레임)
 *
 * 7패킷 묶음을 프레임마다 태그 한 대와 한다. 앵커 1 의 Poll 이 프레임의 시작 기준이다.
 *   Poll (30바이트): 9패킷 Poll 과 같은 머리(앵커, 세션, 회차) + 받는 곳 0xFFFF(모두)
 *     [18..21] 프레임 번호  [22] 이번에 위치를 잴 태그 번호  [23] 표시  [24..29] S, D1, D2 (µs)
 *     앵커 1 이 정한 값이 기준이다. 앵커 2, 3 은 자기가 아는 값을 싣고, 태그는 앵커 1 의 것만 따른다.
 *   Response (34바이트): 차례인 태그가 방송
 *     [0..1] 0x41 0x88  [2] 프레임 번호 아래 8비트  [3..4] PAN  [5..6] 0xFFFF  [7..8] 태그 주소 (0x0100 + 태그 번호)
 *     [9] 종류  [10..13] 프레임 번호  [14] 태그 번호  [15] 표시  [16..21] S, D1, D2  [22..33] 받은 Poll 의 회차 3개
 *   Final (30바이트): 9패킷 Final 과 같은 모양, 받는 곳이 태그 주소
 * 태그 번호는 0 부터 UWBF_TAGS - 1 까지다(드론 0번 ~ 5번).
 */
#ifndef UWBF_TAGS
#define UWBF_TAGS 6
#endif

/*
 * 2판 (2026-11-24 밤): Poll 에 제어 20바이트를 더 싣는다(추가 슬롯 일정, 구간 시각, 대본). 종류 값을 0x44 로 바꿔
 * 1판(0x41, 30바이트)과 섞이면 서로 못 알아보게 했다.
 *   Poll (50바이트): [18..29] 는 1판과 같고
 *     [30] 추가 슬롯 수 K (0~5)  [31] post  [32] frag  [33] xchg (10 µs 단위)
 *     [34..36] 추가 슬롯의 주인 5개(4비트씩, 15 = 없음)  [37..39] 교환 상대 5개
 *     [40] 대본 번호  [41] 대본 명령  [42] 움직일 태그(255 = 모두)  [43] 대본 속도 cm/s  [44] 속도 한도 cm/s
 *     [45] 제어가 바뀐 횟수  [46..49] 대본이 시작하는 프레임 번호
 *
 * 한 프레임의 시각 (앵커 1 의 Poll RMARKER 를 0 으로, 모두 RMARKER 시각, µs)
 *   위치 측정: Poll 0, S, 2S / Response 2S+D1 / Final 2S+D1+D2+(k-1)S.  끝 P = 4S+D1+D2
 *   (일정을 드론 0번이 보내는 설정이면) 일정 패킷: P+post, 그리고 아래가 frag 만큼 밀린다
 *   heartbeat: H = P+post.  조각 2개: H, H+frag
 *   추가 슬롯 j (0부터): E = H + 2 frag + j x W.  방송이면 W = 2 frag (조각: E, E+frag),
 *                        교환이면 W = 3 xchg (패킷: E, E+xchg, E+2 xchg)
 *
 * 드론이 보내는 패킷 (모두 [0..1] 0x41 0x88, [3..4] PAN, [7..8] 보낸 태그 주소, [9] 종류, [10..13] 프레임 번호)
 *   방송 조각 (110바이트, 종류 0x45): [5..6] 0xFFFF  [14] 슬롯(0 = heartbeat, 1~5 = 추가 슬롯)  [15] 조각 번호(0, 1)
 *     [16] 보낸 태그  [17..18] 일련번호(방송마다 1 증가, 두 조각이 같음)  [19] 조각 수  [20..109] 빈 데이터 90바이트
 *     -> 방송 하나에 빈 데이터 180바이트
 *   일정 (23바이트, 종류 0x46): [5..6] 0xFFFF  [14] 보낸 태그  [15] K  [16] 표시  [17..19] 주인  [20..22] 상대
 *   교환 (60바이트, 종류 0x48): [5..6] 받는 태그 주소  [14] 슬롯  [15] 단계(0, 1, 2)  [16] 보낸 태그  [17] 받는 태그
 *     [18..19] 일련번호  [20..59] 빈 데이터
 */
enum {
    UWBF_POLL = 0x44, UWBF_RESPONSE = 0x40, UWBF_FINAL = 0x43,
    UWBF_DATA = 0x45, UWBF_SCHED = 0x46, UWBF_XCHG = 0x48,
    UWBF_POLL_V1 = 0x41,          /* 1판의 Poll. 받으면 "다른 방식"으로 센다 */
    UWBF_CTL_LEN = 20,
    UWBF_POLL_LEN = UWB3_HEADER + 12 + UWBF_CTL_LEN,
    UWBF_RESPONSE_LEN = 22 + 4 * UWB3_ANCHORS,
    UWBF_DATA_LEN = 110, UWBF_DATA_HEAD = 20, UWBF_DATA_FRAGS = 2,
    UWBF_SCHED_LEN = 23,
    UWBF_XCHG_LEN = 60, UWBF_XCHG_HEAD = 20,
    UWBF_MAX_LEN = 125,           /* 표준 PHR 의 한도 127 에서 FCS 2 를 뺀 값 */
    UWBF_MASTER_ANCHOR = 1,
    UWBF_FLAG_MASTER = 0x01,      /* 프레임의 기준을 정하는 앵커가 보낸 Poll */
    UWBF_FLAG_COORD_DRONE = 0x02, /* 추가 슬롯 일정을 드론 0번이 따로 방송한다 (Poll 의 주인 칸은 비어 있음) */
    UWBF_FLAG_EXCHANGE = 0x04,    /* 추가 슬롯을 방송(조각 2개) 대신 교환(패킷 3개)으로 쓴다 */
    UWBF_EXTRA_MAX = 5,
    UWBF_NOBODY = 15,
    UWBF_COORD_DRONE_ID = 0,
    UWBF_TAIL_US = 180,           /* 방송 조각의 RMARKER 뒤로 이어지는 시간 (어림, 112바이트) */
    UWBF_TIME_MIN_10US = 30, UWBF_POST_DEFAULT = 100, UWBF_FRAG_DEFAULT = 60, UWBF_XCHG_DEFAULT = 70,
    /* 대본 명령 */
    UWBF_CMD_IDLE = 0, UWBF_CMD_RUN = 1, UWBF_CMD_HOLD = 2, UWBF_CMD_LAND = 3,
    UWBF_MOVER_ALL = 255,
};

struct uwbf_control {
    uint8_t extra;                 /* 프레임당 추가 슬롯 수 */
    uint8_t post, frag, xchg;      /* 10 µs 단위 */
    uint8_t owner[UWBF_EXTRA_MAX], partner[UWBF_EXTRA_MAX];
    uint8_t script, command, mover, speed, limit, seq;
    uint32_t start;
};

struct uwbf_frame {
    uint32_t frame;
    uint8_t turn, flags;
    struct uwb7_timing timing;
    struct uwbf_control ctl;
};

static inline struct uwbf_control uwbf_control_default(void)
{
    struct uwbf_control c = { 0, UWBF_POST_DEFAULT, UWBF_FRAG_DEFAULT, UWBF_XCHG_DEFAULT,
                              { UWBF_NOBODY, UWBF_NOBODY, UWBF_NOBODY, UWBF_NOBODY, UWBF_NOBODY },
                              { UWBF_NOBODY, UWBF_NOBODY, UWBF_NOBODY, UWBF_NOBODY, UWBF_NOBODY },
                              0, UWBF_CMD_IDLE, UWBF_MOVER_ALL, 0, 0, 0, 0 };
    return c;
}

/* 차례대로 돌리는 일정: 추가 슬롯 j 의 주인은 (차례 + 1 + j), 교환 상대는 프레임이 한 바퀴 돌 때마다 바뀐다 */
static inline void uwbf_round_robin(uint32_t frame, struct uwbf_control *c)
{
    const unsigned turn = frame % UWBF_TAGS;
    for (unsigned j = 0; j < UWBF_EXTRA_MAX; ++j) {
        c->owner[j] = c->partner[j] = UWBF_NOBODY;
        if (j < c->extra && j + 1U < UWBF_TAGS) {
            c->owner[j] = (uint8_t)((turn + 1U + j) % UWBF_TAGS);
            const unsigned others = UWBF_TAGS > 1 ? UWBF_TAGS - 1U : 1U;
            c->partner[j] = (uint8_t)((c->owner[j] + 1U + (frame / UWBF_TAGS) % others) % UWBF_TAGS);
        }
    }
}

static inline bool uwbf_control_ok(const struct uwbf_control *c)
{
    if (c->extra > UWBF_EXTRA_MAX || c->post < UWBF_TIME_MIN_10US || c->frag < UWBF_TIME_MIN_10US ||
        c->xchg < UWBF_TIME_MIN_10US || c->command > UWBF_CMD_LAND) {
        return false;
    }
    for (unsigned j = 0; j < UWBF_EXTRA_MAX; ++j) {
        if ((c->owner[j] >= UWBF_TAGS && c->owner[j] != UWBF_NOBODY) ||
            (c->partner[j] >= UWBF_TAGS && c->partner[j] != UWBF_NOBODY)) {
            return false;
        }
    }
    return true;
}

/* 구간 시각 (µs, 앵커 1 의 Poll RMARKER 기준) */
static inline uint32_t uwbf_position_end_us(struct uwb7_timing t)
{
    return 4U * t.slot + t.response_delay + t.final_delay;
}

static inline uint32_t uwbf_sched_us(struct uwb7_timing t, const struct uwbf_control *c)
{
    return uwbf_position_end_us(t) + 10U * c->post;
}

static inline uint32_t uwbf_heartbeat_us(struct uwb7_timing t, const struct uwbf_control *c, uint8_t flags)
{
    return uwbf_sched_us(t, c) + ((flags & UWBF_FLAG_COORD_DRONE) ? 10U * c->frag : 0U);
}

static inline uint32_t uwbf_extra_width_us(const struct uwbf_control *c, uint8_t flags)
{
    return (flags & UWBF_FLAG_EXCHANGE) ? 30U * c->xchg : 20U * c->frag;
}

static inline uint32_t uwbf_extra_us(struct uwb7_timing t, const struct uwbf_control *c, uint8_t flags, unsigned j)
{
    return uwbf_heartbeat_us(t, c, flags) + 20U * c->frag + j * uwbf_extra_width_us(c, flags);
}

/* 프레임에서 무선이 끝나는 때 (마지막 패킷이 끝나는 시각의 어림) */
static inline uint32_t uwbf_radio_end_us(struct uwb7_timing t, const struct uwbf_control *c, uint8_t flags)
{
    return uwbf_extra_us(t, c, flags, c->extra) - 10U * c->frag + UWBF_TAIL_US +
           (c->extra == 0 ? 0U : (flags & UWBF_FLAG_EXCHANGE) ? 10U * c->frag - 10U * c->xchg : 0U);
}

static inline void uwbf_put_nibbles(uint8_t *out, const uint8_t *values)
{
    out[0] = (uint8_t)((values[0] & 15U) | (values[1] << 4));
    out[1] = (uint8_t)((values[2] & 15U) | (values[3] << 4));
    out[2] = (uint8_t)(values[4] & 15U);
}

static inline void uwbf_get_nibbles(const uint8_t *in, uint8_t *values)
{
    values[0] = in[0] & 15U;
    values[1] = in[0] >> 4;
    values[2] = in[1] & 15U;
    values[3] = in[1] >> 4;
    values[4] = in[2] & 15U;
}

static inline void uwbf_encode_control(uint8_t *out, const struct uwbf_control *c)
{
    out[0] = c->extra;
    out[1] = c->post;
    out[2] = c->frag;
    out[3] = c->xchg;
    uwbf_put_nibbles(out + 4, c->owner);
    uwbf_put_nibbles(out + 7, c->partner);
    out[10] = c->script;
    out[11] = c->command;
    out[12] = c->mover;
    out[13] = c->speed;
    out[14] = c->limit;
    out[15] = c->seq;
    uwb3_put32(out + 16, c->start);
}

static inline void uwbf_decode_control(const uint8_t *in, struct uwbf_control *c)
{
    c->extra = in[0];
    c->post = in[1];
    c->frag = in[2];
    c->xchg = in[3];
    uwbf_get_nibbles(in + 4, c->owner);
    uwbf_get_nibbles(in + 7, c->partner);
    c->script = in[10];
    c->command = in[11];
    c->mover = in[12];
    c->speed = in[13];
    c->limit = in[14];
    c->seq = in[15];
    c->start = uwb3_get32(in + 16);
}

struct uwbf_response {
    struct uwbf_frame info;
    uint8_t tag;
    uint32_t round[UWB3_ANCHORS];
};

static inline void uwbf_encode_poll(uint8_t *frame, struct uwb3_key key, const struct uwbf_frame *info)
{
    uwb3_encode(frame, UWB3_POLL, key);
    frame[5] = (uint8_t)UWB7_BROADCAST;
    frame[6] = UWB7_BROADCAST >> 8;
    frame[9] = UWBF_POLL;
    uwb3_put32(frame + UWB3_HEADER, info->frame);
    frame[UWB3_HEADER + 4] = info->turn;
    frame[UWB3_HEADER + 5] = info->flags;
    uwb7_put16(frame + UWB3_HEADER + 6, info->timing.slot);
    uwb7_put16(frame + UWB3_HEADER + 8, info->timing.response_delay);
    uwb7_put16(frame + UWB3_HEADER + 10, info->timing.final_delay);
    uwbf_encode_control(frame + UWB3_HEADER + 12, &info->ctl);
}

/* Length is the received PSDU length, including its two-byte FCS. */
static inline bool uwbf_decode_poll(const uint8_t *frame, uint16_t length, struct uwb3_key *key,
                                    struct uwbf_frame *info)
{
    if (length != UWBF_POLL_LEN + UWB3_FCS || frame[0] != 0x41 || frame[1] != 0x88 ||
        uwb3_get16(frame + 3) != UWB3_PAN || frame[9] != UWBF_POLL || uwb3_get16(frame + 5) != UWB7_BROADCAST) {
        return false;
    }
    struct uwb3_key candidate = { uwb3_get16(frame + 7), uwb3_get32(frame + 10), uwb3_get32(frame + 14) };
    struct uwbf_frame parsed = { uwb3_get32(frame + UWB3_HEADER), frame[UWB3_HEADER + 4], frame[UWB3_HEADER + 5],
                                 { uwb7_get16(frame + UWB3_HEADER + 6), uwb7_get16(frame + UWB3_HEADER + 8),
                                   uwb7_get16(frame + UWB3_HEADER + 10) }, { 0 } };
    uwbf_decode_control(frame + UWB3_HEADER + 12, &parsed.ctl);
    if (candidate.anchor < 1 || candidate.anchor > UWB3_ANCHORS || candidate.session == 0 || candidate.round == 0 ||
        frame[2] != (uint8_t)candidate.round || parsed.turn >= UWBF_TAGS || !uwb7_timing_ok(parsed.timing) ||
        !uwbf_control_ok(&parsed.ctl)) {
        return false;
    }
    *key = candidate;
    *info = parsed;
    return true;
}

static inline void uwbf_encode_response(uint8_t *frame, const struct uwbf_response *response)
{
    const uint16_t source = (uint16_t)(UWB3_TAG + response->tag);
    frame[0] = 0x41;
    frame[1] = 0x88;
    frame[2] = (uint8_t)response->info.frame;
    frame[3] = (uint8_t)UWB3_PAN;
    frame[4] = UWB3_PAN >> 8;
    frame[5] = (uint8_t)UWB7_BROADCAST;
    frame[6] = UWB7_BROADCAST >> 8;
    frame[7] = (uint8_t)source;
    frame[8] = (uint8_t)(source >> 8);
    frame[9] = UWBF_RESPONSE;
    uwb3_put32(frame + 10, response->info.frame);
    frame[14] = response->tag;
    frame[15] = response->info.flags;
    uwb7_put16(frame + 16, response->info.timing.slot);
    uwb7_put16(frame + 18, response->info.timing.response_delay);
    uwb7_put16(frame + 20, response->info.timing.final_delay);
    for (unsigned i = 0; i < UWB3_ANCHORS; ++i) {
        uwb3_put32(frame + 22 + 4 * i, response->round[i]);
    }
}

static inline bool uwbf_decode_response(const uint8_t *frame, uint16_t length, struct uwbf_response *response)
{
    if (length != UWBF_RESPONSE_LEN + UWB3_FCS || frame[0] != 0x41 || frame[1] != 0x88 ||
        uwb3_get16(frame + 3) != UWB3_PAN || frame[9] != UWBF_RESPONSE ||
        uwb3_get16(frame + 5) != UWB7_BROADCAST) {
        return false;
    }
    struct uwbf_response parsed = { { uwb3_get32(frame + 10), frame[14], frame[15],
                                      { uwb7_get16(frame + 16), uwb7_get16(frame + 18), uwb7_get16(frame + 20) },
                                      { 0 } },
                                    frame[14], { 0 } };
    if (parsed.tag >= UWBF_TAGS || uwb3_get16(frame + 7) != UWB3_TAG + parsed.tag ||
        frame[2] != (uint8_t)parsed.info.frame || !uwb7_timing_ok(parsed.info.timing)) {
        return false;
    }
    for (unsigned i = 0; i < UWB3_ANCHORS; ++i) {
        parsed.round[i] = uwb3_get32(frame + 22 + 4 * i);
    }
    *response = parsed;
    return true;
}

/* Final: 시각 3개(12바이트)는 호출한 쪽이 채운다. tag 는 받는 태그의 번호. */
static inline void uwbf_encode_final(uint8_t *frame, struct uwb3_key key, uint8_t tag)
{
    const uint16_t destination = (uint16_t)(UWB3_TAG + tag);
    uwb3_encode(frame, UWB3_POLL, key);
    frame[5] = (uint8_t)destination;
    frame[6] = (uint8_t)(destination >> 8);
    frame[9] = UWBF_FINAL;
}

static inline bool uwbf_decode_final(const uint8_t *frame, uint16_t length, struct uwb3_key *key, uint8_t *tag)
{
    const uint16_t destination = uwb3_get16(frame + 5);
    if (length != UWB3_FINAL_LEN + UWB3_FCS || frame[0] != 0x41 || frame[1] != 0x88 ||
        uwb3_get16(frame + 3) != UWB3_PAN || frame[9] != UWBF_FINAL ||
        destination < UWB3_TAG || destination >= UWB3_TAG + UWBF_TAGS) {
        return false;
    }
    struct uwb3_key candidate = { uwb3_get16(frame + 7), uwb3_get32(frame + 10), uwb3_get32(frame + 14) };
    if (candidate.anchor < 1 || candidate.anchor > UWB3_ANCHORS ||
        candidate.session == 0 || candidate.round == 0 || frame[2] != (uint8_t)candidate.round) {
        return false;
    }
    *key = candidate;
    *tag = (uint8_t)(destination - UWB3_TAG);
    return true;
}

/* ---- 드론이 보내는 패킷 ---- */
struct uwbf_packet {
    uint32_t frame;
    uint16_t serial;
    uint8_t type, sender, peer, slot, part, parts;   /* part: 방송의 조각 번호 또는 교환의 단계 */
};

static inline void uwbf_drone_header(uint8_t *out, uint8_t type, uint32_t frame, uint8_t sender, uint16_t destination)
{
    const uint16_t source = (uint16_t)(UWB3_TAG + sender);
    out[0] = 0x41;
    out[1] = 0x88;
    out[2] = (uint8_t)frame;
    out[3] = (uint8_t)UWB3_PAN;
    out[4] = UWB3_PAN >> 8;
    out[5] = (uint8_t)destination;
    out[6] = (uint8_t)(destination >> 8);
    out[7] = (uint8_t)source;
    out[8] = (uint8_t)(source >> 8);
    out[9] = type;
    uwb3_put32(out + 10, frame);
}

/* out 은 UWBF_DATA_LEN 바이트. 빈 데이터는 보낸 태그와 일련번호로 만든 무늬 */
static inline void uwbf_encode_data(uint8_t *out, const struct uwbf_packet *p)
{
    uwbf_drone_header(out, UWBF_DATA, p->frame, p->sender, UWB7_BROADCAST);
    out[14] = p->slot;
    out[15] = p->part;
    out[16] = p->sender;
    out[17] = (uint8_t)p->serial;
    out[18] = (uint8_t)(p->serial >> 8);
    out[19] = UWBF_DATA_FRAGS;
    for (unsigned i = UWBF_DATA_HEAD; i < UWBF_DATA_LEN; ++i) {
        out[i] = (uint8_t)(i * 7U + p->serial + p->sender);
    }
}

static inline void uwbf_encode_xchg(uint8_t *out, const struct uwbf_packet *p)
{
    uwbf_drone_header(out, UWBF_XCHG, p->frame, p->sender, (uint16_t)(UWB3_TAG + p->peer));
    out[14] = p->slot;
    out[15] = p->part;
    out[16] = p->sender;
    out[17] = p->peer;
    out[18] = (uint8_t)p->serial;
    out[19] = (uint8_t)(p->serial >> 8);
    for (unsigned i = UWBF_XCHG_HEAD; i < UWBF_XCHG_LEN; ++i) {
        out[i] = (uint8_t)(i * 5U + p->serial + p->sender);
    }
}

static inline void uwbf_encode_sched(uint8_t *out, uint32_t frame, uint8_t sender, uint8_t flags,
                                     const struct uwbf_control *c)
{
    uwbf_drone_header(out, UWBF_SCHED, frame, sender, UWB7_BROADCAST);
    out[14] = sender;
    out[15] = c->extra;
    out[16] = flags;
    uwbf_put_nibbles(out + 17, c->owner);
    uwbf_put_nibbles(out + 20, c->partner);
}

/* 머리(앞 23바이트 이상)만 읽어도 풀 수 있다. length 는 받은 PSDU 길이(FCS 포함), have 는 head 에 실제로 읽은 바이트 수.
 * 일정 패킷이면 owner/partner 를 채운다(c 가 NULL 이 아닐 때). */
static inline bool uwbf_decode_drone(const uint8_t *head, uint16_t length, uint16_t have, struct uwbf_packet *p,
                                     struct uwbf_control *c)
{
    if (have < UWBF_SCHED_LEN || head[0] != 0x41 || head[1] != 0x88 || uwb3_get16(head + 3) != UWB3_PAN) {
        return false;
    }
    const uint16_t source = uwb3_get16(head + 7), destination = uwb3_get16(head + 5);
    struct uwbf_packet q = { uwb3_get32(head + 10), 0, head[9], (uint8_t)(source - UWB3_TAG), UWBF_NOBODY, 0, 0, 1 };
    if (source < UWB3_TAG || source >= UWB3_TAG + UWBF_TAGS || head[2] != (uint8_t)q.frame) {
        return false;
    }
    if (q.type == UWBF_DATA) {
        if (length != UWBF_DATA_LEN + UWB3_FCS || destination != UWB7_BROADCAST || head[16] != q.sender ||
            head[14] > UWBF_EXTRA_MAX || head[15] >= UWBF_DATA_FRAGS) {
            return false;
        }
        q.slot = head[14];
        q.part = head[15];
        q.serial = uwb3_get16(head + 17);
        q.parts = head[19];
    } else if (q.type == UWBF_XCHG) {
        if (length != UWBF_XCHG_LEN + UWB3_FCS || head[16] != q.sender || head[17] >= UWBF_TAGS ||
            destination != UWB3_TAG + head[17] || head[14] < 1 || head[14] > UWBF_EXTRA_MAX || head[15] > 2) {
            return false;
        }
        q.slot = head[14];
        q.part = head[15];
        q.peer = head[17];
        q.serial = uwb3_get16(head + 18);
        q.parts = 3;
    } else if (q.type == UWBF_SCHED) {
        if (length != UWBF_SCHED_LEN + UWB3_FCS || destination != UWB7_BROADCAST || head[14] != q.sender ||
            head[15] > UWBF_EXTRA_MAX) {
            return false;
        }
        if (c != NULL) {
            c->extra = head[15];
            uwbf_get_nibbles(head + 17, c->owner);
            uwbf_get_nibbles(head + 20, c->partner);
            for (unsigned j = 0; j < UWBF_EXTRA_MAX; ++j) {
                if ((c->owner[j] >= UWBF_TAGS && c->owner[j] != UWBF_NOBODY) ||
                    (c->partner[j] >= UWBF_TAGS && c->partner[j] != UWBF_NOBODY)) {
                    return false;
                }
            }
        }
    } else {
        return false;
    }
    *p = q;
    return true;
}

#endif
