/*
 * UWB 프로토콜 계층 (CBF-Link 논문 Sec. 2.2, 4.2, 부록 B를 우리 하드웨어용으로 단순화).
 * 순수 C99. HAL, DW3000 드라이버, malloc 없음. PC(gcc)와 N6 양쪽에서 빌드한다.
 *
 * 들어 있는 것
 *   - 패킷: heartbeat, 상태 레코드, 조정자 스케줄 (리틀 엔디언 직렬화)
 *   - TDMA: 20 ms 프레임 안의 슬롯 배치 (비콘, heartbeat, 앵커 측위, 피어 교환)
 *   - 조정자 배정: 긴급 -> 점수 -> 오래된 순, 프레임당 기체 1회, K개까지
 *   - 거리 일관성 검사, 비콘 기반 시계 오프셋 (DW3000 40비트 타임스탬프)
 * 없는 것: DW3000 무선 구동 (Qorvo 드라이버 포팅 U0 뒤에 n6 쪽에서 붙인다)
 */
#ifndef UWB_PROTOCOL_H
#define UWB_PROTOCOL_H

#include <stdint.h>

#define UWB_MAX_DRONES 8
#define UWB_MAX_ACKS 8
#define UWB_MAX_BIDS 3   /* 논문 M_bid = 3 */
#define UWB_MAX_K 4
#define UWB_MAX_PAYLOAD 116 /* 127 B 표준 프레임 - MAC 헤더 9 B - FCS 2 B */

enum { UWB_PKT_HEARTBEAT = 1, UWB_PKT_STATE = 2, UWB_PKT_SCHEDULE = 3 };

/* 상태 레코드: 위치 mm, 속도 mm/s, 측정 시각 us (네트워크 시간), 위치 오차 상한 mm */
typedef struct {
    uint8_t id;
    uint16_t seq;
    uint32_t t_src_us;
    int16_t pos_mm[2];
    int16_t vel_mms[2];
    uint16_t eps_mm;
} uwb_state_t;

typedef struct { uint8_t id; uint8_t version; } uwb_ack_t;

/* 교환 입찰: 상대 기체, 점수(쌍대 변수 기반, 단위 1e-3), 유효 프레임 */
typedef struct { uint8_t peer; int16_t score; uint16_t valid_until_frame; uint8_t urgent; } uwb_bid_t;

typedef struct {
    uwb_state_t state;
    uint8_t n_ack;
    uwb_ack_t ack[UWB_MAX_ACKS];
    uint8_t n_bid;
    uwb_bid_t bid[UWB_MAX_BIDS];
} uwb_heartbeat_t;

/* 조정자 스케줄: 다음 프레임의 교환 쌍. initiator = a */
typedef struct { uint8_t a, b; } uwb_pair_t;
typedef struct {
    uint16_t epoch;
    uint16_t frame;
    uint8_t n_pair;
    uwb_pair_t pair[UWB_MAX_K];
} uwb_schedule_t;

/* 직렬화: 쓴 바이트 수, 공간 부족이나 잘못된 값이면 -1 */
int uwb_encode_heartbeat(const uwb_heartbeat_t *hb, uint8_t *buf, int cap);
int uwb_decode_heartbeat(const uint8_t *buf, int len, uwb_heartbeat_t *hb);
int uwb_encode_state(const uwb_state_t *s, uint8_t *buf, int cap);
int uwb_decode_state(const uint8_t *buf, int len, uwb_state_t *s);
int uwb_encode_schedule(const uwb_schedule_t *s, uint8_t *buf, int cap);
int uwb_decode_schedule(const uint8_t *buf, int len, uwb_schedule_t *s);

/* ---- TDMA ---- */
typedef struct {
    uint32_t frame_us;     /* 20000 */
    uint32_t beacon_us;    /* 조정자 비콘 슬롯 */
    uint32_t hb_slot_us;   /* heartbeat 슬롯 하나 */
    uint32_t loc_slot_us;  /* 앵커 DS-TWR 슬롯 하나 */
    uint32_t ex_slot_us;   /* 피어 DS-TWR 슬롯 하나 */
    uint8_t n_drones;
    uint8_t n_anchors;
    uint8_t hb_every;      /* 기체마다 hb_every 프레임에 한 번 heartbeat (T_h = hb_every * frame) */
    uint8_t loc_per_frame; /* 프레임당 앵커 측위 슬롯 수 */
    uint8_t k;             /* 프레임당 피어 교환 슬롯 수 */
} uwb_tdma_cfg_t;

enum { UWB_SLOT_BEACON, UWB_SLOT_HB, UWB_SLOT_LOC, UWB_SLOT_EX, UWB_SLOT_IDLE };

typedef struct {
    uint8_t type;
    uint8_t drone;   /* HB, LOC: 송신 기체. EX: 교환 인덱스 */
    uint8_t anchor;  /* LOC 전용 */
    uint32_t start_us; /* 프레임 시작 기준 */
} uwb_slot_t;

/* 설정이 한 프레임에 들어가면 1, 넘으면 0 */
int uwb_tdma_fits(const uwb_tdma_cfg_t *c);
/* 프레임 f의 프레임 내 시각 t_us에 해당하는 슬롯 */
uwb_slot_t uwb_tdma_slot_at(const uwb_tdma_cfg_t *c, uint32_t frame, uint32_t t_us);

/* ---- 조정자 배정 ----
 * age_ms[i][j]: i가 가진 j 레코드의 나이. 남는 슬롯은 가장 오래된 쌍으로 채운다.
 * bids_of[i]: 기체 i가 보낸 입찰 (n_bids_of[i]개). 반환: 배정한 쌍 수 */
int uwb_schedule_pick(uint8_t n, uint8_t k, uint16_t frame,
                      const uwb_bid_t bids_of[][UWB_MAX_BIDS], const uint8_t n_bids_of[],
                      const uint16_t age_ms[][UWB_MAX_DRONES], uwb_pair_t out[]);

/* ---- 검증, 시계 ---- */
/* |r - |q_i - q_j|| <= eps_i + eps_j + sigma_r 이면 1 (수락) */
int uwb_range_consistent(int32_t range_mm, const uwb_state_t *a, const uwb_state_t *b, uint16_t sigma_r_mm);

/* DW3000 40비트 타임스탬프 차이 (b - a, 랩어라운드 처리). 단위 DTU (~15.65 ps) */
int64_t uwb_ts_diff(uint64_t a, uint64_t b);
/* 비콘 송신 시각(조정자 시계)과 수신 시각(내 시계)으로 오프셋 (조정자 - 나, DTU) */
int64_t uwb_clock_offset(uint64_t beacon_tx_coord, uint64_t beacon_rx_local);

#endif
