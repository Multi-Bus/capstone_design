/* 태그(응답기), 프레임 방식: 태그 여러 대가 20 ms 프레임마다 한 대씩 차례로 위치를 잰다. NUCLEO-N657 + DWM3000EVB.
 *
 * 9패킷은 tag.c (v2), 태그 한 대와 하는 7패킷은 tag7.c 에 그대로 있다. prepare_tag.py --packets f --tag-id N 으로 고른다.
 * 앵커 쪽 짝은 src/main_f.c, 패킷 형식은 frames.h 의 "프레임 방식" 부분.
 *
 * 규칙
 *   - 앵커 1 의 Poll 이 프레임의 시작 기준이다. 거기 실린 프레임 번호, 차례인 태그 번호, S, D1, D2 를 따른다.
 *   - 차례가 내 번호(TAGF_ID)일 때만 답한다: Response = 앵커 1 의 Poll 을 받은 시각 + 2S + D1.
 *     앵커 2, 3 의 Poll 을 못 받아도 같은 시각에 답한다. 앵커 1 의 Poll 을 못 받은 프레임에는 아무것도 보내지 않는다.
 *   - 차례가 아닌 프레임에도 계속 듣고, 무엇을 들었는지 프레임마다 한 줄로 남긴다(링크별 손실을 세는 자료).
 *   - 거리는 하나마다 프레임 번호와 칩 시각을 붙여 따로 남긴다. 세 거리를 한 순간의 값으로 합치지 않는다.
 *
 * 기록
 *   - 무선을 다루는 동안에는 글자를 만들지도 내보내지도 않는다. 결과를 작은 구조체로 메모리에 쌓아 두고,
 *     다음 패킷을 기다리는 동안 한 글자씩 UART 로 내보낸다(기다리지 않는 방식). 쌓을 자리가 없으면 버리고 센다.
 *   - TAGF_SD 로 빌드하면 같은 줄을 SD 카드의 UWBLOG.BIN 에도 쓴다(sdlog.c). 카드에 쓰기를 거는 일(약 0.3 ms)은
 *     프레임의 조용한 구간(앵커 1 의 Poll 뒤 12~18 ms)에서만 한다. SD 가 켜져 있으면 SD 가 주 기록이고,
 *     UART 는 따라올 수 있는 만큼만 내보낸다.
 *
 * 2판 (tagf-v2): 위치 측정 뒤에 드론끼리의 방송이 붙는다. 시각은 frames.h 의 "한 프레임의 시각" 참조.
 *   - heartbeat: 차례인 태그가 빈 데이터 180바이트를 조각 2개로 방송한다.
 *   - 추가 슬롯: 일정에 적힌 태그가 같은 방송을 하거나(기본), 상대 태그와 패킷 3개를 주고받는다(교환).
 *   - 일정은 앵커 1 의 Poll 에 실려 온다. "드론 0번이 일정을 보내는 설정"이면 0번이 위치 측정 뒤에 일정 패킷을
 *     따로 방송하고, 다른 태그는 그것을 받아야 그 프레임에 방송할 수 있다.
 *   - 앵커 1 의 Poll 을 못 받은 프레임에는 아무것도 보내지 않는다. 듣고 기록하는 일은 계속한다.
 *   - 기록 줄을 글자로 만드는 일과 SD 쓰기는 프레임의 조용한 구간(무선이 끝난 뒤)에만 한다.
 *   - UART 는 921600 bit/s 로 바꾼다(줄이 많아져서).
 *
 * 줄 형식
 *   TF_READY,... / TF_INFO,fw=tagf-v2,id=<태그 번호>,tags=<태그 수>,...
 *   F,<프레임>,<기준 시각>,<차례>,<들은 것>,<Response>,<Response 여유 µs>,<되돌림 µs>,<수신오류>,
 *     <일정>,<추가 슬롯 수>,<표시>,<무선 끝 µs>,<조용한 구간에 쓴 µs>,<그중 가장 긴 한 번 µs>,<기록 버림 누계>,
 *     <제어 계산 시작 µs>,<제어 계산 끝 µs>,<두 조각을 다 받은 방송 수>
 *      - 기준 시각: 앵커 1 의 Poll 을 받은 DW 시각 상위 32비트(hex). 못 받았으면 0
 *      - 들은 것(hex): 비트 0~2 앵커 1~3 의 Poll, 비트 3 Response, 비트 4~6 앵커 1~3 의 Final (남의 것도 포함)
 *      - Response: 내 차례가 아니면 -, 내 차례면 R 보냄, L 예약이 늦음, N 송신 끝 표시를 못 봄
 *      - 일정: A 앵커 1 의 Poll 에서 받음, D 드론 0번의 일정 패킷에서 받음(0번 자신은 스스로 만듦),
 *              N 드론 0번이 보내는 설정인데 못 받음, - 앵커 1 의 Poll 을 못 받음
 *      - 표시(hex): 0x02 일정을 드론 0번이 보냄, 0x04 추가 슬롯이 교환
 *      - 조용한 구간에 쓴 µs: 바로 앞 프레임에서 기록 만들기, SD, 비행 제어기 통신에 쓴 시간
 *      - 제어 계산: 프레임마다 한 번, 무선이 끝난 직후에 한다. 그 프레임에 받은 방송을 읽고(지금은 세기만 한다),
 *        비행용 빌드에서는 대본의 속도 명령을 정해 보낸다. 시작·끝은 앵커 1 의 Poll 기준 µs, 안 했으면 0.
 *        "받은 패킷이 쓰이기까지의 시간" = 제어 계산 시작 - 그 패킷을 받은 시각(P 줄의 예약 µs + 어긋남)
 *   C, E, U 줄: 비행용 빌드에서만. 비행 제어기의 값과 보낸 속도 명령(C), 대본과 안전의 사건(E), UART 통계(U).
 *      형식은 fcl.h, fcl.c, flight.c 의 머리말
 *   P,<프레임>,<종류>,<방향>,<슬롯>,<조각/단계>,<보낸 태그>,<받는 태그>,<일련번호>,<바이트>,<성공>,<예약 µs>,<어긋남 ns>,
 *     <칩 시각>,<RSSI>,<첫경로>,<시계 차이>
 *      - 종류: H heartbeat, B 추가 슬롯 방송, X 교환, C 일정.  방향: T 내가 보냄, R 내가 받음
 *      - 슬롯: 0 heartbeat, 1~5 추가 슬롯.  받는 태그: 방송이면 15
 *      - 성공: 보낸 것은 1 송신함 / 0 예약이 늦어 못 보냄, 받은 것은 늘 1 (못 받은 것은 보낸 쪽 줄과 견줘서 센다)
 *      - 예약 µs: 앵커 1 의 Poll 기준으로 정해진 시각.  어긋남 ns: 실제 시각 - 예약 시각. 기준이 없으면 2147483647
 *      - 칩 시각: 보낸/받은 DW 시각 상위 32비트(hex).  시계 차이: dwt_readclockoffset() 의 값(받은 것만)
 *      - RSSI, 첫경로: 드라이버가 주는 값 그대로(R 줄과 같음). 보낸 것과 못 읽은 것은 32767
 *   K,<프레임>,<제어 번호>,<표시>,<추가 슬롯 수>,<post>,<frag>,<xchg>,<대본>,<명령>,<시작 프레임>,<움직일 태그>,<속도>,<한도>,
 *     <S>,<D1>,<D2>                                    (앵커 1 이 보낸 제어가 바뀔 때마다. 비행의 설정 기록)
 *   L,<프레임>,<1 켬 / 0 끔>                           (영상과 맞추는 LED)
 *   R,<프레임>,<앵커>,<mm>,<거리 유효>,<끝 종류>,<Final RSSI>,<Final 첫경로>,<Poll 시각>,<Poll→Final ns>
 *      - 내 차례에 받은 Poll 마다 한 줄. 끝 종류: F Final 받음, T 안 옴, E 수신 오류, L Response 를 못 보냄
 *      - Poll 시각: 그 앵커의 Poll 을 받은 DW 시각 상위 32비트(hex)
 *   S,<프레임>,<기록 버림>,<수신오류 누계>,<기타 패킷 누계>,<다른 방식 패킷 누계>      (250 프레임마다)
 *   D,<프레임>,<SD 상태>,<다음에 쓸 섹터>,<쓴 섹터>,<버린 글자>,<쓰기 오류>,<가장 긴 쓰기 µs>,<쓰기를 거는 데 걸린 최대 µs>,
 *     <쌓인 섹터>,<UART 로 못 보낸 줄>                                                 (TAGF_SD 일 때, 250 프레임마다)
 *      - SD 상태: 0 꺼짐, 1 준비, 2 쓰는 중, 3 카드 기다림, 4 복구 중, 5 파일 꽉 참, 6 실패
 */
#include <stdio.h>
#include <string.h>
#include "tagf_config.h" /* prepare_tag.py 가 쓴다: TAGF_ID, UWBF_TAGS. frames.h 보다 먼저 와야 한다 */
#include <config_options.h>
#include <deca_spi.h>
#include <port.h>
#include "deca_probe_interface.h"
#include "radio.h"
#include "tag_antenna_delay.h"

#define TAG_FW "tagf-v2-lfs"   /* v2 + n6_bridge link_failsafe.h */
#ifndef TAGF_ID
#error TAGF_ID (0 .. UWBF_TAGS - 1) 를 정해야 합니다.
#endif
#ifndef TAGF_BUILD_OPT
#define TAGF_BUILD_OPT "?"
#endif
#if defined(TAGF_SIM) || !defined(TAGF_SD)
#undef TAGF_SD
#define TAGF_SD 0          /* PC 모의시험에는 카드가 없다 */
#endif
#ifndef TAGF_FRAME_US
#define TAGF_FRAME_US 20000 /* SD 쓰기를 걸 구간을 정할 때만 쓴다. 앵커의 프레임 길이와 같게 */
#endif
#if TAGF_SD
#include "sdfat.h"
#include "sdlog.h"
#endif
#if defined(TAGF_SIM) || !defined(TAGF_FLIGHT)
#undef TAGF_FLIGHT
#define TAGF_FLIGHT 0      /* 비행 제어기 연결(flight.c)은 비행용 빌드에만 */
#endif
#if TAGF_FLIGHT
#include "flight.h"
#endif
#ifndef TAGF_EARLY_REARM
#define TAGF_EARLY_REARM 1 /* 방송 조각을 받으면 신호 세기를 읽기 전에 수신부터 다시 켠다 */
#endif
_Static_assert(TAGF_ID >= 0 && TAGF_ID < UWBF_TAGS, "TAGF_ID 는 0 부터 UWBF_TAGS - 1 까지");

extern dwt_txconfig_t txconfig_options;
extern dwt_config_t config_options;

enum {
    TX_LEAD_MAX_US = 300, /* Response 송신 시작 이만큼 전에 Poll 듣기를 멈추고 예약한다 (tag7.c 와 같은 규칙) */
    TX_LEAD_MIN_US = 130,
    TX_LEAD_BACK_US = 260,
    END_MARGIN_US = 350,  /* 마지막 Final 의 RMARKER 예정 시각 뒤 이만큼 더 기다린다 */
    RX_EARLY_US = 260,    /* Response 송신이 끝난 뒤 수신을 켜는 때: Final 프리앰블보다 약 60 µs 먼저 */
    STATS_EVERY = 250,    /* 이만큼의 프레임마다 S 줄 */
    RECORDS = 512,        /* 쌓아 두는 기록 수 (2의 거듭제곱) */
    QUIET_AFTER_US = 150, /* 무선이 끝난 뒤 이만큼 지나면 조용한 구간 */
    QUIET_END_US = 1500,  /* 다음 프레임 이만큼 전에는 조용한 구간의 일을 새로 시작하지 않는다 */
    SEEN_RESPONSE = 0x08, SEEN_FINAL = 0x10,
    ACT_LEAD_US = 150,    /* 내 방송: 프리앰블이 시작되기 이만큼 전에 듣기를 멈추고 예약한다 (데이터는 미리 써 둔다) */
    ACT_FINE_US = 1500,   /* 내 방송이 이만큼 남으면 MCU 타이머 대신 DW 시계를 읽어 판단 */
    TX_DONE_US = 400,     /* 송신 끝 표시를 기다리는 한도: RMARKER 뒤 */
    XCHG_WAIT_US = 350,   /* 교환: 상대 패킷의 RMARKER 예정 시각 뒤 이만큼까지 기다린다 */
    HEAD_BYTES = UWBF_POLL_LEN + UWB3_FCS, /* 받은 패킷에서 읽는 앞부분. 방송 조각은 빈 데이터를 읽지 않는다 */
    ACTIONS = UWBF_EXTRA_MAX + 2,
    LED_PERIOD = 250, LED_ON = 5, LED_LONG_PERIOD = 3000, LED_LONG_ON = 50, /* 프레임 수 */
    DT_UNKNOWN = 0x7FFFFFFF,
};
#define DTU_40BIT_MASK 0xFFFFFFFFFFULL

/* ------------------------------------------------------------------ UART: 기다리지 않고 한 글자씩 */
#ifdef TAGF_SIM
bool tagf_uart_ready(void);
void tagf_uart_put(uint8_t value);
uint32_t tagf_micros(void);          /* 모의시험기의 가상 시간 */
static inline void tagf_led(bool on) { (void)on; }
static inline void tagf_board_init(void) { }
#else
extern UART_HandleTypeDef huart1;
static inline bool tagf_uart_ready(void) { return (huart1.Instance->ISR & USART_ISR_TXE) != 0U; }
static inline void tagf_uart_put(uint8_t value) { huart1.Instance->TDR = value; }

/* MCU 의 마이크로초 시계: CPU 사이클 카운터를 쌓는다. 7초 안에 한 번은 불러야 한다(주 반복문이 늘 부른다) */
static uint32_t tagf_micros(void)
{
    static uint32_t last, rest, micros;
    const uint32_t per_us = SystemCoreClock / 1000000U;
    const uint32_t now = DWT->CYCCNT, delta = now - last + rest;
    last = now;
    micros += delta / per_us;
    rest = delta % per_us;
    return micros;
}

static inline void tagf_led(bool on)
{
    if (on) {
        BSP_LED_On(LED_BLUE);
    } else {
        BSP_LED_Off(LED_BLUE);
    }
}

static void tagf_board_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    HAL_Delay(20);                       /* 앞의 115200 출력이 다 나가게 */
    huart1.Init.BaudRate = 921600;
    (void)HAL_UART_Init(&huart1);
    BSP_LED_Off(LED_BLUE);
}
#endif

/* ------------------------------------------------------------------ 기록 */
enum record_kind { REC_FRAME, REC_RANGE, REC_STATS, REC_SD, REC_PACKET, REC_CONTROL, REC_LED };

struct record {
    uint8_t kind;
    uint32_t frame;
    union {
        struct { uint32_t t0_hi; int16_t lead_us; uint16_t turn_us, rx_err, radio_end_us, work_us, work_max_us, dropped,
                 calc_start_us, calc_end_us; uint8_t turn, seen, extra, flags, heard; char response, sched; } f;
        struct { uint32_t t_hi; int32_t dt_ns; uint16_t serial, sched_us; int16_t rssi, first_path, cko;
                 uint8_t slot, part, src, dst, bytes, ok; char type, dir; } p;
        struct { struct uwbf_control c; struct uwb7_timing t; uint8_t flags; } k;
        struct { uint8_t on; } l;
        struct { uint32_t poll_hi, end_ns; int32_t mm; int16_t rssi, first_path; uint8_t anchor, valid; char end; } r;
        struct { uint32_t dropped, rx_errors, rx_other, legacy; } s;
        struct { uint32_t next_block, written, dropped, errors, max_write_us, max_start_us, queued, uart_skipped;
                 uint8_t state; } d;
    } u;
};

static struct record records[RECORDS];
static uint32_t rec_head, rec_tail;   /* head 에 넣고 tail 에서 꺼낸다 */
enum { LINE_MAX = 160, UART_RING = 4096 };   /* UART 로 나갈 글자를 쌓아 두는 고리 (2의 거듭제곱) */
static char uart_ring[UART_RING];
static uint32_t uart_head, uart_tail;
static uint32_t log_dropped, rx_errors, rx_other, legacy_frames;

static void push_record(const struct record *r)
{
    if (rec_head - rec_tail >= RECORDS) {
        ++log_dropped;
        return;
    }
    records[rec_head++ % RECORDS] = *r;
}

#if TAGF_SD
static uint32_t uart_skipped;
static bool sd_active;
#endif

static unsigned format_record(const struct record *r, char *out, unsigned size)
{
    int n = 0;
    if (r->kind == REC_FRAME) {
        n = snprintf(out, size, "F,%lu,%08lX,%u,%02X,%c,%d,%u,%u,%c,%u,%02X,%u,%u,%u,%u,%u,%u,%u\r\n", (unsigned long)r->frame,
                     (unsigned long)r->u.f.t0_hi, (unsigned)r->u.f.turn, (unsigned)r->u.f.seen, r->u.f.response,
                     (int)r->u.f.lead_us, (unsigned)r->u.f.turn_us, (unsigned)r->u.f.rx_err, r->u.f.sched,
                     (unsigned)r->u.f.extra, (unsigned)r->u.f.flags, (unsigned)r->u.f.radio_end_us,
                     (unsigned)r->u.f.work_us, (unsigned)r->u.f.work_max_us, (unsigned)r->u.f.dropped,
                     (unsigned)r->u.f.calc_start_us, (unsigned)r->u.f.calc_end_us, (unsigned)r->u.f.heard);
    } else if (r->kind == REC_PACKET) {
        n = snprintf(out, size, "P,%lu,%c,%c,%u,%u,%u,%u,%u,%u,%u,%u,%ld,%08lX,%d,%d,%d\r\n", (unsigned long)r->frame,
                     r->u.p.type, r->u.p.dir, (unsigned)r->u.p.slot, (unsigned)r->u.p.part, (unsigned)r->u.p.src,
                     (unsigned)r->u.p.dst, (unsigned)r->u.p.serial, (unsigned)r->u.p.bytes, (unsigned)r->u.p.ok,
                     (unsigned)r->u.p.sched_us, (long)r->u.p.dt_ns, (unsigned long)r->u.p.t_hi, (int)r->u.p.rssi,
                     (int)r->u.p.first_path, (int)r->u.p.cko);
    } else if (r->kind == REC_CONTROL) {
        const struct uwbf_control *c = &r->u.k.c;
        n = snprintf(out, size, "K,%lu,%u,%02X,%u,%u,%u,%u,%u,%u,%lu,%u,%u,%u,%u,%u,%u\r\n", (unsigned long)r->frame,
                     (unsigned)c->seq, (unsigned)r->u.k.flags, (unsigned)c->extra, (unsigned)c->post, (unsigned)c->frag,
                     (unsigned)c->xchg, (unsigned)c->script, (unsigned)c->command, (unsigned long)c->start,
                     (unsigned)c->mover, (unsigned)c->speed, (unsigned)c->limit, (unsigned)r->u.k.t.slot,
                     (unsigned)r->u.k.t.response_delay, (unsigned)r->u.k.t.final_delay);
    } else if (r->kind == REC_LED) {
        n = snprintf(out, size, "L,%lu,%u\r\n", (unsigned long)r->frame, (unsigned)r->u.l.on);
    } else if (r->kind == REC_RANGE) {
        n = snprintf(out, size, "R,%lu,%u,%ld,%u,%c,%d,%d,%08lX,%lu\r\n", (unsigned long)r->frame,
                     (unsigned)r->u.r.anchor, (long)r->u.r.mm, (unsigned)r->u.r.valid, r->u.r.end, (int)r->u.r.rssi,
                     (int)r->u.r.first_path, (unsigned long)r->u.r.poll_hi, (unsigned long)r->u.r.end_ns);
    } else if (r->kind == REC_STATS) {
        n = snprintf(out, size, "S,%lu,%lu,%lu,%lu,%lu\r\n", (unsigned long)r->frame,
                     (unsigned long)r->u.s.dropped, (unsigned long)r->u.s.rx_errors, (unsigned long)r->u.s.rx_other,
                     (unsigned long)r->u.s.legacy);
    } else {
        n = snprintf(out, size, "D,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n", (unsigned long)r->frame,
                     (unsigned)r->u.d.state, (unsigned long)r->u.d.next_block, (unsigned long)r->u.d.written,
                     (unsigned long)r->u.d.dropped, (unsigned long)r->u.d.errors, (unsigned long)r->u.d.max_write_us,
                     (unsigned long)r->u.d.max_start_us, (unsigned long)r->u.d.queued,
                     (unsigned long)r->u.d.uart_skipped);
    }
    return (unsigned)(n < 0 ? 0 : n >= (int)size ? (int)size - 1 : n);
}

/* ------------------------------------------------------------------ 이번 프레임 */
struct action {
    uint32_t us;          /* 첫 패킷의 RMARKER (앵커 1 의 Poll 기준 µs) */
    uint8_t slot, peer;
    char type;            /* C 일정, H heartbeat, B 추가 슬롯 방송, X 교환(내가 시작) */
};

static struct {
    bool have_t0;         /* 이 프레임에 앵커 1 의 Poll 을 받았다 */
    bool have_sched;      /* 추가 슬롯의 주인을 안다 */
    bool locked;          /* 최근에 앵커 1 의 Poll 을 받아 프레임의 박자를 안다 */
    uint32_t frame;
    uint64_t t0;          /* 앵커 1 의 Poll 을 받은 DW 시각 (40비트) */
    uint32_t t0_micros;   /* 같은 때의 MCU 시계 */
    struct uwb7_timing timing;
    struct uwbf_control ctl;
    uint8_t flags, turn;
    uint32_t radio_end_us;
    struct action act[ACTIONS];
    uint8_t acts, next;   /* next: 다음에 할 것 */
    bool prepared;        /* 다음 것의 데이터를 칩에 써 두었다 */
    bool control_done;    /* 이 프레임의 제어 계산을 했다 */
    uint8_t buffer[UWBF_DATA_LEN];
} fs;

static uint16_t serial_next;
static uint32_t work_us, work_max_us;   /* 조용한 구간에 쓴 시간 (프레임마다 0 으로) */

/* 다른 태그에게서 마지막으로 받은 방송 (제어 계산이 읽는다) */
static struct { uint32_t frame; uint16_t serial; uint8_t parts; } neighbor[UWBF_TAGS];
static struct record open_frame;      /* 지금 프레임의 F 기록 (아직 쌓지 않음) */
static bool frame_open;

/* 지금이 프레임의 조용한 구간인가. 박자를 모르면(앵커 1 이 안 들림) 늘 조용한 것으로 본다 */
static bool quiet_now(void)
{
    if (!fs.locked) {
        return true;
    }
    const uint32_t since = tagf_micros() - fs.t0_micros;
    if (since > 5U * TAGF_FRAME_US) {
        fs.locked = false;
        return true;
    }
    const uint32_t phase = since % TAGF_FRAME_US;
    return phase >= fs.radio_end_us + QUIET_AFTER_US && phase < TAGF_FRAME_US - QUIET_END_US;
}

/* 제어 계산: 프레임마다 한 번, 무선이 끝난 직후. 이 실험에는 알고리즘이 없어서 받은 방송을 읽기만 하고,
 * 비행용 빌드에서는 대본의 속도 명령을 정해 비행 제어기에 보낸다. 시작과 끝 시각을 F 줄에 남긴다. */
static void control_step(void)
{
    const uint32_t start = tagf_micros() - fs.t0_micros;
    uint8_t heard = 0;
    for (unsigned i = 0; i < UWBF_TAGS; ++i) {
        heard += neighbor[i].frame == fs.frame && neighbor[i].parts == (1U << UWBF_DATA_FRAGS) - 1U;
    }
#if TAGF_FLIGHT
#if TAGF_SD
    const uint8_t sd_state = sdlog_stats()->state;
    const bool log_ok = sd_active && (sd_state == SDLOG_IDLE || sd_state == SDLOG_WRITING || sd_state == SDLOG_WAIT_READY);
#else
    const bool log_ok = false;   /* SD 없이 만든 비행용 빌드는 대본을 시작하지 않는다 */
#endif
    flight_control(fs.frame, &fs.ctl, log_ok);
#endif
    const uint32_t end = tagf_micros() - fs.t0_micros;
    fs.control_done = true;
    if (frame_open && open_frame.frame == fs.frame) {
        open_frame.u.f.calc_start_us = (uint16_t)(start > 65535U ? 65535U : start);
        open_frame.u.f.calc_end_us = (uint16_t)(end > 65535U ? 65535U : end);
        open_frame.u.f.heard = heard;
    }
}

/* 한 글자를 UART 로. 어디서든 불러도 된다(싸다) */
static void uart_pump(void)
{
    if (uart_tail != uart_head && tagf_uart_ready()) {
        tagf_uart_put((uint8_t)uart_ring[uart_tail++ % UART_RING]);
    }
}

static bool uart_room(void)
{
    return UART_RING - (uart_head - uart_tail) >= LINE_MAX;
}

static void line_out(const char *line, unsigned n)
{
#if TAGF_SD
    if (sd_active) {
        sdlog_write(line, n);
        if (!uart_room()) {               /* SD 가 주 기록이다. UART 는 자리가 있을 때만 따라간다 */
            ++uart_skipped;
            return;
        }
    }
#endif
    for (unsigned i = 0; i < n; ++i) {
        uart_ring[uart_head++ % UART_RING] = line[i];
    }
}

/* 패킷을 기다리는 동안에만 부른다. 글자를 만드는 일, SD, 비행 제어기 통신은 조용한 구간에만 한다.
 * 한 번에 기록 하나만 다뤄서 수신 사건을 늦게 보는 일이 없게 한다. */
static void log_service(void)
{
    uart_pump();
    if (!quiet_now()) {
#if TAGF_SD
        if (sd_active && sdlog_stats()->state == SDLOG_WRITING) {
            sdlog_service(0);                 /* 걸어 둔 쓰기가 끝났는지만 본다 */
        }
#endif
        return;
    }
    if (fs.have_t0 && !fs.control_done && fs.locked) {
        control_step();            /* 조용한 구간의 첫 일 */
        return;
    }
    const uint32_t start = tagf_micros();
    bool worked = false;
#if TAGF_FLIGHT
    worked = flight_service(start);
#endif
#if TAGF_SD
    if (sd_active) {
        const struct sdlog_stats *st = sdlog_stats();
        if (st->state == SDLOG_WRITING) {
            sdlog_service(0);
        } else if (st->state != SDLOG_IDLE || st->queued != 0 || rec_tail == rec_head) {
            sdlog_service(1);                 /* 쓸 것이 있거나, 덜 찬 조각을 내보낼 때가 됐는지 본다 */
            worked = true;
        }
    }
    const bool sink_free = sd_active || uart_room();
#else
    const bool sink_free = uart_room();
#endif
    if (sink_free) {
        char line[LINE_MAX];
        unsigned n = 0;
        if (rec_tail != rec_head) {
            n = format_record(&records[rec_tail++ % RECORDS], line, sizeof(line));
        }
#if TAGF_FLIGHT
        else {
            n = flight_take_line(line, sizeof(line));
        }
#endif
        if (n != 0) {
            line_out(line, n);
            worked = true;
        }
    }
    if (worked) {
        const uint32_t took = tagf_micros() - start;
        work_us += took;
        if (took > work_max_us) {
            work_max_us = took;
        }
    }
}

/* ------------------------------------------------------------------ 시각 */
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

static uint32_t since_us(uint64_t rmarker, uint32_t now_hi)
{
    return hi32_to_ns(now_hi - hi32_of(rmarker)) / 1000U;
}

static uint32_t tx_lead_us(struct uwb7_timing t)
{
    const uint32_t lead = t.response_delay > TX_LEAD_BACK_US ? (uint32_t)t.response_delay - TX_LEAD_BACK_US : 0U;
    return lead > TX_LEAD_MAX_US ? TX_LEAD_MAX_US : lead < TX_LEAD_MIN_US ? TX_LEAD_MIN_US : lead;
}

static void rx_on(void)
{
    dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

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

/* ------------------------------------------------------------------ 프레임마다 들은 것 */
static uint32_t frames_logged;
static struct uwb7_timing applied;    /* 수신 켜는 때를 맞춰 둔 S, D1, D2 */
static bool rx_is_on;                 /* 방금 다룬 수신 사건 뒤에 수신을 이미 다시 켰다 */

static void close_frame(void)
{
    if (!frame_open) {
        return;
    }
    frame_open = false;
    open_frame.u.f.work_us = (uint16_t)(work_us > 65535U ? 65535U : work_us);
    open_frame.u.f.work_max_us = (uint16_t)(work_max_us > 65535U ? 65535U : work_max_us);
    open_frame.u.f.dropped = (uint16_t)log_dropped;
    work_us = work_max_us = 0;
    push_record(&open_frame);
    if (++frames_logged % STATS_EVERY == 0) {
        const struct record stats = { .kind = REC_STATS, .frame = open_frame.frame,
                                      .u.s = { log_dropped, rx_errors, rx_other, legacy_frames } };
        push_record(&stats);
#if TAGF_SD
        const struct sdlog_stats *st = sdlog_stats();
        const struct record sd = { .kind = REC_SD, .frame = open_frame.frame,
                                   .u.d = { st->next_block, st->written, st->dropped_bytes, st->errors,
                                            st->max_write_us, st->max_start_us, st->queued, uart_skipped,
                                            st->state } };
        push_record(&sd);
#endif
    }
}

/* 프레임 번호가 바뀌면 앞 프레임의 줄을 닫고 새로 연다 */
static void note_frame(uint32_t frame, uint8_t turn)
{
    if (frame_open && open_frame.frame == frame) {
        return;
    }
    close_frame();
    if (fs.frame != frame) {
        fs.have_t0 = fs.have_sched = false; /* 새 프레임인데 아직 앵커 1 의 Poll 은 못 받았다 */
        fs.acts = fs.next = 0;
        fs.prepared = false;
        fs.frame = frame;
    }
    open_frame = (struct record){ .kind = REC_FRAME, .frame = frame,
                                  .u.f = { .turn = turn, .response = '-', .sched = '-' } };
    frame_open = true;
    /* 영상과 맞추는 LED: 5초마다 0.1초, 60초마다 1초 */
    static uint8_t led_on;
    const uint8_t want = (frame % LED_PERIOD) < LED_ON || (frame % LED_LONG_PERIOD) < LED_LONG_ON;
    if (want != led_on) {
        led_on = want;
        tagf_led(want != 0);
        const struct record led = { .kind = REC_LED, .frame = frame, .u.l = { want } };
        push_record(&led);
    }
}

static void apply_timing(struct uwb7_timing t)
{
    if (t.slot == applied.slot && t.response_delay == applied.response_delay && t.final_delay == applied.final_delay) {
        return;
    }
    applied = t;
    /* 드라이버의 단위는 1.0256 µs (= 40/39 µs) */
    dwt_setrxaftertxdelay(t.final_delay > RX_EARLY_US ? ((uint32_t)t.final_delay - RX_EARLY_US) * 39U / 40U : 0U);
    dwt_setrxtimeout(0);
}

/* ------------------------------------------------------------------ 구간 시각 */
/* 앵커 1 의 Poll 뒤 us 인 때의 예약 시각 (DW 시각 상위 32비트, 맨 아래 비트는 칩이 버린다) */
static uint32_t slot_hi(uint32_t us)
{
    return hi32_of(fs.t0 + (uint64_t)us * UWB3_DTU_PER_UUS) & ~1U;
}

/* 실제 시각(40비트)이 예약(앵커 1 의 Poll + us)에서 얼마나 어긋났는가, ns */
static int32_t slot_error_ns(uint64_t actual, uint32_t us)
{
    /* 예약 송신의 RMARKER 는 예약 시각에 안테나 지연을 더한 때다 */
    const uint64_t planned = (fs.t0 + (uint64_t)us * UWB3_DTU_PER_UUS + UWB3_ANTENNA_DELAY) & DTU_40BIT_MASK;
    const uint64_t diff = (actual - planned) & DTU_40BIT_MASK;
    if (diff < (1ULL << 39)) {
        return diff > 0x3FFFFFFFULL * 60ULL ? DT_UNKNOWN : (int32_t)dtu_to_ns(diff);
    }
    const uint64_t back = (planned - actual) & DTU_40BIT_MASK;
    return back > 0x3FFFFFFFULL * 60ULL ? DT_UNKNOWN : -(int32_t)dtu_to_ns(back);
}

/* 받은 패킷이 있어야 할 시각 (앵커 1 의 Poll 기준 µs) */
static uint32_t expected_us(const struct uwbf_packet *p)
{
    if (p->type == UWBF_SCHED) {
        return uwbf_sched_us(fs.timing, &fs.ctl);
    }
    if (p->type == UWBF_XCHG) {
        return uwbf_extra_us(fs.timing, &fs.ctl, fs.flags, p->slot - 1U) + p->part * 10U * fs.ctl.xchg;
    }
    return (p->slot == 0 ? uwbf_heartbeat_us(fs.timing, &fs.ctl, fs.flags)
                         : uwbf_extra_us(fs.timing, &fs.ctl, fs.flags, p->slot - 1U)) + p->part * 10U * fs.ctl.frag;
}

/* ------------------------------------------------------------------ 내가 보낼 것들 */
static void add_action(char type, uint32_t us, uint8_t slot, uint8_t peer)
{
    if (fs.acts < ACTIONS) {
        fs.act[fs.acts++] = (struct action){ us, slot, peer, type };
    }
}

/* 일정을 알게 됐을 때 부른다: heartbeat 와 추가 슬롯 가운데 내 것을 시각 순서로 넣는다 */
static void plan_broadcasts(void)
{
    if (fs.turn == TAGF_ID) {
        add_action('H', uwbf_heartbeat_us(fs.timing, &fs.ctl, fs.flags), 0, UWBF_NOBODY);
    }
    for (unsigned j = 0; j < fs.ctl.extra && j < UWBF_EXTRA_MAX; ++j) {
        if (fs.ctl.owner[j] != TAGF_ID) {
            continue;
        }
        const uint32_t us = uwbf_extra_us(fs.timing, &fs.ctl, fs.flags, j);
        if (!(fs.flags & UWBF_FLAG_EXCHANGE)) {
            add_action('B', us, (uint8_t)(j + 1U), UWBF_NOBODY);
        } else if (fs.ctl.partner[j] < UWBF_TAGS && fs.ctl.partner[j] != TAGF_ID) {
            add_action('X', us, (uint8_t)(j + 1U), fs.ctl.partner[j]);
        }
    }
}

/* 다음에 보낼 것의 데이터를 미리 칩에 써 둔다(수신 중에도 된다). 그래야 보낼 때 예약 명령만 내리면 된다 */
static void prepare_next(void)
{
    if (fs.prepared || fs.next >= fs.acts) {
        return;
    }
    const struct action *a = &fs.act[fs.next];
    struct uwbf_packet p = { fs.frame, serial_next, UWBF_DATA, TAGF_ID, a->peer, a->slot, 0, UWBF_DATA_FRAGS };
    uint16_t length = UWBF_DATA_LEN;
    if (a->type == 'C') {
        uwbf_encode_sched(fs.buffer, fs.frame, TAGF_ID, fs.flags, &fs.ctl);
        length = UWBF_SCHED_LEN;
    } else if (a->type == 'X') {
        p.type = UWBF_XCHG;
        uwbf_encode_xchg(fs.buffer, &p);
        length = UWBF_XCHG_LEN;
    } else {
        uwbf_encode_data(fs.buffer, &p);
    }
    (void)dwt_writetxdata(length, fs.buffer, 0);
    dwt_writetxfctrl(length + UWB3_FCS, 0, 1);
    fs.prepared = true;
}

static struct record packet_record(char type, char dir, const struct uwbf_packet *p, uint16_t bytes, uint32_t us)
{
    const struct record r = { .kind = REC_PACKET, .frame = p->frame,
                              .u.p = { .dt_ns = DT_UNKNOWN, .serial = p->serial, .sched_us = (uint16_t)us,
                                       .rssi = UWB3_UNKNOWN_POWER, .first_path = UWB3_UNKNOWN_POWER,
                                       .slot = p->slot, .part = p->part, .src = p->sender, .dst = p->peer,
                                       .bytes = (uint8_t)bytes, .type = type, .dir = dir } };
    return r;
}

/* 받은 패킷의 기록 가운데 신호 세기를 아직 못 읽은 것. 내 송신을 먼저 예약한 뒤에 읽는다 */
static struct record pending_rx;
static bool have_pending_rx;

static void finish_pending_rx(void)
{
    if (have_pending_rx) {
        const struct uwb3_power power = uwb3_read_power();
        pending_rx.u.p.rssi = power.rssi;
        pending_rx.u.p.first_path = power.first_path;
        push_record(&pending_rx);
        have_pending_rx = false;
    }
}

/* 예약 송신 하나: 데이터는 이미 칩에 있다. 송신이 끝날 때까지 기다리고 기록을 채운다. 참이면 보냈다 */
static bool send_at(uint32_t us, uint8_t mode, struct record *r)
{
    const uint32_t hi = slot_hi(us);
    dwt_setdelayedtrxtime(hi);
    const bool sent = dwt_starttx(mode) == DWT_SUCCESS;
    finish_pending_rx();
    r->u.p.t_hi = hi;
    if (!sent) {
        return false;
    }
    const uint32_t give_up = hi + uwb7_us_to_hi(TX_DONE_US);
    while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK)) {
        if ((int32_t)(dwt_readsystimestamphi32() - give_up) >= 0) {
            dwt_forcetrxoff();
            return false;
        }
        uart_pump();
    }
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    r->u.p.ok = 1;
    return true;
}

/* 보낸 패킷의 기록을 마무리한다. 예약 송신은 예약한 시각(+ 안테나 지연)에 정확히 나가거나 아예 실패하므로,
 * 실제 시각은 칩에서 다시 읽지 않고 예약한 값으로 적는다(어긋남은 예약 단위 8 ns 로 자른 만큼뿐이다). */
static void stamp_tx(struct record *r, uint32_t us)
{
    if (r->u.p.ok) {
        r->u.p.dt_ns = slot_error_ns((((uint64_t)r->u.p.t_hi << 8) + UWB3_ANTENNA_DELAY) & DTU_40BIT_MASK, us);
    }
    push_record(r);
}

/* 수신을 기다린다. 사건이 나면 상태를, 시간이 지나면 0 을 돌려준다 */
static uint32_t wait_rx_us(uint32_t until_us)
{
    return wait_rx_until(slot_hi(until_us));
}

static uint16_t read_head(uint8_t *head, uint16_t *have)
{
    return uwbf_read_head(head, HEAD_BYTES, have);
}

static char packet_letter(const struct uwbf_packet *p)
{
    return p->type == UWBF_SCHED ? 'C' : p->type == UWBF_XCHG ? 'X' : p->slot == 0 ? 'H' : 'B';
}

/* 방금 받은 드론 패킷의 기록을 만든다 (신호 세기는 아직) */
static struct record received_record(const struct uwbf_packet *p, uint16_t length)
{
    const uint64_t rx = uwb3_timestamp(false);
    const bool mine = fs.have_t0 && fs.frame == p->frame;
    struct record r = packet_record(packet_letter(p), 'R', p, (uint16_t)(length - UWB3_FCS),
                                    mine ? expected_us(p) : 0U);
    r.u.p.t_hi = hi32_of(rx);
    r.u.p.cko = dwt_readclockoffset();
    r.u.p.ok = 1;
    if (mine) {
        r.u.p.dt_ns = slot_error_ns(rx, r.u.p.sched_us);
    }
    return r;
}

/* 방송: 조각 2개. 둘째 조각은 조각 번호 한 바이트만 고쳐 쓴다 */
static void run_broadcast(const struct action *a)
{
    struct uwbf_packet p = { fs.frame, serial_next++, UWBF_DATA, TAGF_ID, UWBF_NOBODY, a->slot, 0, UWBF_DATA_FRAGS };
    struct record first = packet_record(a->type, 'T', &p, UWBF_DATA_LEN, a->us);
    const bool sent = send_at(a->us, DWT_START_TX_DELAYED, &first);
    p.part = 1;
    const uint32_t us2 = a->us + 10U * fs.ctl.frag;
    struct record second = packet_record(a->type, 'T', &p, UWBF_DATA_LEN, us2);
    uint8_t part = 1;
    (void)dwt_writetxdata(1, &part, 15);
    const uint32_t hi2 = slot_hi(us2);
    dwt_setdelayedtrxtime(hi2);
    const bool sent2 = dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS;
    (void)sent;
    stamp_tx(&first, a->us);
    second.u.p.t_hi = hi2;
    if (sent2) {
        const uint32_t give_up = hi2 + uwb7_us_to_hi(TX_DONE_US);
        second.u.p.ok = 1;
        while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK)) {
            if ((int32_t)(dwt_readsystimestamphi32() - give_up) >= 0) {
                dwt_forcetrxoff();
                second.u.p.ok = 0;
                break;
            }
            uart_pump();
        }
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    }
    stamp_tx(&second, us2);
}

static void run_sched(const struct action *a)
{
    struct uwbf_packet p = { fs.frame, 0, UWBF_SCHED, TAGF_ID, UWBF_NOBODY, 0, 0, 1 };
    struct record r = packet_record('C', 'T', &p, UWBF_SCHED_LEN, a->us);
    (void)send_at(a->us, DWT_START_TX_DELAYED, &r);
    stamp_tx(&r, a->us);
}

/* 교환의 뒷부분: 내가 보낸 패킷(us_tx) 뒤에 상대의 다음 단계(want)를 기다린다. 받으면 그 기록을 쌓고 참 */
static bool exchange_listen(uint32_t us_rx, uint8_t slot, uint8_t peer, uint8_t want)
{
    uint8_t head[HEAD_BYTES];
    uint16_t have;
    struct uwbf_packet q;
    const uint32_t status = wait_rx_us(us_rx + XCHG_WAIT_US);
    bool ok = false;
    if (status & DWT_INT_RXFCG_BIT_MASK) {
        const uint16_t length = read_head(head, &have);
        if (uwbf_decode_drone(head, length, have, &q, NULL) && q.type == UWBF_XCHG && q.frame == fs.frame &&
            q.slot == slot && q.sender == peer && q.peer == TAGF_ID && q.part == want) {
            pending_rx = received_record(&q, length);
            have_pending_rx = true;
            ok = true;
        } else {
            ++rx_other;
        }
    } else if (status != 0) {
        ++rx_errors;
        ++open_frame.u.f.rx_err;
    } else {
        dwt_forcetrxoff();
    }
    dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
    return ok;
}

/* 교환을 내가 시작한다: 0 단계를 보내고, 1 단계를 받으면 2 단계를 보낸다 */
static void run_exchange(const struct action *a)
{
    const uint32_t gap = 10U * fs.ctl.xchg;
    struct uwbf_packet p = { fs.frame, serial_next++, UWBF_XCHG, TAGF_ID, a->peer, a->slot, 0, 3 };
    struct record r = packet_record('X', 'T', &p, UWBF_XCHG_LEN, a->us);
    applied = (struct uwb7_timing){ 0 };   /* 아래에서 수신 켜는 때를 바꾸니, 다음 묶음에서 다시 맞추게 한다 */
    dwt_setrxaftertxdelay(0);
    const bool sent = send_at(a->us, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED, &r);
    stamp_tx(&r, a->us);
    if (!sent || !exchange_listen(a->us + gap, a->slot, a->peer, 1)) {
        dwt_forcetrxoff();
        finish_pending_rx();
        return;
    }
    p.part = 2;
    uwbf_encode_xchg(fs.buffer, &p);
    (void)dwt_writetxdata(UWBF_XCHG_HEAD, fs.buffer, 0);  /* 빈 데이터는 0 단계와 같다. 머리만 고쳐 쓴다 */
    r = packet_record('X', 'T', &p, UWBF_XCHG_LEN, a->us + 2U * gap);
    (void)send_at(a->us + 2U * gap, DWT_START_TX_DELAYED, &r);
    stamp_tx(&r, a->us + 2U * gap);
}

/* 교환의 상대 쪽: 나에게 온 0 단계(q)를 방금 받았다. 1 단계를 보내고 2 단계를 기다린다 */
static void answer_exchange(const struct uwbf_packet *q)
{
    const uint32_t gap = 10U * fs.ctl.xchg;
    const uint32_t us = uwbf_extra_us(fs.timing, &fs.ctl, fs.flags, q->slot - 1U) + gap;
    struct uwbf_packet p = { fs.frame, q->serial, UWBF_XCHG, TAGF_ID, q->sender, q->slot, 1, 3 };
    struct record r = packet_record('X', 'T', &p, UWBF_XCHG_LEN, us);
    uwbf_encode_xchg(fs.buffer, &p);
    (void)dwt_writetxdata(UWBF_XCHG_LEN, fs.buffer, 0);
    dwt_writetxfctrl(UWBF_XCHG_LEN + UWB3_FCS, 0, 1);
    fs.prepared = false;                   /* 미리 써 둔 내 방송 데이터를 덮어썼다 */
    applied = (struct uwb7_timing){ 0 };
    dwt_setrxaftertxdelay(0);
    const bool sent = send_at(us, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED, &r);
    stamp_tx(&r, us);
    if (sent) {
        (void)exchange_listen(us + gap, q->slot, q->sender, 2);
    }
    dwt_forcetrxoff();
    finish_pending_rx();
}

static void run_action(void)
{
    const struct action a = fs.act[fs.next];
    dwt_forcetrxoff();
    dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
    prepare_next();
    ++fs.next;
    fs.prepared = false;
    if (a.type == 'C') {
        run_sched(&a);
    } else if (a.type == 'X') {
        run_exchange(&a);
    } else {
        run_broadcast(&a);
    }
    finish_pending_rx();
    prepare_next();
}

/* 다음에 보낼 것이 있고 이제 듣기를 멈출 때인가 */
static bool action_due(void)
{
    if (!fs.have_t0 || fs.next >= fs.acts) {
        return false;
    }
    const uint32_t us = fs.act[fs.next].us;
    if ((int32_t)(fs.t0_micros + us - ACT_FINE_US - tagf_micros()) > 0) {
        return false;
    }
    const uint32_t stop_hi = slot_hi(us) - uwb7_us_to_hi(UWB7_SHR_US + ACT_LEAD_US);
    return (int32_t)(dwt_readsystimestamphi32() - stop_hi) >= 0;
}

/* 곧 보내야 해서 수신을 다시 켤 틈이 없는가 */
static bool action_soon(uint32_t within_us)
{
    return fs.have_t0 && fs.next < fs.acts &&
           (int32_t)(fs.t0_micros + fs.act[fs.next].us - UWB7_SHR_US - ACT_LEAD_US - within_us - tagf_micros()) <= 0;
}

/* 앵커 1 의 Poll 을 받았다: 이 프레임의 박자와 일정을 잡는다 */
static void begin_frame(const struct uwbf_frame *info, uint64_t poll1_rx)
{
    static struct uwbf_control logged;
    static struct uwb7_timing logged_timing;
    static uint8_t logged_flags, logged_once;
    static uint32_t logged_frame;
    if (info->frame < logged_frame) {
        logged_once = 0;   /* 앵커 1 이 다시 켜졌다(프레임 번호가 처음부터). 설정 줄을 다시 남긴다 */
    }
    logged_frame = info->frame;
    fs.have_t0 = true;
    fs.locked = true;
    fs.frame = info->frame;
    fs.t0 = poll1_rx;
    fs.t0_micros = tagf_micros() - since_us(poll1_rx, dwt_readsystimestamphi32());
    fs.timing = info->timing;
    fs.ctl = info->ctl;
    fs.flags = info->flags;
    fs.turn = info->turn;
    fs.acts = fs.next = 0;
    fs.prepared = false;
    fs.control_done = false;
    fs.radio_end_us = uwbf_radio_end_us(fs.timing, &fs.ctl, fs.flags);
    if (fs.flags & UWBF_FLAG_COORD_DRONE) {
        /* 일정은 드론 0번이 만든다. 0번은 지금 만들고, 나머지는 일정 패킷을 받아야 안다 */
        fs.have_sched = TAGF_ID == UWBF_COORD_DRONE_ID;
        if (fs.have_sched) {
            uwbf_round_robin(fs.frame, &fs.ctl);
            add_action('C', uwbf_sched_us(fs.timing, &fs.ctl), 0, UWBF_NOBODY);
        }
        open_frame.u.f.sched = fs.have_sched ? 'D' : 'N';
    } else {
        fs.have_sched = true;
        open_frame.u.f.sched = 'A';
    }
    if (fs.have_sched) {
        plan_broadcasts();
    }
    open_frame.u.f.t0_hi = hi32_of(poll1_rx);
    open_frame.u.f.turn = info->turn;
    open_frame.u.f.extra = fs.ctl.extra;
    open_frame.u.f.flags = fs.flags;
    open_frame.u.f.radio_end_us = (uint16_t)fs.radio_end_us;
    /* 제어가 바뀌면 한 줄 남긴다 (주인과 상대는 프레임마다 바뀌므로 보지 않는다) */
    struct uwbf_control base = info->ctl;
    memset(base.owner, 0, sizeof(base.owner));
    memset(base.partner, 0, sizeof(base.partner));
    if (!logged_once || memcmp(&base, &logged, sizeof(base)) != 0 || logged_flags != info->flags ||
        memcmp(&logged_timing, &info->timing, sizeof(logged_timing)) != 0) {
        const struct record k = { .kind = REC_CONTROL, .frame = info->frame,
                                  .u.k = { info->ctl, info->timing, info->flags } };
        push_record(&k);
        logged = base;
        logged_timing = info->timing;
        logged_flags = info->flags;
        logged_once = 1;
    }
}

/* 드론이 보낸 패킷(방송 조각, 일정, 교환)을 받았다 */
static void on_drone_packet(const struct uwbf_packet *p, const struct uwbf_control *sched, uint16_t length)
{
    note_frame(p->frame, (uint8_t)(p->frame % UWBF_TAGS));
    struct record r = received_record(p, length);
    if (p->type == UWBF_DATA && p->sender < UWBF_TAGS) {
        if (neighbor[p->sender].frame != p->frame || neighbor[p->sender].serial != p->serial) {
            neighbor[p->sender].frame = p->frame;
            neighbor[p->sender].serial = p->serial;
            neighbor[p->sender].parts = 0;
        }
        neighbor[p->sender].parts |= (uint8_t)(1U << p->part);
    }
    const bool mine = fs.have_t0 && fs.frame == p->frame;
    if (p->type == UWBF_SCHED && mine && (fs.flags & UWBF_FLAG_COORD_DRONE) && !fs.have_sched &&
        p->sender == UWBF_COORD_DRONE_ID) {
        fs.ctl.extra = sched->extra;
        memcpy(fs.ctl.owner, sched->owner, sizeof(fs.ctl.owner));
        memcpy(fs.ctl.partner, sched->partner, sizeof(fs.ctl.partner));
        fs.have_sched = true;
        fs.radio_end_us = uwbf_radio_end_us(fs.timing, &fs.ctl, fs.flags);
        open_frame.u.f.sched = 'D';
        open_frame.u.f.extra = fs.ctl.extra;
        open_frame.u.f.radio_end_us = (uint16_t)fs.radio_end_us;
        plan_broadcasts();
        prepare_next();   /* heartbeat 가 바로 뒤일 수 있다. 데이터를 지금 써 둔다 */
    }
    if (p->type == UWBF_XCHG && p->part == 0 && p->peer == TAGF_ID && mine && fs.have_sched &&
        (fs.flags & UWBF_FLAG_EXCHANGE)) {
        pending_rx = r;
        have_pending_rx = true;
        answer_exchange(p);
        prepare_next();
        return;
    }
    if (action_soon(250)) {
        /* 곧 내 차례다. 신호 세기는 송신을 예약한 뒤에 읽는다 */
        pending_rx = r;
        have_pending_rx = true;
        return;
    }
#if TAGF_EARLY_REARM
    rx_on();
    rx_is_on = true;
#endif
    const struct uwb3_power power = uwb3_read_power();
    r.u.p.rssi = power.rssi;
    r.u.p.first_path = power.first_path;
    push_record(&r);
    prepare_next();
}

/* ------------------------------------------------------------------ 내 차례: 묶음 하나 */
struct exchange {
    struct uwb3_key key;
    uint64_t poll_rx, final_rx;
    uint8_t final_frame[UWB3_FINAL_LEN + UWB3_FCS];
    uint8_t have_poll, have_final;
    char end;
};

/* 앵커 1 의 Poll(key, info)을 방금 받았고 내 차례다. 끝나면 수신은 꺼져 있다. */
static void run_bundle(struct uwb3_key key, const struct uwbf_frame *info, uint64_t poll1_rx)
{
    uint8_t frame[HEAD_BYTES];
    struct exchange x[UWB3_ANCHORS] = { 0 };
    struct uwbf_frame other;
    const struct uwb7_timing t = info->timing;
    const uint32_t shr_hi = uwb7_us_to_hi(UWB7_SHR_US);
    const uint32_t slot_hi = uwb7_us_to_hi(t.slot);
    const uint32_t resp_hi = hi32_of(poll1_rx + (uint64_t)(2U * t.slot + t.response_delay) * UWB3_DTU_PER_UUS) & ~1U;
    const uint32_t stop_hi = resp_hi - shr_hi - uwb7_us_to_hi(tx_lead_us(t));
    uint32_t status, now_hi, turn_us = 0;
    uint8_t tag;

    apply_timing(t);
    x[0] = (struct exchange){ .key = key, .poll_rx = poll1_rx, .have_poll = 1, .end = 'T' };

    /* 1) 앵커 2, 3 의 Poll 을 듣는다. 앵커 3 의 것을 받으면 바로 다음 단계로 간다. */
    bool more = true, receiving = false;
    uint64_t last_rx = poll1_rx;
    while (more) {
        rx_on();
        receiving = true;
        now_hi = dwt_readsystimestamphi32();
        if (since_us(last_rx, now_hi) > turn_us) {
            turn_us = since_us(last_rx, now_hi);
        }
        status = wait_rx_until(stop_hi);
        if (status == 0) {
            break;
        }
        receiving = false; /* 수신 사건이 나면 칩이 수신을 끈다 */
        if (status & DWT_INT_RXFCG_BIT_MASK) {
            if (uwbf_decode_poll(frame, uwbf_read(frame, sizeof(frame)), &key, &other) &&
                other.frame == info->frame && !x[key.anchor - 1].have_poll) {
                x[key.anchor - 1] = (struct exchange){ .key = key, .poll_rx = uwb3_timestamp(false),
                                                       .have_poll = 1, .end = 'T' };
                last_rx = x[key.anchor - 1].poll_rx;
                open_frame.u.f.seen |= (uint8_t)(1U << (key.anchor - 1));
                more = key.anchor < UWB3_ANCHORS;
            } else {
                ++rx_other;
            }
        } else {
            ++rx_errors;
            ++open_frame.u.f.rx_err;
        }
    }

    /* 2) Response 를 예약한다. 받은 Poll 의 회차 번호를 싣는다. */
    struct uwbf_response response = { *info, TAGF_ID, { 0 } };
    unsigned last_anchor = 0, expected = 0;
    for (unsigned i = 0; i < UWB3_ANCHORS; ++i) {
        if (x[i].have_poll) {
            response.round[i] = x[i].key.round;
            last_anchor = i + 1;
            ++expected;
        }
    }
    response.info.flags = 0;
    if (receiving) {
        dwt_forcetrxoff();
    }
    uwbf_encode_response(frame, &response);
    dwt_setdelayedtrxtime(resp_hi);
    const int sent = uwb3_send(frame, UWBF_RESPONSE_LEN, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    now_hi = dwt_readsystimestamphi32();
    dwt_writesysstatuslo(UWB3_RX_EVENTS); /* Poll 의 수신 표시를 지운다. 송신 끝 표시는 건드리지 않는다 */
    char kind = 'N';
    uint32_t end_seen_hi = now_hi;
    struct exchange *last = NULL;
    if (sent != DWT_SUCCESS) {
        kind = 'L';
    } else {
        const int32_t lead_hi = (int32_t)(resp_hi - shr_hi - now_hi);
        open_frame.u.f.lead_us = (int16_t)(lead_hi >= 0 ? (int32_t)(hi32_to_ns((uint32_t)lead_hi) / 1000U)
                                                        : -(int32_t)(hi32_to_ns((uint32_t)-lead_hi) / 1000U));

        /* 3) Final 을 듣는다. 송신이 끝나면 칩이 스스로 수신을 켠다. */
        const uint32_t final_shr_hi = resp_hi + uwb7_us_to_hi(t.final_delay) - shr_hi;
        const uint32_t end_hi = resp_hi + uwb7_us_to_hi(t.final_delay + (last_anchor - 1U) * t.slot + END_MARGIN_US);
        for (;;) {
            status = dwt_readsysstatuslo();
            if (status & DWT_INT_TXFRS_BIT_MASK) {
                kind = 'R';
            }
            if (status & UWB3_RX_EVENTS) {
                if (status & DWT_INT_RXFCG_BIT_MASK) {
                    struct exchange *e = NULL;
                    if (uwbf_decode_final(frame, uwbf_read(frame, sizeof(frame)), &key, &tag) && tag == TAGF_ID) {
                        e = &x[key.anchor - 1];
                    }
                    if (e != NULL && e->have_poll && !e->have_final && uwb3_same_key(key, e->key)) {
                        e->final_rx = uwb3_timestamp(false);
                        memcpy(e->final_frame, frame, sizeof(e->final_frame));
                        e->have_final = 1;
                        last = e;
                        open_frame.u.f.seen |= (uint8_t)(SEEN_FINAL << (key.anchor - 1));
                        --expected;
                    } else {
                        ++rx_other;
                    }
                } else {
                    /* 어느 앵커의 Final 자리였는지는 시각으로 가린다 */
                    const uint32_t since_hi = dwt_readsystimestamphi32() - final_shr_hi;
                    const uint32_t slot = (int32_t)since_hi > 0 ? since_hi / slot_hi : 0U;
                    if (slot < UWB3_ANCHORS && x[slot].have_poll && !x[slot].have_final) {
                        x[slot].end = 'E';
                    }
                    ++rx_errors;
                    ++open_frame.u.f.rx_err;
                }
                if (expected == 0) {
                    break;
                }
                rx_on();
                now_hi = dwt_readsystimestamphi32();
                if ((int32_t)(now_hi - end_hi) >= 0) {
                    break;
                }
                if (last != NULL && since_us(last->final_rx, now_hi) > turn_us) {
                    turn_us = since_us(last->final_rx, now_hi);
                }
            } else if ((int32_t)(dwt_readsystimestamphi32() - end_hi) >= 0) {
                break;
            }
        }
        end_seen_hi = dwt_readsystimestamphi32();
        dwt_forcetrxoff();
        dwt_writesysstatuslo(UWB3_ATTEMPT_EVENTS);
    }

    /* 4) 거리. 신호 세기는 마지막으로 받은 Final 것만 읽는다(그 뒤로 수신한 것이 없어 값이 남아 있다). */
    struct uwb3_power power = { UWB3_UNKNOWN_POWER, UWB3_UNKNOWN_POWER };
    if (last != NULL) {
        power = uwb3_read_power();
    }
    const uint32_t response_tx = (uint32_t)uwb3_timestamp(true);
    for (unsigned i = 0; i < UWB3_ANCHORS; ++i) {
        struct exchange *e = &x[i];
        if (!e->have_poll) {
            continue;
        }
        struct record r = { .kind = REC_RANGE, .frame = info->frame,
                            .u.r = { .poll_hi = hi32_of(e->poll_rx), .anchor = (uint8_t)(i + 1),
                                     .rssi = UWB3_UNKNOWN_POWER, .first_path = UWB3_UNKNOWN_POWER,
                                     .end = kind == 'L' ? 'L' : e->end } };
        if (e->have_final) {
            r.u.r.end = 'F';
            r.u.r.end_ns = dtu_to_ns((e->final_rx - e->poll_rx) & DTU_40BIT_MASK);
            r.u.r.valid = uwb3_distance_mm(e->final_frame, (uint32_t)e->poll_rx, response_tx, (uint32_t)e->final_rx,
                                           &r.u.r.mm);
            if (e == last) {
                r.u.r.rssi = power.rssi;
                r.u.r.first_path = power.first_path;
            }
        } else {
            r.u.r.end_ns = hi32_to_ns(end_seen_hi - hi32_of(e->poll_rx));
        }
        push_record(&r);
    }
    open_frame.u.f.response = kind;
    open_frame.u.f.turn_us = (uint16_t)(turn_us > 65535U ? 65535U : turn_us);
}

/* This entry is called by the handoff project's TEST_DS_TWR_RESPONDER selection. */
int ds_twr_responder(void)
{
    char line[200];
    tagf_board_init();
    reset_DWIC();
    Sleep(2);
    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS ||
        !dwt_checkidlerc() || dwt_initialise(DWT_READ_OTP_ALL) != DWT_SUCCESS ||
        dwt_configure(&config_options) != DWT_SUCCESS) {
        test_run_info((unsigned char *)"TF_INIT_FAILED");
#if TAGF_FLIGHT
        /* UWB 칩이 안 잡혀도 노트북과 비행 제어기 사이의 중계는 살려 둔다(이륙·착륙 명령이 이 보드를 지난다) */
        flight_init(TAGF_ID, TAGF_FRAME_US);
        for (;;) {
            (void)flight_service(tagf_micros());
        }
#else
        for (;;) { }
#endif
    }
    port_set_dw_ic_spi_fastrate();
    dwt_configuretxrf(&txconfig_options);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_settxantennadelay(UWB3_ANTENNA_DELAY);
    dwt_setrxantennadelay(UWB3_ANTENNA_DELAY);
    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);
    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout(0);
    test_run_info((unsigned char *)"TF_READY,channel=5");
    snprintf(line, sizeof(line), "TF_INFO,fw=" TAG_FW ",id=%u,tags=%u,addr=%u,ant_dly=%u,records=%u,sd=%u,flight=%u,"
             "frame_us=%u,data_bytes=%u,xchg_bytes=%u,early_rearm=%u,opt=" TAGF_BUILD_OPT,
             (unsigned)TAGF_ID, (unsigned)UWBF_TAGS, (unsigned)(UWB3_TAG + TAGF_ID), (unsigned)UWB3_ANTENNA_DELAY,
             (unsigned)RECORDS, (unsigned)TAGF_SD, (unsigned)TAGF_FLIGHT, (unsigned)TAGF_FRAME_US,
             (unsigned)UWBF_DATA_LEN, (unsigned)UWBF_XCHG_LEN, (unsigned)TAGF_EARLY_REARM);
    test_run_info((unsigned char *)line);
#if TAGF_SD
    {
        /* 카드를 켜고 기록 파일을 찾는다(수백 ms). 시작 줄을 카드에도 남겨 켤 때마다의 기록을 나눌 수 있게 한다 */
        char sd_line[160];
        sd_active = sdlog_init() != 0;
        const struct sdlog_stats *st = sdlog_stats();
        snprintf(sd_line, sizeof(sd_line), "TF_SD,ready=%u,card=%u,card_error=0x%08lX,file=%s,first_block=%lu,"
                 "file_blocks=%lu,next_block=%lu", (unsigned)sd_active, (unsigned)st->card_ok,
                 (unsigned long)st->init_error, sdfat_result_name((enum sdfat_result)st->file_result),
                 (unsigned long)st->first_block, (unsigned long)st->file_blocks, (unsigned long)st->next_block);
        test_run_info((unsigned char *)sd_line);
        if (sd_active) {
            static const char ready[] = "TF_READY,channel=5\n";
            sdlog_write(ready, sizeof(ready) - 1U);
            sdlog_write(line, (unsigned)strlen(line));
            sdlog_write("\n", 1);
            sdlog_write(sd_line, (unsigned)strlen(sd_line));
            sdlog_write("\n", 1);
        }
    }
#endif
#if TAGF_FLIGHT
    flight_init(TAGF_ID, TAGF_FRAME_US);
#endif

    rx_on();
    for (;;) {
        uint32_t status;
        for (;;) {
            status = dwt_readsysstatuslo();
            if (status & UWB3_RX_EVENTS) {
                break;
            }
            if (action_due()) {
                status = 0;
                break;
            }
            log_service(); /* 기다리는 동안에만 기록을 내보낸다 */
        }
        if (status == 0) {
            run_action();
            rx_on();
            continue;
        }
        uint8_t frame[HEAD_BYTES];
        uint16_t have = 0;
        struct uwb3_key key;
        struct uwbf_frame info;
        struct uwbf_response response;
        struct uwbf_packet packet;
        struct uwbf_control sched = uwbf_control_default();
        uint8_t tag;
        rx_is_on = false;
        if (status & DWT_INT_RXFCG_BIT_MASK) {
            const uint16_t length = read_head(frame, &have);
            if (uwbf_decode_drone(frame, length, have, &packet, &sched)) {
                on_drone_packet(&packet, &sched, length);
            } else if (uwbf_decode_poll(frame, length, &key, &info)) {
                note_frame(info.frame, info.turn);
                open_frame.u.f.seen |= (uint8_t)(1U << (key.anchor - 1));
                if (key.anchor == UWBF_MASTER_ANCHOR && (info.flags & UWBF_FLAG_MASTER)) {
                    const uint64_t poll1_rx = uwb3_timestamp(false);
                    begin_frame(&info, poll1_rx);
                    if (info.turn == TAGF_ID) {
                        run_bundle(key, &info, poll1_rx);
                    }
                    prepare_next();
                }
            } else if (uwbf_decode_response(frame, length, &response)) {
                note_frame(response.info.frame, response.tag);
                open_frame.u.f.seen |= SEEN_RESPONSE;
            } else if (uwbf_decode_final(frame, length, &key, &tag)) {
                if (frame_open) {
                    open_frame.u.f.seen |= (uint8_t)(SEEN_FINAL << (key.anchor - 1));
                }
            } else if (length != 0 && (frame[9] == UWB3_POLL || frame[9] == UWB3_RESPONSE || frame[9] == UWB3_FINAL ||
                                       frame[9] == UWB7_POLL || frame[9] == UWB7_RESPONSE || frame[9] == UWB7_FINAL ||
                                       frame[9] == UWBF_POLL_V1)) {
                /* 9패킷, 태그 한 대용 7패킷, 프레임 방식 1판 펌웨어가 섞여 있다. 프레임이 안 잡혀도 보이게 S 줄을 남긴다 */
                if (++legacy_frames == 1 || legacy_frames % 500U == 0) {
                    const struct record stats = { .kind = REC_STATS, .frame = 0,
                                                  .u.s = { log_dropped, rx_errors, rx_other, legacy_frames } };
                    push_record(&stats);
                }
            } else {
                ++rx_other;
            }
        } else if (status & SYS_STATUS_ALL_RX_ERR) {
            ++rx_errors;
            if (frame_open) {
                ++open_frame.u.f.rx_err;
            }
        }
        /* 곧 내가 보낼 차례면 수신을 켰다가 바로 끄는 데 시간을 쓰지 않는다 */
        if (!rx_is_on) {
            if (action_soon(200)) {
                dwt_writesysstatuslo(UWB3_RX_EVENTS); /* 방금 다룬 수신 표시만 지운다 */
            } else {
                rx_on();
            }
        }
    }
}
