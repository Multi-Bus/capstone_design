/* 비행 제어기 연결: MAVLink 중계, 대본, 안전. 설명은 fcl.h */
#include "fcl.h"

#include <stdio.h>
#include <string.h>

#include "link_failsafe.h"   /* n6_bridge 와 같은 링크 끊김 강제 DISARM */

enum {
    MAV_V1 = 0xFE, MAV_V2 = 0xFD,
    MSG_HEARTBEAT = 0, MSG_ATTITUDE = 30, MSG_LOCAL_POSITION_NED = 32, MSG_COMMAND_LONG = 76, MSG_COMMAND_ACK = 77,
    MSG_SET_POSITION_TARGET_LOCAL_NED = 84,
    CRC_HEARTBEAT = 50, CRC_ATTITUDE = 39, CRC_LOCAL_POSITION_NED = 185, CRC_COMMAND_LONG = 152, CRC_COMMAND_ACK = 143,
    CRC_SET_POSITION_TARGET = 143,
    CMD_DO_SET_MODE = 176, CMD_SET_MESSAGE_INTERVAL = 511,
    MAV_FRAME_BODY_NED = 8,
    /* 위치와 가속도, 기수 방향은 무시하고 속도와 회전 속도(0)만 쓴다 -> 기수 방향이 그대로 유지된다 */
    TYPE_MASK_VELOCITY = 0x0007 | 0x01C0 | 0x0400,
    MAV_TYPE_GCS = 6, MAV_COMP_AUTOPILOT = 1, MAV_MODE_FLAG_ARMED = 0x80,
    FRAME_MAX = 280 + 13,
    STALL_MS = 50,                 /* 메시지가 중간에 이만큼 멈추면 버린다 */
    BLE_POSITION_MS = 190, BLE_ATTITUDE_MS = 490,
    POSITION_INTERVAL_US = 20000, ATTITUDE_INTERVAL_US = 100000,
    REQUEST_AFTER_MS = 1500, REQUEST_EVERY_MS = 2000,
    STOP_REPEATS = 5, LAND_REPEATS = 3, REPEAT_MS = 100,
    LINES = 8, LINE_MAX = 150,
    READY_FRAMES = 100,            /* 시작 이만큼 전부터 "준비됨"을 본다 */
};

struct parser {
    uint8_t data[FRAME_MAX];
    uint16_t have, need;
    uint32_t last_ms;
};

struct seg { int16_t forward_cm, right_cm; uint16_t hold_ms; };

/* 대본. 움직이는 줄은 거리만 적고, 걸리는 프레임 수는 속도로 정한다. 멈춰 있는 줄은 시간으로 적는다.
 *   1: 앞으로 2 m -> 5 s -> 원점 -> 5 s -> 오른쪽으로 2 m -> 5 s -> 원점 -> 5 s      (설계서 7.3 의 2, 3단계)
 *   2: 앞으로 4 m -> 5 s -> 원점 -> 5 s                                               (7.2 자: 직선 왕복)
 *   3: 앞으로 3 m -> 10 s -> 원점 -> 5 s                                              (7.2 차: 가다가 정지)
 *   4: 제자리 60 s                                                                    (7.3 의 1, 4단계)
 * 대본의 맨 앞 "제자리 30 s"는 시작 프레임(앵커 1 을 켠 뒤 1500 프레임)으로 대신한다. */
static const struct seg script_1[] = { { 200, 0, 0 }, { 0, 0, 5000 }, { -200, 0, 0 }, { 0, 0, 5000 },
                                       { 0, 200, 0 }, { 0, 0, 5000 }, { 0, -200, 0 }, { 0, 0, 5000 } };
static const struct seg script_2[] = { { 400, 0, 0 }, { 0, 0, 5000 }, { -400, 0, 0 }, { 0, 0, 5000 } };
static const struct seg script_3[] = { { 300, 0, 0 }, { 0, 0, 10000 }, { -300, 0, 0 }, { 0, 0, 5000 } };
static const struct seg script_4[] = { { 0, 0, 60000 } };
static const struct { const struct seg *segs; uint8_t count; } scripts[FCL_SCRIPTS] = {
    { script_1, sizeof(script_1) / sizeof(script_1[0]) }, { script_2, sizeof(script_2) / sizeof(script_2[0]) },
    { script_3, sizeof(script_3) / sizeof(script_3[0]) }, { script_4, sizeof(script_4) / sizeof(script_4[0]) },
};

static struct {
    uint8_t id;
    uint32_t frame_us;
    struct fcl_io io;
    struct parser from_fc, from_ble;
    uint8_t tx_seq;
    /* 비행 제어기가 알려 준 것 */
    uint8_t fc_sysid, fc_compid;
    bool fc_seen, armed, have_position, have_attitude, position_new;
    uint32_t heartbeat_ms, mode, position_ms, attitude_ms, request_position_ms, request_attitude_ms;
    float x, y, z, vx, vy, vz, roll, pitch, yaw;
    uint32_t ble_position_ms, ble_attitude_ms;
    /* 대본 */
    enum fcl_state state;
    bool have_control, latched;
    struct uwbf_control ctl;
    uint32_t frame, control_ms, last_command_frame, log_bad_ms;
    bool log_bad, command_valid;
    float x0, y0;
    int16_t command_vx, command_vy;
    uint16_t row;
    uint8_t stop_left, land_left;
    uint32_t repeat_ms;
    /* 기록할 줄 */
    char lines[LINES][LINE_MAX];
    uint8_t line_head, line_tail;
    uint32_t lines_dropped, ble_dropped, fc_dropped, garbage;
    LinkFs_t linkfs;
} g;

/* ------------------------------------------------------------------ 기록할 줄 */
static char *line_slot(void)
{
    if ((uint8_t)(g.line_head - g.line_tail) >= LINES) {
        ++g.lines_dropped;
        return NULL;
    }
    return g.lines[g.line_head++ % LINES];
}

static void event(const char *name, long a, long b)
{
    char *line = line_slot();
    if (line != NULL) {
        snprintf(line, LINE_MAX, "E,%lu,%s,%ld,%ld\r\n", (unsigned long)g.frame, name, a, b);
    }
}

unsigned fcl_take_line(char *out, unsigned size)
{
    if (g.line_head == g.line_tail) {
        return 0;
    }
    const char *line = g.lines[g.line_tail++ % LINES];
    const unsigned n = (unsigned)strlen(line);
    if (n >= size) {
        return 0;
    }
    memcpy(out, line, n);
    return n;
}

enum fcl_state fcl_state_now(void)
{
    return g.state;
}

/* ------------------------------------------------------------------ MAVLink */
static uint16_t crc_add(uint16_t crc, uint8_t byte)
{
    uint8_t t = (uint8_t)(byte ^ (uint8_t)crc);
    t = (uint8_t)(t ^ (t << 4));
    return (uint16_t)((crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ (t >> 4));
}

static uint16_t crc_of(const uint8_t *data, unsigned length, uint8_t extra)
{
    uint16_t crc = 0xFFFF;
    for (unsigned i = 0; i < length; ++i) {
        crc = crc_add(crc, data[i]);
    }
    return crc_add(crc, extra);
}

static void put_float(uint8_t *out, float value)
{
    memcpy(out, &value, 4);   /* 보드와 PC 모두 리틀 엔디언 */
}

static float get_float(const uint8_t *in)
{
    float value;
    memcpy(&value, in, 4);
    return value;
}

/* 보드가 비행 제어기에 보내는 MAVLink 2 메시지 */
static bool send_to_fc(uint32_t message, const uint8_t *payload, uint8_t length, uint8_t extra)
{
    uint8_t frame[12 + 64];
    if (length > 64) {
        return false;
    }
    frame[0] = MAV_V2;
    frame[1] = length;
    frame[2] = 0;
    frame[3] = 0;
    frame[4] = g.tx_seq++;
    frame[5] = FCL_SYSID;
    frame[6] = FCL_COMPID;
    frame[7] = (uint8_t)message;
    frame[8] = (uint8_t)(message >> 8);
    frame[9] = (uint8_t)(message >> 16);
    memcpy(frame + 10, payload, length);
    const uint16_t crc = crc_of(frame + 1, 9U + length, extra);
    frame[10 + length] = (uint8_t)crc;
    frame[11 + length] = (uint8_t)(crc >> 8);
    if (!g.io.to_fc(frame, 12U + length)) {
        ++g.fc_dropped;
        return false;
    }
    return true;
}

static void send_command(uint16_t command, float p1, float p2)
{
    uint8_t payload[33] = { 0 };
    put_float(payload, p1);
    put_float(payload + 4, p2);
    payload[28] = (uint8_t)command;
    payload[29] = (uint8_t)(command >> 8);
    payload[30] = g.fc_sysid;
    payload[31] = g.fc_compid;
    (void)send_to_fc(MSG_COMMAND_LONG, payload, sizeof(payload), CRC_COMMAND_LONG);
}

/* 기체 기준 속도 명령 (앞 +x, 오른쪽 +y, cm/s). 높이는 그대로, 기수 방향도 그대로 */
static void send_velocity(int16_t vx_cm, int16_t vy_cm, uint32_t now_ms)
{
    uint8_t payload[53] = { 0 };
    payload[0] = (uint8_t)now_ms;
    payload[1] = (uint8_t)(now_ms >> 8);
    payload[2] = (uint8_t)(now_ms >> 16);
    payload[3] = (uint8_t)(now_ms >> 24);
    put_float(payload + 16, (float)vx_cm * 0.01f);
    put_float(payload + 20, (float)vy_cm * 0.01f);
    payload[48] = (uint8_t)TYPE_MASK_VELOCITY;
    payload[49] = (uint8_t)(TYPE_MASK_VELOCITY >> 8);
    payload[50] = g.fc_sysid;
    payload[51] = g.fc_compid;
    payload[52] = MAV_FRAME_BODY_NED;
    (void)send_to_fc(MSG_SET_POSITION_TARGET_LOCAL_NED, payload, sizeof(payload), CRC_SET_POSITION_TARGET);
}

static void send_land(void)
{
    send_command(CMD_DO_SET_MODE, 1.0f, (float)FCL_MODE_LAND); /* n6_fly.py 의 킬과 같은 명령 */
}

/* 비행 제어기에서 온 메시지 하나. 노트북으로 넘길지 돌려준다 */
static bool on_fc_message(const uint8_t *frame, unsigned total, uint32_t now_ms)
{
    const bool v2 = frame[0] == MAV_V2;
    const unsigned head = v2 ? 10U : 6U, length = frame[1];
    const uint32_t message = v2 ? (uint32_t)frame[7] | ((uint32_t)frame[8] << 8) | ((uint32_t)frame[9] << 16) : frame[5];
    const uint8_t sysid = frame[v2 ? 5 : 3], compid = frame[v2 ? 6 : 4];
    uint8_t extra;
    (void)total;
    switch (message) {
    case MSG_HEARTBEAT: extra = CRC_HEARTBEAT; break;
    case MSG_ATTITUDE: extra = CRC_ATTITUDE; break;
    case MSG_LOCAL_POSITION_NED: extra = CRC_LOCAL_POSITION_NED; break;
    case MSG_COMMAND_ACK: extra = CRC_COMMAND_ACK; break;
    default: return true;
    }
    const uint16_t crc = crc_of(frame + 1, head - 1U + length, extra);
    if (frame[head + length] != (uint8_t)crc || frame[head + length + 1U] != (uint8_t)(crc >> 8)) {
        return true; /* 깨진 것은 읽지 않고 그대로 넘긴다(받는 쪽이 버린다) */
    }
    uint8_t p[32] = { 0 };   /* MAVLink 2 는 끝의 0 을 잘라 보낸다. 0 으로 채워 읽는다 */
    memcpy(p, frame + head, length < sizeof(p) ? length : sizeof(p));
    if (message == MSG_HEARTBEAT) {
        if (compid == MAV_COMP_AUTOPILOT && p[4] != MAV_TYPE_GCS) {
            if (!g.fc_seen) {
                g.fc_seen = true;
                event("fc_seen", sysid, p[4]);
            }
            g.fc_sysid = sysid;
            g.fc_compid = compid;
            g.heartbeat_ms = now_ms;
            g.mode = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
            g.armed = (p[6] & MAV_MODE_FLAG_ARMED) != 0;
        }
        return true;
    }
    if (message == MSG_LOCAL_POSITION_NED) {
        g.x = get_float(p + 4);
        g.y = get_float(p + 8);
        g.z = get_float(p + 12);
        g.vx = get_float(p + 16);
        g.vy = get_float(p + 20);
        g.vz = get_float(p + 24);
        g.have_position = g.position_new = true;
        g.position_ms = now_ms;
        if ((uint32_t)(now_ms - g.ble_position_ms) < BLE_POSITION_MS) {
            return false;
        }
        g.ble_position_ms = now_ms;
        return true;
    }
    if (message == MSG_ATTITUDE) {
        g.roll = get_float(p + 4);
        g.pitch = get_float(p + 8);
        g.yaw = get_float(p + 12);
        g.have_attitude = true;
        g.attitude_ms = now_ms;
        if ((uint32_t)(now_ms - g.ble_attitude_ms) < BLE_ATTITUDE_MS) {
            return false;
        }
        g.ble_attitude_ms = now_ms;
        return true;
    }
    /* COMMAND_ACK: 보드가 보낸 명령에 대한 답은 노트북에 넘기지 않는다 */
    return !(length >= 9U && p[8] == FCL_SYSID);
}

/* 바이트를 모아 메시지 하나가 되면 참. 그때 p->data 에 p->need 바이트가 있다 */
static bool feed(struct parser *p, uint8_t byte, uint32_t now_ms)
{
    if (p->have != 0 && (uint32_t)(now_ms - p->last_ms) > STALL_MS) {
        p->have = 0;
        ++g.garbage;
    }
    p->last_ms = now_ms;
    if (p->have == 0) {
        if (byte != MAV_V1 && byte != MAV_V2) {
            ++g.garbage;
            return false;
        }
        p->need = 0;
    }
    p->data[p->have++] = byte;
    if (p->data[0] == MAV_V1 && p->have == 2) {
        p->need = (uint16_t)(8U + p->data[1]);
    } else if (p->data[0] == MAV_V2 && p->have == 3) {
        p->need = (uint16_t)(12U + p->data[1] + ((p->data[2] & 1U) ? 13U : 0U));
    }
    if (p->need != 0 && p->have == p->need) {
        p->have = 0;
        return true;
    }
    return false;
}

void fcl_from_fc(const uint8_t *data, unsigned length, uint32_t now_ms)
{
    for (unsigned i = 0; i < length; ++i) {
        LinkFs_OnFcByte(&g.linkfs, data[i], now_ms);
        if (feed(&g.from_fc, data[i], now_ms) && on_fc_message(g.from_fc.data, g.from_fc.need, now_ms) &&
            !g.io.to_ble(g.from_fc.data, g.from_fc.need)) {
            ++g.ble_dropped;
        }
    }
}

void fcl_from_ble(const uint8_t *data, unsigned length, uint32_t now_ms)
{
    for (unsigned i = 0; i < length; ++i) {
        LinkFs_OnGcsByte(&g.linkfs, data[i], now_ms);
        if (feed(&g.from_ble, data[i], now_ms) && !g.io.to_fc(g.from_ble.data, g.from_ble.need)) {
            ++g.fc_dropped;
        }
    }
}

/* ------------------------------------------------------------------ 대본 */
static uint32_t seg_frames(const struct seg *s, uint8_t speed_cm, uint32_t frame_us)
{
    if (s->forward_cm == 0 && s->right_cm == 0) {
        return (uint32_t)(((uint64_t)s->hold_ms * 1000U) / frame_us);
    }
    const uint32_t distance = (uint32_t)(s->forward_cm < 0 ? -s->forward_cm : s->forward_cm) +
                              (uint32_t)(s->right_cm < 0 ? -s->right_cm : s->right_cm);
    /* 거리 / 속도를 프레임 수로 (올림) */
    return (uint32_t)(((uint64_t)distance * 1000000U + (uint64_t)speed_cm * frame_us - 1U) / ((uint64_t)speed_cm * frame_us));
}

bool fcl_script_velocity(uint8_t script, uint8_t speed_cm, uint32_t frame_us, uint32_t step, int16_t *vx_cm,
                         int16_t *vy_cm, uint16_t *row)
{
    *vx_cm = *vy_cm = 0;
    *row = 0;
    if (script < 1 || script > FCL_SCRIPTS || speed_cm == 0 || frame_us == 0) {
        return false;
    }
    const struct seg *segs = scripts[script - 1].segs;
    for (unsigned i = 0; i < scripts[script - 1].count; ++i) {
        const uint32_t frames = seg_frames(&segs[i], speed_cm, frame_us);
        if (step < frames) {
            *row = (uint16_t)i;
            *vx_cm = (int16_t)(segs[i].forward_cm > 0 ? speed_cm : segs[i].forward_cm < 0 ? -(int16_t)speed_cm : 0);
            *vy_cm = (int16_t)(segs[i].right_cm > 0 ? speed_cm : segs[i].right_cm < 0 ? -(int16_t)speed_cm : 0);
            return true;
        }
        step -= frames;
    }
    *row = scripts[script - 1].count;
    return false;
}

uint32_t fcl_script_extent_cm(uint8_t script)
{
    if (script < 1 || script > FCL_SCRIPTS) {
        return 0;
    }
    int32_t forward = 0, right = 0, extent = 0;
    for (unsigned i = 0; i < scripts[script - 1].count; ++i) {
        forward += scripts[script - 1].segs[i].forward_cm;
        right += scripts[script - 1].segs[i].right_cm;
        const int32_t a = forward < 0 ? -forward : forward, b = right < 0 ? -right : right;
        extent = a > extent ? a : extent;
        extent = b > extent ? b : extent;
    }
    return (uint32_t)extent;
}

/* 그만둔다. hold: 속도 0 을 몇 번 보낸다. land: 착륙 명령을 몇 번 보낸다. 둘 다 아니면 아무것도 보내지 않는다 */
static void stop(const char *why, long a, long b, bool hold, bool land, uint32_t now_ms)
{
    event(why, a, b);
    g.state = FCL_STOPPED;
    g.latched = true;
    g.command_vx = g.command_vy = 0;
    g.stop_left = hold ? STOP_REPEATS : 0;
    g.land_left = land ? LAND_REPEATS : 0;
    g.repeat_ms = now_ms - REPEAT_MS;   /* 첫 번째는 다음 tick 에서 바로 */
    if (land) {
        event("land_sent", 0, 0);
    }
}

static bool same_script(const struct uwbf_control *a, const struct uwbf_control *b)
{
    return a->script == b->script && a->command == b->command && a->start == b->start && a->mover == b->mover &&
           a->speed == b->speed && a->limit == b->limit;
}

static void emit_state_line(bool sent, uint32_t now_ms)
{
    char *line = line_slot();
    if (line == NULL) {
        return;
    }
    int n = snprintf(line, LINE_MAX, "C,%lu,", (unsigned long)g.frame);
    if (g.have_position) {
        n += snprintf(line + n, (size_t)(LINE_MAX - n), "%ld,%ld,%ld,%ld,%ld,%ld,", (long)(g.x * 1000.0f),
                      (long)(g.y * 1000.0f), (long)(g.z * 1000.0f), (long)(g.vx * 1000.0f), (long)(g.vy * 1000.0f),
                      (long)(g.vz * 1000.0f));
    } else {
        n += snprintf(line + n, (size_t)(LINE_MAX - n), ",,,,,,");
    }
    if (g.have_attitude) {
        n += snprintf(line + n, (size_t)(LINE_MAX - n), "%ld,%ld,%ld,", (long)(g.roll * 5729.578f),
                      (long)(g.pitch * 5729.578f), (long)(g.yaw * 5729.578f));
    } else {
        n += snprintf(line + n, (size_t)(LINE_MAX - n), ",,,");
    }
    snprintf(line + n, (size_t)(LINE_MAX - n), "%ld,%ld,%lu,%u,%u,%u,%d,%d,%u\r\n",
             g.have_position ? (long)(uint32_t)(now_ms - g.position_ms) : -1L,
             g.have_attitude ? (long)(uint32_t)(now_ms - g.attitude_ms) : -1L, (unsigned long)g.mode,
             (unsigned)g.armed, (unsigned)g.state, (unsigned)g.row, 10 * (int)g.command_vx, 10 * (int)g.command_vy,
             (unsigned)sent);
}

void fcl_control(uint32_t frame, const struct uwbf_control *ctl, bool log_ok, uint32_t now_ms)
{
    bool sent = false;
    const bool hb_ok = g.fc_seen && (uint32_t)(now_ms - g.heartbeat_ms) < FCL_HEARTBEAT_FRESH_MS;
    const bool pos_ok = g.have_position && (uint32_t)(now_ms - g.position_ms) < FCL_POSITION_FRESH_MS;
    const bool guided = hb_ok && g.armed && g.mode == FCL_MODE_GUIDED;

    /* 기록이 되는지: 안 되는 상태가 이어진 시간을 잰다 */
    if (log_ok) {
        g.log_bad = false;
    } else if (!g.log_bad) {
        g.log_bad = true;
        g.log_bad_ms = now_ms;
    }
    const bool log_dead = g.log_bad && (uint32_t)(now_ms - g.log_bad_ms) > FCL_LOG_TIMEOUT_MS;

    /* 대본이 바뀌었거나 앵커 1 이 다시 켜졌으면(프레임 번호가 뒤로 감) 처음부터 */
    if (!g.have_control || !same_script(ctl, &g.ctl) || frame < g.frame) {
        if (g.state == FCL_RUN) {
            g.frame = frame;
            stop("abort_ctl", ctl->script, (long)ctl->start, true, false, now_ms);
        }
        g.state = FCL_IDLE;
        g.latched = false;
        g.command_valid = false;
        g.ctl = *ctl;
        g.have_control = true;
    }
    g.frame = frame;
    g.control_ms = now_ms;

    const bool active = ctl->command == UWBF_CMD_RUN && ctl->script >= 1 && ctl->script <= FCL_SCRIPTS &&
                        ctl->speed != 0;
    const uint8_t limit = ctl->limit < FCL_HARD_LIMIT_CM ? ctl->limit : FCL_HARD_LIMIT_CM;
    const uint8_t speed = ctl->speed < limit ? ctl->speed : limit;

    if (!active) {
        if (g.state == FCL_RUN) {
            stop("abort_ctl", ctl->script, ctl->command, true, false, now_ms);
        } else if (!g.latched) {
            g.state = FCL_IDLE;
        }
    } else if (g.state == FCL_IDLE || g.state == FCL_READY) {
        if (frame < ctl->start) {
            g.state = (frame + READY_FRAMES >= ctl->start && guided && pos_ok && log_ok) ? FCL_READY : FCL_IDLE;
        } else if (frame - ctl->start > FCL_START_WINDOW_FRAMES || !guided || !pos_ok || !log_ok || speed == 0) {
            /* 늦게 왔거나 준비가 안 됐다. 이 대본에는 끼지 않는다 */
            g.state = FCL_SKIPPED;
            g.latched = true;
            event("skip_notready", (long)(frame - ctl->start),
                  (guided ? 0 : 1) | (pos_ok ? 0 : 2) | (log_ok ? 0 : 4) | (speed != 0 ? 0 : 8));
        } else {
            g.state = FCL_RUN;
            g.x0 = g.x;
            g.y0 = g.y;
            g.command_valid = false;
            event("start", ctl->script, speed);
        }
    }

    if (g.state == FCL_RUN) {
        const float zone_m = (float)(fcl_script_extent_cm(ctl->script) + FCL_ZONE_MARGIN_CM) * 0.01f;
        const float dx = g.x - g.x0, dy = g.y - g.y0;
        int16_t vx = 0, vy = 0;
        uint16_t row = 0;
        if (!hb_ok) {
            stop("abort_stale", 1, (long)(uint32_t)(now_ms - g.heartbeat_ms), true, false, now_ms);
        } else if (!g.armed) {
            stop("abort_disarm", 0, 0, false, false, now_ms);
        } else if (g.mode != FCL_MODE_GUIDED) {
            stop("abort_mode", (long)g.mode, 0, false, false, now_ms);   /* 조종기나 노트북이 가져갔다 */
        } else if (!pos_ok) {
            stop("abort_stale", 2, (long)(uint32_t)(now_ms - g.position_ms), true, false, now_ms);
        } else if (dx > zone_m || dx < -zone_m || dy > zone_m || dy < -zone_m) {
            stop("abort_zone", (long)(dx * 1000.0f), (long)(dy * 1000.0f), false, true, now_ms);
        } else if (log_dead) {
            stop("abort_log", (long)(uint32_t)(now_ms - g.log_bad_ms), 0, false, true, now_ms);
        } else if (!fcl_script_velocity(ctl->script, speed, g.frame_us, frame - ctl->start, &vx, &vy, &row)) {
            g.row = row;
            g.state = FCL_DONE;
            g.latched = true;
            g.command_vx = g.command_vy = 0;
            g.stop_left = STOP_REPEATS;
            g.repeat_ms = now_ms - REPEAT_MS;
            event("done", ctl->script, (long)(frame - ctl->start));
        } else {
            if (ctl->mover != UWBF_MOVER_ALL && ctl->mover != g.id) {
                vx = vy = 0;     /* 이번 대본에서 움직이는 기체가 아니다: 제자리 */
            }
            g.row = row;
            if (!g.command_valid || vx != g.command_vx || vy != g.command_vy ||
                frame - g.last_command_frame >= FCL_COMMAND_EVERY_FRAMES) {
                g.command_vx = vx;
                g.command_vy = vy;
                g.command_valid = true;
                g.last_command_frame = frame;
                send_velocity(vx, vy, now_ms);
                sent = true;
            }
        }
    }
    if (sent || g.position_new) {
        g.position_new = false;
        emit_state_line(sent, now_ms);
    }
}

void fcl_tick(uint32_t now_ms)
{
    /* 노트북 하트비트가 끊긴 채 LAND + 시동이 이어지면 강제 DISARM (link_failsafe.h, n6_bridge 와 같은 조건) */
    uint8_t cut[LINKFS_FRAME_LEN];
    if (LinkFs_Poll(&g.linkfs, now_ms, cut) != 0U) {
        if (!g.io.to_fc(cut, LINKFS_FRAME_LEN)) {
            ++g.fc_dropped;
        }
        event("linkfs_cut", (long)g.linkfs.cuts, (long)g.linkfs.fc_mode);
    }
    const bool hb_ok = g.fc_seen && (uint32_t)(now_ms - g.heartbeat_ms) < 3000U;
    if (g.fc_seen && !hb_ok) {
        g.fc_seen = false;
        g.have_position = g.have_attitude = false;
        event("fc_lost", 0, 0);
    }
    /* 위치와 자세가 안 오면 다시 요청한다 (노트북이 스트림을 껐다 켜면 요청이 지워진다) */
    if (hb_ok) {
        if ((!g.have_position || (uint32_t)(now_ms - g.position_ms) > REQUEST_AFTER_MS) &&
            (uint32_t)(now_ms - g.request_position_ms) > REQUEST_EVERY_MS) {
            g.request_position_ms = now_ms;
            send_command(CMD_SET_MESSAGE_INTERVAL, (float)MSG_LOCAL_POSITION_NED, (float)POSITION_INTERVAL_US);
        }
        if ((!g.have_attitude || (uint32_t)(now_ms - g.attitude_ms) > REQUEST_AFTER_MS) &&
            (uint32_t)(now_ms - g.request_attitude_ms) > REQUEST_EVERY_MS) {
            g.request_attitude_ms = now_ms;
            send_command(CMD_SET_MESSAGE_INTERVAL, (float)MSG_ATTITUDE, (float)ATTITUDE_INTERVAL_US);
        }
    }
    /* 앵커 1 의 Poll 이 끊겼다: 속도 0 을 보내고 그만둔다 */
    if (g.state == FCL_RUN && (uint32_t)(now_ms - g.control_ms) > FCL_LINK_TIMEOUT_MS) {
        stop("abort_link", (long)(uint32_t)(now_ms - g.control_ms), 0, true, false, now_ms);
    }
    if ((g.stop_left != 0 || g.land_left != 0) && (uint32_t)(now_ms - g.repeat_ms) >= REPEAT_MS) {
        g.repeat_ms = now_ms;
        if (g.land_left != 0) {
            --g.land_left;
            send_land();
        } else {
            --g.stop_left;
            /* 그사이 조종기가 가져갔으면 보내지 않는다 */
            if (g.mode == FCL_MODE_GUIDED) {
                send_velocity(0, 0, now_ms);
            }
        }
    }
}

void fcl_init(uint8_t tag_id, uint32_t frame_us, const struct fcl_io *io)
{
    memset(&g, 0, sizeof(g));
    g.id = tag_id;
    g.frame_us = frame_us;
    g.io = *io;
    g.fc_sysid = 1;
    g.fc_compid = MAV_COMP_AUTOPILOT;
    g.mode = 0xFFFFFFFFUL;
    /* 첫 요청은 HEARTBEAT 를 보자마자 나가게 */
    g.request_position_ms = g.request_attitude_ms = 0U - (uint32_t)REQUEST_EVERY_MS - 1U;
}
