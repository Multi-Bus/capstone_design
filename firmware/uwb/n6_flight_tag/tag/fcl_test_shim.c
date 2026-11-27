/* PC 시험용: fcl.c 를 공유 라이브러리로 만들어 tag/test_fcl.py 가 부른다. 보드에는 들어가지 않는다. */
#include <string.h>
#include "fcl.h"

static uint8_t fc_out[65536], ble_out[65536];
static unsigned fc_len, ble_len, room = sizeof(fc_out);

static bool put(uint8_t *buffer, unsigned *have, const uint8_t *data, unsigned length)
{
    if (*have + length > room) {
        return false;
    }
    memcpy(buffer + *have, data, length);
    *have += length;
    return true;
}

static bool to_fc(const uint8_t *data, unsigned length) { return put(fc_out, &fc_len, data, length); }
static bool to_ble(const uint8_t *data, unsigned length) { return put(ble_out, &ble_len, data, length); }

void shim_init(unsigned id, unsigned frame_us)
{
    static const struct fcl_io io = { to_fc, to_ble };
    fc_len = ble_len = 0;
    room = sizeof(fc_out);
    fcl_init((uint8_t)id, frame_us, &io);
}

void shim_room(unsigned bytes) { room = bytes; }

static unsigned take(uint8_t *buffer, unsigned *have, uint8_t *out, unsigned max)
{
    const unsigned n = *have < max ? *have : max;
    memcpy(out, buffer, n);
    memmove(buffer, buffer + n, *have - n);
    *have -= n;
    return n;
}

unsigned shim_take_fc(uint8_t *out, unsigned max) { return take(fc_out, &fc_len, out, max); }
unsigned shim_take_ble(uint8_t *out, unsigned max) { return take(ble_out, &ble_len, out, max); }

void shim_control(unsigned frame, unsigned script, unsigned command, unsigned start, unsigned mover, unsigned speed,
                  unsigned limit, int log_ok, unsigned now_ms)
{
    struct uwbf_control c = uwbf_control_default();
    c.script = (uint8_t)script;
    c.command = (uint8_t)command;
    c.start = start;
    c.mover = (uint8_t)mover;
    c.speed = (uint8_t)speed;
    c.limit = (uint8_t)limit;
    fcl_control(frame, &c, log_ok != 0, now_ms);
}
