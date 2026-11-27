/* 비행 제어기 연결 (보드와 상관없는 부분): MAVLink 중계, 대본, 안전.
 *
 *   노트북 --BLE--> XIAO --USART2--> [이 보드] --UART7--> 비행 제어기(ArduCopter, SERIAL4 MAVLink2)
 *
 * 하는 일
 *   1) 중계: 노트북에서 온 MAVLink 메시지는 통째로 비행 제어기에, 비행 제어기에서 온 것은 통째로 노트북에 넘긴다.
 *      (n6_bridge 는 바이트를 그대로 넘겼다. 여기서는 보드가 자기 명령을 끼워 넣어야 해서 메시지 단위로 넘긴다.)
 *      보드가 요청한 빠른 값(위치 50 Hz, 자세 10 Hz)은 BLE 가 따라오지 못하므로 노트북 쪽으로는 5 Hz, 2 Hz 로 줄인다.
 *   2) 읽기: 비행 제어기의 HEARTBEAT(모드, 시동), LOCAL_POSITION_NED(위치, 속도), ATTITUDE(기울기, 기수 방향).
 *   3) 대본: 앵커 1 의 Poll 에 실려 온 대본 번호, 시작 프레임, 속도에 따라 프레임 번호로 속도 명령을 정해
 *      SET_POSITION_TARGET_LOCAL_NED(기체 기준 속도, 기수 방향 유지)로 보낸다. 10 Hz, 값이 바뀌면 바로.
 *   4) 안전
 *      - 시작 프레임에 "시동 + GUIDED + 위치 값이 새것 + 기록이 되는 중"이 아니면 그 대본에 끼지 않는다.
 *      - 도는 중에 모드가 GUIDED 가 아니게 되면(조종기 스위치, 노트북의 착륙 명령) 바로 그만둔다. 아무것도 더 보내지 않는다.
 *      - 시작한 자리에서 정해 둔 거리보다 멀어지면 착륙 명령(LAND 모드)을 보낸다.
 *      - 기록이 2초 넘게 안 되면 착륙 명령을 보낸다.
 *      - 앵커 1 의 Poll 이 0.5초 끊기면 속도 0 을 보내고 그만둔다.
 *      - 속도는 Poll 의 한도와 FCL_HARD_LIMIT_CM 가운데 작은 값을 넘지 못한다.
 *      한 번 그만두면 앵커 1 을 다시 켜기(프레임 번호가 처음부터) 전에는 다시 시작하지 않는다.
 *      비행 제어기 쪽: 명령이 끊기면 제자리에 멈추게 GUID_TIMEOUT 을 0.5 로 둔다(파라미터, 사람이 설정).
 *
 * 이 파일은 보드 없이 PC 에서도 컴파일된다(tag/test_fcl.py 가 그렇게 시험한다).
 */
#ifndef UWB_FCL_H
#define UWB_FCL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "frames.h"

enum {
    FCL_SYSID = 251, FCL_COMPID = 191, /* 보드가 보내는 MAVLink 의 번호 (노트북은 250) */
    FCL_MODE_GUIDED = 4, FCL_MODE_LAND = 9,
    FCL_HARD_LIMIT_CM = 100,           /* 대본 속도의 절대 한도 cm/s */
    FCL_ZONE_MARGIN_CM = 150,          /* 대본이 가는 거리에 이만큼 더한 것이 구역 */
    FCL_START_WINDOW_FRAMES = 25,      /* 시작 프레임 뒤 이 안에 첫 계산이 와야 시작한다 */
    FCL_LINK_TIMEOUT_MS = 500, FCL_LOG_TIMEOUT_MS = 2000,
    FCL_HEARTBEAT_FRESH_MS = 1500, FCL_POSITION_FRESH_MS = 500,
    FCL_COMMAND_EVERY_FRAMES = 5,      /* 값이 같아도 이만큼마다 다시 보낸다 (20 ms 프레임이면 10 Hz) */
    FCL_SCRIPTS = 4,
};

/* 대본 상태 (C 줄에 나온다) */
enum fcl_state { FCL_IDLE = 0, FCL_READY = 1, FCL_RUN = 2, FCL_DONE = 3, FCL_STOPPED = 4, FCL_SKIPPED = 5 };

struct fcl_io {
    /* 메시지 하나를 통째로 넘긴다. 자리가 없어 못 넣으면 false (그 메시지는 버려진다) */
    bool (*to_fc)(const uint8_t *data, unsigned length);
    bool (*to_ble)(const uint8_t *data, unsigned length);
};

void fcl_init(uint8_t tag_id, uint32_t frame_us, const struct fcl_io *io);
void fcl_from_fc(const uint8_t *data, unsigned length, uint32_t now_ms);
void fcl_from_ble(const uint8_t *data, unsigned length, uint32_t now_ms);

/* 프레임마다 한 번: 앵커 1 의 Poll 을 받은 프레임의 조용한 구간에서 부른다 (제어 계산) */
void fcl_control(uint32_t frame, const struct uwbf_control *ctl, bool log_ok, uint32_t now_ms);

/* 자주 부른다(프레임이 없어도): 시간 제한, 값 요청, 되풀이 보내기 */
void fcl_tick(uint32_t now_ms);

/* 기록할 줄(C, E)을 하나 꺼낸다. 없으면 0 */
unsigned fcl_take_line(char *out, unsigned size);

enum fcl_state fcl_state_now(void);

/* 대본: 시작한 뒤 step 번째 프레임의 속도(cm/s, 앞 +x, 오른쪽 +y)와 줄 번호. 대본이 끝났으면 false */
bool fcl_script_velocity(uint8_t script, uint8_t speed_cm, uint32_t frame_us, uint32_t step, int16_t *vx_cm,
                         int16_t *vy_cm, uint16_t *row);
uint32_t fcl_script_extent_cm(uint8_t script);   /* 시작한 자리에서 가장 멀리 가는 거리 */

#endif
