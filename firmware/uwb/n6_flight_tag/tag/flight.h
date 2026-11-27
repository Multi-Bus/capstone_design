/* 비행용 빌드에서만 쓰는 부분: UART 두 개(노트북 쪽 BLE, 비행 제어기)와 fcl.c 를 잇는다. 설명은 flight.c */
#ifndef UWB_FLIGHT_H
#define UWB_FLIGHT_H

#include <stdbool.h>
#include <stdint.h>
#include "frames.h"

void flight_init(uint8_t tag_id, uint32_t frame_us);
/* 조용한 구간에서 자주 부른다. UART 로 들어온 것을 조금씩(한 번에 96바이트까지) 넘긴다. 일을 했으면 참 */
bool flight_service(uint32_t now_us);
/* 프레임마다 한 번(제어 계산): 대본과 안전을 보고 속도 명령을 낸다 */
void flight_control(uint32_t frame, const struct uwbf_control *ctl, bool log_ok);
/* 기록할 줄(C, E, U)을 하나 꺼낸다. 없으면 0 */
unsigned flight_take_line(char *out, unsigned size);

#endif
