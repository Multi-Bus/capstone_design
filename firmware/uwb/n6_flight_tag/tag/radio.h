#ifndef UWB_3TO1_RADIO_H
#define UWB_3TO1_RADIO_H

#include <deca_device_api.h>
#include "frames.h"

#define UWB3_RX_EVENTS (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)
#define UWB3_ATTEMPT_EVENTS (UWB3_RX_EVENTS | DWT_INT_TXFRS_BIT_MASK)

static inline uint64_t uwb3_timestamp(bool transmit)
{
    uint8_t bytes[5];
    if (transmit) {
        dwt_readtxtimestamp(bytes);
    } else {
        dwt_readrxtimestamp(bytes, DWT_COMPAT_NONE);
    }
    return ((uint64_t)bytes[4] << 32) | uwb3_get32(bytes);
}

static inline uint16_t uwb3_read(uint8_t frame[UWB3_FINAL_LEN + UWB3_FCS])
{
    uint8_t ranging_flag = 0;
    const uint16_t length = dwt_getframelength(&ranging_flag);
    if (!ranging_flag || length > UWB3_FINAL_LEN + UWB3_FCS) {
        return 0;
    }
    dwt_readrxdata(frame, length, 0);
    return length;
}

static inline int uwb3_send(uint8_t *frame, uint16_t length, uint8_t mode)
{
    if (dwt_writetxdata(length, frame, 0) != DWT_SUCCESS) {
        return DWT_ERROR;
    }
    dwt_writetxfctrl(length + UWB3_FCS, 0, 1);
    return dwt_starttx(mode);
}

struct uwb3_power { int16_t rssi; int16_t first_path; };

static inline struct uwb3_power uwb3_read_power(void)
{
    dwt_cirdiags_t diag;
    struct uwb3_power power = { UWB3_UNKNOWN_POWER, UWB3_UNKNOWN_POWER };
    if (dwt_readdiagnostics_acc(&diag, DWT_ACC_IDX_IP_M) == DWT_SUCCESS) {
        (void)dwt_calculate_rssi(&diag, DWT_ACC_IDX_IP_M, &power.rssi);
        (void)dwt_calculate_first_path_power(&diag, DWT_ACC_IDX_IP_M, &power.first_path);
    }
    return power;
}

/* 프레임 방식: 길이가 여러 가지인 패킷을 읽는다. capacity 는 frame 의 크기. 못 읽으면 0. */
static inline uint16_t uwbf_read(uint8_t *frame, uint16_t capacity)
{
    uint8_t ranging_flag = 0;
    const uint16_t length = dwt_getframelength(&ranging_flag);
    if (!ranging_flag || length < UWB3_FCS || length > capacity) {
        return 0;
    }
    dwt_readrxdata(frame, length, 0);
    return length;
}

/* 프레임 방식 2판: 받은 패킷의 앞부분(capacity 까지)만 읽는다. 긴 방송 조각의 빈 데이터는 읽지 않는다.
 * 받은 길이(FCS 포함)를 돌려주고, 읽은 바이트 수를 have 에 넣는다. 못 읽으면 0. */
static inline uint16_t uwbf_read_head(uint8_t *head, uint16_t capacity, uint16_t *have)
{
    uint8_t ranging_flag = 0;
    const uint16_t length = dwt_getframelength(&ranging_flag);
    *have = 0;
    if (!ranging_flag || length < UWB3_FCS || length > UWBF_MAX_LEN + UWB3_FCS) {
        return 0;
    }
    *have = length < capacity ? length : capacity;
    dwt_readrxdata(head, *have, 0);
    return length;
}

#endif
