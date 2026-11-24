/*
 * 무선 설정 — 태그(NUCLEO-N657 + DWM3000EVB, ds_twr_responder)와 반드시 같아야 한다.
 * 값은 핸드오프 프로젝트 FSBL/Core/UWB/Src/config_options.c 의 CONFIG_OPTION_35 그대로이고,
 * 태그 빌드(TEST_DS_TWR_RESPONDER)도 이 옵션을 고른다.
 * 채널 5, 프리앰블 128, PAC 8, 코드 9, SFD type 1, 6.8 Mb/s, 표준 PHR, STS 끔.
 */
#include "deca_device_api.h"
#include "radio_config.h"

#ifndef CDK_POWER_INDEX
#define CDK_POWER_INDEX 27
#endif

dwt_config_t config_options = {
    5,                 /* Channel number. */
    DWT_PLEN_128,      /* Preamble length. Used in TX only. */
    DWT_PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,                 /* TX preamble code. Used in TX only. */
    9,                 /* RX preamble code. Used in RX only. */
    1,                 /* non-standard 8 symbol SFD (SFD type 1) */
    DWT_BR_6M8,        /* Data rate. */
    DWT_PHRMODE_STD,   /* PHY header mode. */
    DWT_PHRRATE_STD,   /* PHY header rate. */
    (128 + 1 + 8 - 8), /* SFD timeout */
    DWT_STS_MODE_OFF,  /* STS off */
    DWT_STS_LEN_64,
    DWT_PDOA_M0        /* PDOA off */
};

dwt_txconfig_t txconfig_options = {
    0x34,       /* PG delay. */
    0xfdfdfdfd, /* TX power (아래 radio_apply_tx_power 에서 보드용 값으로 바뀜) */
    0x0         /* PG count */
};

/*
 * drone-hw-bench qorvo_1to1/port.c 의 __wrap_dwt_configuretxrf 와 같은 처리.
 * 참조 전력 인덱스(기본 27 = 0x1B, 팀 Task 1 LISTCAL 기록)를 dwt_calculate_linear_tx_power 로
 * TX_POWER 레지스터 값과 PLL bias 로 바꾼다. 인덱스를 레지스터에 바로 쓰면 태그의 Poll 수신 오류가
 * 잦았다고 1대1 시험 기록에 있다.
 */
int radio_apply_tx_power(struct radio_power_info *info)
{
    power_indexes_t indexes = { .input = { CDK_POWER_INDEX, CDK_POWER_INDEX, CDK_POWER_INDEX, CDK_POWER_INDEX } };
    tx_adj_res_t result = { 0 };
    if (dwt_calculate_linear_tx_power(5, &indexes, &result) != DWT_SUCCESS) {
        return DWT_ERROR;
    }
    dwt_txconfig_t adjusted = txconfig_options;
    adjusted.power = result.tx_frame_cfg.tx_power_setting;
    dwt_configuretxrf(&adjusted);
    dwt_setpllbiastrim(result.tx_frame_cfg.pll_bias);
    if (info != NULL) {
        info->power_index = CDK_POWER_INDEX;
        info->tx_power = adjusted.power;
        info->pll_bias = result.tx_frame_cfg.pll_bias;
    }
    return DWT_SUCCESS;
}
