#pragma once
#include <stdint.h>
#include "deca_device_api.h"

extern dwt_config_t config_options;
extern dwt_txconfig_t txconfig_options;

struct radio_power_info {
    uint32_t power_index;
    uint32_t tx_power;
    uint32_t pll_bias;
};

int radio_apply_tx_power(struct radio_power_info *info);
