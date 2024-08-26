/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Library configuration definitions
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _TS_CAL_H_
#define _TS_CAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "ts_common.h"

typedef struct tsChannelCalibration_s
{
    int32_t buffer_mV;
    int32_t bias_mV;
    int32_t attenuatorGain1M_mdB;
    int32_t attenuatorGain50_mdB;
    int32_t bufferGain_mdB;
    int32_t trimRheostat_range;
    int32_t preampLowGainError_mdB;
    int32_t preampHighGainError_mdB;
    // int32_t preampAttenuatorGain_mdB[11]; // TBD Usage
    int32_t preampOutputGainError_mdB;
    int32_t preampLowOffset_mV;
    int32_t preampHighOffset_mV;
} tsChannelCalibration_t;


typedef struct tsScopeCalibration_s
{
    tsChannelCalibration_t afeCal[TS_NUM_CHANNELS];

} tsScopeCalibration_t;


/**
 * @brief Set the calibration data for a channel on the Thunderscope device
 * 
 * @param ts Handle to the Thunderscope device
 * @param channel Channel number
 * @param cal TBD Calibration data
 * @return int32_t TS_STATUS_OK if the calibration was accepted
*/
int32_t thunderscopeCalibrationSet(tsHandle_t ts, uint32_t channel, tsChannelCalibration_t cal);


#ifdef __cplusplus
}
#endif
#endif