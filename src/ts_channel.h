/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Channel management functions for the Thunderscope
 * LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _TS_CHANNEL_H_
#define _TS_CHANNEL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ts_common.h"
#include "ts_calibration.h"
#include "liblitepcie.h"
#include "hmcad15xx.h"

typedef void* tsChannelHdl_t;

/**
 * @brief Initialize all Channel AFEs
 * 
 * @param pTsChannels Thunderscope Channel handle
 * @param ts    Thunderscope device handle
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
int32_t ts_channel_init(tsChannelHdl_t* pTsChannels, file_t ts);

/**
 * @brief Tear down and clean up Thunderscope Channel object(s)
 * 
 * @param tsChannels Thunderscope Channel handle
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
int32_t ts_channel_destroy(tsChannelHdl_t tsChannels);

/**
 * @brief Start and Stop the ADC capture
 * 
 * @param tsChannels Thunderscope Channel handle
 * @param en Flag to enable the ADC. 1 to start, 0 to stop.
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
int32_t ts_channel_run(tsChannelHdl_t tsChannels, uint8_t en);

/**
 * @brief Set the operating parameters for a channel
 * 
 * @param tsChannels Thunderscope Channel handle
 * @param chanIdx Channel Index
 * @param param Pointer to the parameter structure to set
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
int32_t ts_channel_params_set(tsChannelHdl_t tsChannels, uint32_t chanIdx, tsChannelParam_t* param);

/**
 * @brief Get the current operating parameters for a channel
 * 
 * @param tsChannels Thunderscope Channel handle
 * @param chanIdx Channel Index
 * @param param Pointer to the parameter structure to return the retrieved values
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
int32_t ts_channel_params_get(tsChannelHdl_t tsChannels, uint32_t chanIdx, tsChannelParam_t* param);

/**
 * @brief Get the current state of the Thunderscope
 * 
 * @param tsChannels Thunderscope Channel handle
 * @return tsScopeState_t Current State information of the Thunderscope device
 */
tsScopeState_t ts_channel_scope_status(tsChannelHdl_t tsChannels);

/**
 * @brief Set the sampling format for the Thunderscope
 * 
 * @param tsChannels Thunderscope Channel handle
 * @param rate Samples per Second
 * @param resolution Number of bits in each sample.  Valid values are 2^8 (256), 2^12 (4096), and 2^14 (16384).
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
int32_t ts_channel_sample_rate_set(tsChannelHdl_t tsChannels, uint32_t rate, uint32_t resolution);

/**
 * @brief Set the calibration parameters for a channel
 * 
 * @param tsChannels Thunderscope Channel handle
 * @param chanIdx Channel Index
 * @param cal Pointer to the calibration structure to apply
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
int32_t ts_channel_calibration_set(tsChannelHdl_t tsChannels, uint32_t chanIdx, tsChannelCalibration_t* cal);

/**
 * @brief Get the calibration parameters for a channel
 * 
 * @param tsChannels Thunderscope Channel handle
 * @param chanIdx Channel Index
 * @param cal Pointer to the calibration structure
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
 int32_t ts_channel_calibration_get(tsChannelHdl_t tsChannels, uint32_t chanIdx, tsChannelCalibration_t* cal);

/** 
 * @brief Manually set the AFE controls for a channel
 * 
 * @param tsChannels Thunderscope Channel handle
 * @param chanIdx Channel Index
 * @param ctrl AFE parameters to apply
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
int32_t ts_channel_calibration_manual(tsChannelHdl_t tsChannels, uint32_t chanIdx, tsChannelCtrl_t ctrl);

/**
 * @brief Set the ADC into a Test Mode
 * 
 * @param tsChannels Thunderscope Channel handle
 * @param mode Test mode enum for the HMCAD15XX
 * @param pattern1 First 16-bit Test Pattern value used in the SINGLE and DUAL test modes
 * @param pattern2 Second 16-bit Test Pattern value used in the DUAL test mode
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
int32_t ts_channel_set_adc_test(tsChannelHdl_t tsChannels, hmcad15xxTestMode_t mode, uint16_t pattern1, uint16_t pattern2);

#ifdef __cplusplus
}
#endif
#endif