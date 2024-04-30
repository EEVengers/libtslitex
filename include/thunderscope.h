/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Data Control and Management functions for the
 * Thunderscope LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#ifndef _THUNDERSCOPE_H_
#define _THUNDERSCOPE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "ts_common.h"

/**
 * @brief Get the name of a device from a list of available devices
 * 
 * @param devIndex Index to search
 * @param nameBuffer Pointer to the string buffer to return the device name
 * @param bufLen Length of the available space in the name buffer
 * @return int32_t TS_STATUS_OK if the name is valid, else TS_STATUS_ERROR
 */
int32_t thunderscopeListDevices(uint32_t devIndex, char* nameBuffer, uint32_t bufLen);

/**
 * @brief Open a new Thunderscope device instance
 * 
 * @param devName Path to the device
 * @return tsHandle_t Handle to the Thunderscope device
 */
tsHandle_t thunderscopeOpen(char* devName);

/**
 * @brief Close the Thunderscope device
 * 
 * @param ts Handle to the Thunderscope device
 * @return int32_t TS_STATUS_OK if the device was closed
 */
int32_t thunderscopeClose(tsHandle_t ts);

/**
 * @brief Get the current configuration of a channel on the Thunderscope device
 * 
 * @param ts Handle to the Thunderscope device
 * @param channel Channel number
 * @param conf Reference used to return the Channel configuration
 * @return int32_t 
 */
int32_t thunderscopeChannelConfigGet(tsHandle_t ts, uint32_t channel, tsChannelParam_t* conf);

/**
 * @brief Set the configuration for a channel on the Thunderscope device
 * 
 * @param ts Handle to the Thunderscope device
 * @param channel Channel number
 * @param conf Channel configuration structure
 * @return int32_t TS_STATUS_OK if the channel was configured
 */
int32_t thunderscopeChannelConfigSet(tsHandle_t ts, uint32_t channel, tsChannelParam_t conf);

/**
 * @brief Enable or Disable 
 * 
 * @param ts Handle to the Thunderscope device
 * @param enable Flag to enable or disable the ADC data
 * @return int32_t TS_STATUS_OK if the command was applied correctly
 */
int32_t thunderscopeDataEnable(tsHandle_t ts, uint8_t enable);

/**
 * @brief Read data into a buffer
 * 
 * @param ts Handle to the Thunderscope device
 * @param buffer Pointer to a buffer to store data samples in
 * @param len Length of the data buffer available
 * @return int32_t Length of the data read into the buffer, or a negative error code
 */
int32_t thunderscopeRead(tsHandle_t ts, uint8_t* buffer, uint32_t len);

#ifdef __cplusplus
}
#endif
#endif