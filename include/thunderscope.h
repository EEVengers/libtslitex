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
 * @param info Pointer to the device info stucture to return the device name and identifiers
 * @return int32_t TS_STATUS_OK if the name is valid, else TS_STATUS_ERROR
 */
int32_t thunderscopeListDevices(uint32_t devIndex, tsDeviceInfo_t *info);

/**
 * @brief Open a new Thunderscope device instance
 * 
 * @param devIdx Device index to open
 * @param skip_init Do not initialize device peripherals.  Useful when doing gateware upgrades
 *                  or other maintenance.
 * @return tsHandle_t Handle to the Thunderscope device
 */
tsHandle_t thunderscopeOpen(uint32_t devIdx, bool skip_init);

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
 * @return int32_t TS_STATUS_OK if the channel configuration was retrieved
 */
int32_t thunderscopeChannelConfigGet(tsHandle_t ts, uint32_t channel, tsChannelParam_t* conf);

/**
 * @brief Set the configuration for a channel on the Thunderscope device
 * 
 * @param ts Handle to the Thunderscope device
 * @param channel Channel number
 * @param conf Reference to the Channel configuration structure
 * @return int32_t TS_STATUS_OK if the channel was configured
 */
int32_t thunderscopeChannelConfigSet(tsHandle_t ts, uint32_t channel, tsChannelParam_t* conf);

/**
 * @brief Set the mode and frequency for the external Reference Clock
 * 
 * @param ts Handle to the Thunderscope device
 * @param mode Set the Clock IN/OUT mode
 * @param refclk_freq Set the input clock frequency if in IN mode, or output frequency if in OUT mode
 * @return int32_t TS_STATUS_OK if the reference clock was configured
 */
int32_t thunderscopeRefClockSet(tsHandle_t ts, tsRefClockMode_t mode, uint32_t refclk_freq);

/**
 * @brief Get the status for the Thunderscope device
 * 
 * @param ts Handle to the Thunderscope device
 * @param conf Reference to the Scope State structure
 * @return int32_t TS_STATUS_OK if the Thunderscope configuration was retrieved
*/
int32_t thunderscopeStatusGet(tsHandle_t ts, tsScopeState_t* conf);

/**
 * @brief Set the sample rate and format for the Thunderscope device
 * 
 * @param ts Handle to the Thunderscope device
 * @param rate Sample Rate to collect (samples per second)
 * @param mode Sample Capture Mode
 * @return int32_t TS_STATUS_OK if the Thunderscope was configured
*/
int32_t thunderscopeSampleModeSet(tsHandle_t ts, uint32_t rate, tsSampleFormat_t mode);

/**
 * @brief Set the approximate rate at which interrupts will fire
 * 
 * This method sets the target frequency for interrupts in the driver.  A higher value will trigger
 * more often, zero is invalid.  The default update rate is 100Hz, or every 10ms.  The driver
 * configuration will be updated as the sample mode is changed.  This setting will take affect the
 * next time the ADC is enabled.
 * 
 * @param ts Handle to the Thunderscope device
 * @param rate Sample Interrupt Rate (interrupts per second)
 * @return int32_t TS_STATUS_OK if the Thunderscope was configured
 */
int32_t thunderscopeSampleInterruptRate(tsHandle_t ts, uint32_t interrupt_rate);

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

/**
 * @brief Read data into a buffer and retrieve sample count
 * 
 * @param ts Handle to the Thunderscope device
 * @param buffer Pointer to a buffer to store data samples in
 * @param len Length of the data buffer available
 * @param count Pointer to store the sample count number of the first sample in the buffer
 * @return int32_t Length of the data read into the buffer, or a negative error code
 */
int32_t thunderscopeReadCount(tsHandle_t ts, uint8_t* buffer, uint32_t len, uint64_t* count);

/**
 * @brief Configure External Sync Interface
 * 
 * @param ts Handle to the Thunderscope device
 * @param mode Set the sync interface to in, out, or disabled
 * @return int32_t TS_STATUS_OK if the mode was set successfully, or a negative error code
 */
int32_t thunderscopeExtSyncConfig(tsHandle_t ts, tsSyncMode_t mode);

/**
 * @brief Assert a Sync Event on the External Sync Interface
 * 
 * @param ts Handle to the Thunderscope device
 * @return int32_t TS_STATUS_OK if the sync event was asserted set successfully, or a negative error code
 */
int32_t thunderscopeEventSyncAssert(tsHandle_t ts);

/**
 * @brief Poll for a Event
 * 
 * Event struct will be populated with TS_EVT_NONE if there is no event available.
 * 
 * @param ts Handle to the Thunderscope device
 * @param evt Pointer to an Event struct to fill
 * @return int32_t TS_STATUS_OK if polled successfully, or a negative error code
 */
int32_t thunderscopeEventGet(tsHandle_t ts, tsEvent_t* evt);

/**
 * @brief Load a new user firmware onto the Thunderscope
 * 
 * @param ts Handle to the Thunderscope device
 * @param buffer Pointer to a buffer containing the new firmware bitstream
 * @param len Length of the data buffer available
 * @return int32_t TS_STATUS_OK if the firmware was updated successfully, or a negative error code
 */
int32_t thunderscopeFwUpdate(tsHandle_t ts, const char* bitstream, uint32_t len);

/**
 * @brief Read user-defined data blob from the Thunderscope
 * 
 * @param ts Handle to the Thunderscope device
 * @param buffer Pointer to a buffer to store the data as read.
 * @param offset Offset to begin reading within the user data section
 * @param readLen Length of the data buffer
 * @return int32_t Length of the data read if successfull, or a negative error code
 */
int32_t thunderscopeUserDataRead(tsHandle_t ts, char* buffer, uint32_t offset, uint32_t readLen);

/**
 * @brief Write a user-defined data blob to the Thunderscope
 * 
 * This function writes from a section of SPI Flash reserved for application-defined data, such as 
 * calibration data, persistent settings, etc.  Because the SPI flash needs to erase on a 4k page,
 * any offset or length not aligned with a 4k boundary will be read to a temporary buffer, modified,
 * and written back.
 * 
 * @param ts Handle to the Thunderscope device
 * @param buffer Pointer to a buffer of data to be stored.
 * @param offset Offset to begin writing within the user data section
 * @param writeLen Length of the data buffer
 * @return int32_t Length of the data written if successfull, or a negative error code
 */
int32_t thunderscopeUserDataWrite(tsHandle_t ts, const char* buffer, uint32_t offset, uint32_t writeLen);

/**
 * @brief Get the current progress of the firmware update
 * 
 * @param ts Handle to the Thunderscope device
 * @param progress Pointer to a variable to store the progress percentage
 * @return int32_t TS_STATUS_OK if the progress was retrieved successfully, or a negative error code
 */
int32_t thunderscopeGetFwProgress(tsHandle_t ts, uint32_t* progress);

#ifdef __cplusplus
}
#endif
#endif