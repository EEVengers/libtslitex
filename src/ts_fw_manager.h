/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Provide methods to fetch and store content in flash
 *
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 */
#ifndef _TS_FW_MANAGER_H_
#define _TS_FW_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ts_common.h"
#include "spiflash.h"
#include "platform.h"

#include <stdatomic.h>

typedef struct ts_fw_manager_s {
    spiflash_dev_t flash_dev;
    const flash_layout_t* partition_table;
    _Atomic uint32_t fw_progress;
    uint32_t fw_progress_max;
} ts_fw_manager_t;

/**
 * @brief Initialize the SPIFlash and Partition Table
 * 
 * @param fd File handle for the Thunderscope Instance
 * @param mngr Pointer to a fw_manager struct to be initialized
 * 
 * @return TS_STATUS_OK on success.
 */
int32_t ts_fw_manager_init(file_t fd, ts_fw_manager_t* mngr);

/**
 * @brief Load a new user FW image to flash
 * 
 * @param mngr Pointer to a manager instance
 * @param file_stream Pointer to a buffer containing a bitstream file
 * @param len Length of the bitstream buffer
 * 
 * @return TS_STATUS_OK on success.
 */
int32_t ts_fw_manager_user_fw_update(ts_fw_manager_t* mngr, const char* file_stream, uint32_t len);

/**
 * @brief Get the current progress of the firmware update
 * 
 * @param mngr Pointer to a manager instance
 * @param progress Pointer to a variable to store the progress percentage
 * 
 * @return TS_STATUS_OK on success.
 */
int32_t ts_fw_manager_get_progress(ts_fw_manager_t* mngr, uint32_t* progress);

/**
 * @brief Read the User Calibration data from flash
 * 
 * @param mngr Pointer to a manager instance
 * @param file_stream Pointer to a buffer to store the User Calibration data
 * @param len Available space in the calibration data buffer
 * 
 * @return TS_STATUS_OK on success.
 */
int32_t ts_fw_manager_user_cal_get(ts_fw_manager_t* mngr, const char* file_stream, uint32_t max_len);

/**
 * @brief Load User Calibration data to flash
 * 
 * @param mngr Pointer to a manager instance
 * @param file_stream Pointer to a buffer containing User calibration data
 * @param len Length of the calibration data buffer
 * 
 * @return TS_STATUS_OK on success.
 */
int32_t ts_fw_manager_user_cal_update(ts_fw_manager_t* mngr, const char* file_stream, uint32_t len);

/**
 * @brief Save the Factory calibration data to a file
 *
 * @param mngr Pointer to a manager instance
 * @param file_stream Pointer to a buffer to store the factory calibration
 * @param max_len Available space in the buffer
 * 
 * @return TS_STATUS_OK on success.
 */
int32_t ts_fw_manager_factory_cal_get(ts_fw_manager_t* mngr, const char* file_stream, uint32_t max_len);

/**
 * @brief Reset the user flash partitions with the Factory image
 * 
 * @param mngr Pointer to a manager instance
 * @param reset_config Flag to clear and reinitialize the User configuration partition
 * @param reset_bitstream Flag to reinitialize the user bitstream with the factory image
 * 
 * @return TS_STATUS_OK on success.
 */
int32_t ts_fw_manager_factory_reset(ts_fw_manager_t* mngr, bool reset_config, bool reset_bitstream);


#ifdef __cplusplus
}
#endif

#endif