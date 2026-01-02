/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Library configuration definitions
 *
 * Copyright (C) 2025 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _TS_DATA_H_
#define _TS_DATA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ts_common.h"
#include "ts_calibration.h"
#include "ts_fw_manager.h"


#define TAGSTR(x)   (uint32_t)((x[0] << 24) + (x[1] << 16) + (x[2] << 8) + x[3])
#define TAG_HWID    TAGSTR("HWID")
#define TAG_FCAL    TAGSTR("FCAL")

/**
 * @brief Read and parse the Factory Calibration from flash
 * 
 * @param mngr Pointer to the FW Manager instance
 * @param fcal Pointer to the calibration data struct
 * 
 * @return TS_STATUS_OK if the calibration was parsed successfully
 */
int32_t ts_data_factory_cal_get(ts_fw_manager_t* mngr, tsScopeCalibration_t *fcal);

/**
 * @brief Read and parse the Factory Device Information
 * 
 * @param mngr Pointer to the FW Manager instance
 * @param infos Pointer to the device information structf
 * 
 * @return TS_STATUS_OK if the Device information data was parsed
 */
int32_t ts_data_factory_id_get(ts_fw_manager_t* mngr, tsDeviceInfo_t* infos);


#ifdef __cplusplus
}
#endif

#endif