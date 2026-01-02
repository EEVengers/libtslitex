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

typedef enum tsCalAdcTest_e
{
    TS_ADC_TEST_DISABLE,
    TS_ADC_TEST_SINGLE,
    TS_ADC_TEST_DUAL,
    TS_ADC_TEST_RAMP,
    TS_ADC_TEST_DESKEW,
    TS_ADC_TEST_SYNC
} tsCalAdcTest_t;

typedef struct tsChannelCalibration_s
{
    int32_t buffer_uV;
    int32_t bias_uV;
    int32_t attenuatorGain1M_mdB;
    int32_t attenuatorGain50_mdB;
    int32_t bufferGain_mdB;
    int32_t trimRheostat_range;
    int32_t preampLowGainError_mdB;
    int32_t preampHighGainError_mdB;
    int32_t preampAttenuatorGain_mdB[11];
    int32_t preampOutputGainError_mdB;
    int32_t preampLowOffset_uV;
    int32_t preampHighOffset_uV;
    int32_t preampInputBias_uA;
} tsChannelCalibration_t;

typedef struct tsAdcCalibration_s
{
    // Fine Gain Branch Adjustment
    uint8_t branchFineGain[8];
} tsAdcCalibration_t;

typedef struct tsChannelCtrl_s
{
    // FE Attenuator
    uint8_t atten;
    // FE Termination
    uint8_t term;
    // DC Coupling
    uint8_t dc_couple;
    // Trim DPOT
    uint8_t dpot;
    // Trim DAC
    uint16_t dac;
    //Preamp Control
    uint8_t pga_high_gain;
    uint8_t pga_atten;
    uint8_t pga_bw;
} tsChannelCtrl_t;

typedef struct tsScopeCalibration_s
{
    tsChannelCalibration_t afeCal[TS_NUM_CHANNELS];
    tsAdcCalibration_t adcCal;
} tsScopeCalibration_t;


/**
 * @brief Set the calibration data for a channel on the Thunderscope device
 * 
 * @param ts Handle to the Thunderscope device
 * @param channel Channel number
 * @param cal Channel Calibration data
 * @return int32_t TS_STATUS_OK if the calibration was accepted
 */
int32_t thunderscopeChanCalibrationSet(tsHandle_t ts, uint32_t channel, tsChannelCalibration_t *cal);

/**
 * @brief Get the calibration data for a channel on the Thunderscope device
 * 
 * @param ts Handle to the Thunderscope device
 * @param channel Channel number
 * @param cal Calibration Data Pointer
 * @return int32_t TS_STATUS_OK if the calibration was retrieved
 */
int32_t thunderscopeChanCalibrationGet(tsHandle_t ts, uint32_t channel, tsChannelCalibration_t *cal);

/**
 * @brief Set the calibration data for the ADC on the Thunderscope device
 * 
 * @param ts Handle to the Thunderscope device
 * @param cal ADC Calibration data
 * @return int32_t TS_STATUS_OK if the calibration was accepted
 */
int32_t thunderscopeAdcCalibrationSet(tsHandle_t ts, tsAdcCalibration_t *cal);

/**
 * @brief Get the calibration data for the ADC on the Thunderscope device
 * 
 * @param ts Handle to the Thunderscope device
 * @param cal ADC Calibration Data Pointer
 * @return int32_t TS_STATUS_OK if the calibration was retrieved
 */
int32_t thunderscopeAdcCalibrationGet(tsHandle_t ts, tsAdcCalibration_t *cal);

/**
 * @brief Manually set parameters for the devices in a channel to aid in calibration.
 * 
 * @param ts Handle to the Thunderscope device
 * @param channel Channel number
 * @param ctrl AFE Control Parameters Pointer
 * @return int32_t TS_STATUS_OK if the parameters were applied
 */
int32_t thunderscopeCalibrationManualCtrl(tsHandle_t ts, uint32_t channel, tsChannelCtrl_t *ctrl);

/**
 * @brief Manually set the ADC test pattern mode.
 * 
 * @param ts Handle to the Thunderscope device
 * @param test_mode Test Pattern (see HMCAD15xx Documentation)
 * @param test_pattern Value used for Single/Dual test modes
 * @return int32_t TS_STATUS_OK if the parameters were applied
 */
int32_t thunderscopeCalibrationAdcTest(tsHandle_t ts, tsCalAdcTest_t test_mode, uint32_t test_pattern);

/******************************************************************************
 * Factory Provisioning API
 * WARNING: May cause data corruption
 ******************************************************************************/
/**
 * @brief Erase the entire Factory data partition
 * 
 * @param ts Handle to the Thunderscope device
 * @param dna Device DNA value for confirmation
 * @return int32_t TS_STATUS_OK if the partition is erased successfully
 */
int32_t thunderscopeFactoryProvisionPrepare(tsHandle_t ts, uint64_t dna);

/**
 * @brief Append an arbitrary data item to the Factory data partition
 * 
 * @param ts Handle to the Thunderscope device
 * @param tag 32-bit Tag to identify the data item
 * @param length Length of the content string
 * @param content ASCII string containing a JSON object
 * @return int32_t TS_STATUS_OK if the object was written successfully
 */
int32_t thunderscopeFactoryProvisionAppendTLV(tsHandle_t ts, const uint32_t tag, uint32_t length, const char* content);

/**
 * @brief Test the Factory data items are valid
 * 
 * @param ts Handle to the Thunderscope device
 * @return int32_t TS_STATUS_OK if all data item checks pass
 */
int32_t thunderscopeFactoryProvisionVerify(tsHandle_t ts);

/**
 * @brief Retrieve an item from the Factory data partition.  If a NULL buffer is provided, only
 * return the length of the item.
 * 
 * @param ts Handle to the Thunderscope device
 * @param tag 32-bit Tag value to lookup
 * @param content_buffer Pointer for a buffer to store the read value string
 * @param item_max_len Maximum size of value string that the provided buffer can hold
 * @return int32_t Length of the item value or a negative error code if the Item could not be read
 */
int32_t thunderscopeFactoryReadItem(tsHandle_t ts, const uint32_t tag, char* content_buffer, uint32_t item_max_len);

#ifdef __cplusplus
}
#endif
#endif