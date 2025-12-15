/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Control the ADC in the Thunderscope LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _ADC_H_
#define _ADC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ts_common.h"
#include "ts_calibration.h"
#include "hmcad15xx.h"
#include "platform.h"
#include "spi.h"

typedef struct ts_adc_s
{
    file_t ctrl;
    hmcad15xxADC_t adcDev;
    hmcad15xxChCfg_t tsChannels[TS_NUM_CHANNELS];
} ts_adc_t;


/**
 * @brief Initialize the Thunderscope ADC
 * 
 * @param adc Pointer to a ADC instance
 * @param spi SPI device for the ADC
 * @param fd Control file handle
 * @return int32_t TS_STATUS_OK if initialized successfully
*/
int32_t ts_adc_init(ts_adc_t* adc, spi_dev_t spi, file_t fd);

/**
 * @brief Configure the ADC Channel Input
 *
 * @param adc Pointer to a ADC instance
 * @param channel Thunderscope Channel number to configure
 * @param input Input Channel of the ADC associated to the Thunderscope Channel
 * @param invert Flag to indicate the ADC input is inverted
 * @return int32_t TS_STATUS_OK if channel is configured successfully
*/
int32_t ts_adc_set_channel_conf(ts_adc_t* adc, uint8_t channel, uint8_t input, uint8_t invert);

/**
 * @brief Set the Gain parameters for a Thunderscope Channel
 * 
 * @param adc Pointer to a ADC instance
 * @param channel Thunderscope Channel number to configure
 * @param gainCoarse Coarse Gain parameter for the ADC Input
 * @return int32_t TS_STATUS_OK if the gain was set successfully
*/
int32_t ts_adc_set_gain(ts_adc_t* adc, uint8_t channel, int32_t gainCoarse);

/**
 * @brief Enable or Disable a Thunderscope Channel ADC
 * 
 * @param adc Pointer to a ADC instance
 * @param channel Thunderscope Channel number to configure
 * @param enable Flag to enable or disable the Thunderscope Channel
 * @return int32_t TS_STATUS_OK if the channel was set successfully
*/
int32_t ts_adc_channel_enable(ts_adc_t* adc, uint8_t channel, uint8_t enable);

/**
 * @brief Update the ADC configuration with the current active channels
 * 
 * @param adc Pointer to a ADC instance
 * @return int32_t TS_STATUS_OK if the channels were updated successfully
 */
int32_t ts_adc_update_channels(ts_adc_t* adc);

/**
 * @brief Shutdown the Thunderscope ADC
 * 
 * @param adc Pointer to a ADC instance
 * @return int32_t TS_STATUS_OK if the ADC was shutdown successfully
*/
int32_t ts_adc_shutdown(ts_adc_t* adc);

/**
 * @brief Enable/Disable the ADC data triggering
 * 
 * @param adc Pointer to a ADC instance
 * @param en 1 to enable the ADC, 0 to disable.
 * @return int32_t TS_STATUS_OK if the enable flag was applied successfully
 */
int32_t ts_adc_run(ts_adc_t* adc, uint8_t en);

/**
 * @brief Set the expected Sample Rate and Resolution for the ADC
 * 
 * @param adc Pointer to a ADC instance
 * @param sample_rate Rate of the ADC Sample clock (Hz)
 * @param mode Resolution mode setting for the ADC.
 * @return int32_t TS_STATUS_OK if the mode was applied successfully
 */
int32_t ts_adc_set_sample_mode(ts_adc_t* adc, uint32_t sample_rate, tsSampleFormat_t mode);

/**
 * @brief Set the calibration on the ADC
 * 
 * @param adc Pointer to a ADC instance
 * @param cal Pointer to a ADC Calibration structure
 * @return int32_t TS_STATUS_OK if the calibration was applied successfully
 */
int32_t ts_adc_cal_set(ts_adc_t* adc, tsAdcCalibration_t *cal);

/**
 * @brief Set the calibration on the ADC
 * 
 * @param adc Pointer to a ADC instance
 * @param cal Pointer to a ADC Calibration structure
 * @return int32_t TS_STATUS_OK if the calibration was applied successfully
 */
int32_t ts_adc_cal_get(ts_adc_t* adc, tsAdcCalibration_t *cal);

#ifdef __cplusplus
}
#endif
#endif