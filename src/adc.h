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
 * @param gainFine Fine Gain parameter for the ADC Input
 * @return int32_t TS_STATUS_OK if the gain was set successfully
*/
int32_t ts_adc_set_gain(ts_adc_t* adc, uint8_t channel, int32_t gainCoarse, int32_t gainFine);

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

#ifdef __cplusplus
}
#endif
#endif