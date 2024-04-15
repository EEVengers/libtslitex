/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Control the AFE for a channe in the Thunderscope
 * LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _AFE_H_
#define _AFE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"
#include "i2c.h"
#include "gpio.h"

/**
 * @brief Setup an AFE channel with controls and set initial values
 * 
 * @param channel Channel Number
 * @param afe_amp AFE Amplifier SPI device
 * @param trimDac Trim DAC Device
 * @param trimPot Trim digital Pot device
 * @param termination AFE Termination Control GPIO
 * @param attenuator AFE Attenuation Relay GPIO
 * @param coupling AFE AC/DC Coupling Control GPIO
 * @return int32_t TS_STATUS_OK on success, else error
 */
int32_t ts_afe_init_channel(uint8_t channel, spi_dev_t afe_amp, i2c_t trimDac,
            i2c_t trimPot, gpio_t termination, gpio_t attenuator, gpio_t coupling);


/**
 * @brief Set the AFE input gain for a channel
 * 
 * @param channel Channel Number
 * @param gain_mdB Gain in milli-dB
 * @return int32_t Actual gain value set. TS_STATUS_ERROR on error.
 */
int32_t ts_afe_set_gain(uint8_t channel, int32_t gain_mdB);

/**
 * @brief Set the AFE Bandwidth filter for a channel
 * 
 * @param channel Channel Number
 * @param bw_MHz Requested Bandwidth in MHz
 * @return int32_t Actual Bandwidth set, else TS_STATUS_ERROR
 */
int32_t ts_afe_set_bw_filter(uint8_t channel, uint32_t bw_MHz);

/**
 * @brief Enable/Disable the AFE Termination
 * 
 * @param channel Channel Number
 * @param enable 1 to enable, 0 to disable
 * @return int32_t TS_STATUS_OK on success, else error
 */
int32_t ts_afe_termination_control(uint8_t channel, uint8_t enable);

/**
 * @brief Enable/Disable the AFE 10x Attenuation
 * 
 * @param channel Channel Number
 * @param enable 1 to enable, 0 to disable
 * @return int32_t TS_STATUS_OK on success, else error
 */
int32_t ts_afe_attenuation_control(uint8_t channel, uint8_t enable);

/**
 * @brief Enable/Disable the AFE Coupling
 * 
 * @param channel Channel Number
 * @param enable 1 to enable, 0 to disable
 * @return int32_t TS_STATUS_OK on success, else error
 */
int32_t ts_afe_coupling_control(uint8_t channel, uint8_t enable);

#ifdef __cplusplus
}
#endif
#endif