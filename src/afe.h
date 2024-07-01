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
#include "lmh6518.h"


typedef struct ts_afe_s
{
    uint8_t channel;
    spi_dev_t amp;
    lmh6518Config_t ampConf;
    i2c_t trimDac;
    uint8_t trimDacCh;
    i2c_t trimPot;
    uint8_t trimPotCh;
    gpio_t termPin;
    gpio_t attenuatorPin;
    gpio_t couplingPin;
}ts_afe_t;

/**
 * @brief Setup an AFE channel with controls and set initial values
 * 
 * @param afe   Pointer to an AFE instance
 * @param channel Channel Number
 * @param afe_amp AFE Amplifier SPI device
 * @param trimDac Trim DAC Device
 * @param dacCh Channel on the Trim DAC
 * @param trimPot Trim digital Pot device
 * @param potCh Channel on the Trim DPOT
 * @param termination AFE Termination Control GPIO
 * @param attenuator AFE Attenuation Relay GPIO
 * @param coupling AFE AC/DC Coupling Control GPIO
 * @return int32_t TS_STATUS_OK on success, else error
 */
int32_t ts_afe_init(ts_afe_t* afe, uint8_t channel, spi_dev_t afe_amp, i2c_t trimDac, uint8_t dacCh,
            i2c_t trimPot, uint8_t potCh, gpio_t termination, gpio_t attenuator, gpio_t coupling);


/**
 * @brief Set the AFE input gain for a channel
 * 
 * @param afe   Pointer to an AFE instance
 * @param gain_mdB Gain in milli-dB
 * @return int32_t Actual gain value set. TS_STATUS_ERROR on error.
 */
int32_t ts_afe_set_gain(ts_afe_t* afe, int32_t gain_mdB);

/**
 * @brief Set the AFE input gain for a channel
 * 
 * @param afe   Pointer to an AFE instance
 * @param offset_mV Offset in mV
 * @param offset_mV Applied Offset in mV
 * @return int32_t TS_STATUS_OK on success, else error
 */
int32_t ts_afe_set_offset(ts_afe_t* afe, int32_t offset_mV, int32_t* offset_actual);

/**
 * @brief Set the AFE Bandwidth filter for a channel
 * 
 * @param afe   Pointer to an AFE instance
 * @param bw_MHz Requested Bandwidth in MHz
 * @return int32_t Actual Bandwidth set, else TS_STATUS_ERROR
 */
int32_t ts_afe_set_bw_filter(ts_afe_t* afe, uint32_t bw_MHz);

/**
 * @brief Enable/Disable the AFE Termination

 * @param afe   Pointer to an AFE instance
 * @param enable 1 to enable, 0 to disable
 * @return int32_t TS_STATUS_OK on success, else error
 */
int32_t ts_afe_termination_control(ts_afe_t* afe, uint8_t enable);

/**
 * @brief Enable/Disable the AFE 50x Attenuation
 * 
 * @param afe   Pointer to an AFE instance
 * @param enable 1 to enable, 0 to disable
 * @return int32_t TS_STATUS_OK on success, else error
 */
int32_t ts_afe_attenuation_control(ts_afe_t* afe, uint8_t enable);

/**
 * @brief Enable/Disable the AFE Coupling
 * 
 * @param afe   Pointer to an AFE instance
 * @param enable 1 to enable, 0 to disable
 * @return int32_t TS_STATUS_OK on success, else error
 */
int32_t ts_afe_coupling_control(ts_afe_t* afe, uint8_t enable);

#ifdef __cplusplus
}
#endif
#endif