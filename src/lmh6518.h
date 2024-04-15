/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Driver for the LMH6518 Variable Gain Amplifier as used
 * in the Thunderscope AFE.  Datasheet reference at 
 * https://www.ti.com/lit/ds/symlink/lmh6518.pdf
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _LMH6518_H_
#define _LMH6518_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

#define LMH6518_MAX_BW_FILT_MHZ     (900)
#define LMH6518_MIN_BW_FILT_MHZ     (20)

#define LMH6518_MAX_GAIN_mDB        (38840)
#define LMH6518_MIN_GAIN_mDB        (-1160)

#define LMH6518_CONFIG_INIT         {.atten = 10}

typedef enum lmh6518PowerMode_e {
    PM_FULL_POWER = 0,
    PM_AUX_HIZ = 1
} lmh6518PowerMode_t;

typedef enum lmh6518Preamp_e {
    PREAMP_LG = 0,
    PREAMP_HG = 1
} lmh6518Preamp_t;

typedef struct lmh6518Config_s {
    lmh6518PowerMode_t pm;
    lmh6518Preamp_t preamp;
    uint8_t filter;
    uint8_t atten;
} lmh6518Config_t;

/**
 * @brief Calculate the amplifier settings to achieve the closest
 *          gain to the requested value
 * 
 * @param conf Pointer to the configuration structure
 * @param gain_mdB Requested Gain value in milli-dB
 * @return int32_t Actual gain calculated. Zero indicates error
 */
int32_t lmh6518_calc_gain_config(lmh6518Config_t* conf, int32_t gain_mdB);

/**
 * @brief Set the bandwidth filter in the amplifier
 * 
 * @param conf Pointer to the configuration structure
 * @param bw_MHz Requested bandwidth in MHz
 * @return uint32_t Actual Bandwidth setting in MHz. Zero indicates error
 */
uint32_t lmh6518_set_bandwidth_filter(lmh6518Config_t* conf, uint32_t bw_MHz);

/**
 * @brief Apply the configuration to an amplifier
 * 
 * @param dev SPI device structure for the amplifier to be set
 * @param conf Configuration to be applied
 * @return int32_t Status of the operation, 0 for success, 
 *                  negative value otherwise.
 */
int32_t lmh6518_apply_config(spi_dev_t dev, lmh6518Config_t conf);

#ifdef __cplusplus
}
#endif
#endif