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
#include "ts_common.h"
#include "spi.h"
#include "lmh6518.h"
#include "util.h"

#include <stddef.h>

#define LMH6518_CMD_READ        (0x80)
#define LMH6518_CMD_WRITE       (0x00)

#define LMH6518_POWER_MODE_MASK (0x0400)
#define LMH6518_POWER_MODE(x)   (((x) << 10) & LMH6518_POWER_MODE_MASK)

#define LMH6518_FILTER_MASK     (0x01C0)
#define LMH6518_FILTER(x)       (((x) << 6) & LMH6518_FILTER_MASK)
#define LMH6518_FILTER_MAX_VAL  (6)

#define LMH6518_PREAMP_MASK     (0x0010)
#define LMH6518_PREAMP(x)       (((x) << 4) & LMH6518_PREAMP_MASK)

#define LMH6518_ATTEN_MASK      (0x000F)
#define LMH6518_ATTEN(x)        ((x) & LMH6518_ATTEN_MASK)
#define LMH6518_ATTEN_MAX_VAL   (10)

#define LMH6518_INPUT_AMP_LG    (10000)
#define LMH6518_INPUT_AMP_HG    (30000)
#define LMH6518_OUTPUT_AMP      (8840)

#define LMH6518_ATTEN_STEPS     (10)
#define LMH6518_FILTER_STEPS    (7)

// Table of incremental Bandwidth Filters available.  All
// values in MHz.  Use index as config value.  Values come
// from datasheet Table 3.
static uint32_t g_filterTable[LMH6518_FILTER_STEPS] = {
    0,  20, 100, 200,
    350, 650, 750
};

// Table of incremental Attenuation values available.  All
// values in mdB.  Use index as config value.  Values come
// from datasheet Table 4.
static int32_t g_attenuationTable[LMH6518_ATTEN_STEPS] = {
    0,  -2000,
    -4000, -6000,
    -8000, -10000,
    -12000, -13000,
    -18000, -20000
};

int32_t lmh6518_calc_gain_config(lmh6518Config_t* conf, int32_t gain_mdB)
{
    uint8_t atten_index = LMH6518_ATTEN_STEPS - 1;
    int32_t input_gain = LMH6518_INPUT_AMP_LG;
    int32_t gain_actual = input_gain + (g_attenuationTable[atten_index])
                            + LMH6518_OUTPUT_AMP;
    
    int32_t prev_gain = gain_actual;
    uint8_t prev_index = atten_index;
    int32_t prev_input = input_gain;

    if(NULL == conf)
    {
        //Return zero on Error
        return 0;
    }
    
    do
    {
        // Check if we're larger than the requested gain
        if(gain_mdB <= gain_actual)
        {
            break;
        }

        if(atten_index == 0)
        {
            atten_index = LMH6518_ATTEN_STEPS - 1;
            input_gain = LMH6518_INPUT_AMP_HG;
        }
        else
        {
            atten_index--;
        }

        // Calculate next Gain    
        gain_actual = input_gain + (g_attenuationTable[atten_index])
                            + LMH6518_OUTPUT_AMP;
    
    } while(gain_actual < LMH6518_MAX_GAIN_mdB);
    
    //Check if previous setting was closer
    if((gain_mdB - prev_gain) < (gain_actual - gain_mdB))
    {
        input_gain = prev_input;
        atten_index = prev_index;
        gain_actual = prev_gain;
    }
    
    conf->preamp = (input_gain == LMH6518_INPUT_AMP_LG) ? 
                        PREAMP_LG : PREAMP_HG;
    conf->atten = atten_index;

    return gain_actual;
}


int32_t lmh6518_gain_from_config(lmh6518Config_t conf)
{
    int32_t input_gain = LMH6518_INPUT_AMP_LG;
    int32_t gain_actual;

    input_gain = (conf.preamp == PREAMP_LG) ? 
                    LMH6518_INPUT_AMP_LG : LMH6518_INPUT_AMP_HG;
    // Calculate next Gain    
    gain_actual = input_gain + (g_attenuationTable[conf.atten])
                        + LMH6518_OUTPUT_AMP;

    return gain_actual;
}

uint32_t lmh6518_set_bandwidth_filter(lmh6518Config_t* conf, uint32_t bw_MHz)
{
    int32_t bw_actual = 0;
    uint8_t filter_index = 1;

    if(NULL == conf)
    {
        //Return zero on Error
        return 0;
    }

    while(++filter_index < LMH6518_ATTEN_STEPS)
    {
        if(g_filterTable[filter_index] >= bw_MHz)
        {
            bw_actual = g_filterTable[filter_index];
            break;
        }
    }

    if(filter_index == LMH6518_ATTEN_STEPS)
    {
        // Index zero for full bandwidth
        filter_index = 0;
        bw_actual = LMH6518_MAX_BW_FILT_MHZ;
    }

    conf->filter = filter_index;

    return bw_actual;
}

int32_t lmh6518_apply_config(spi_dev_t dev, lmh6518Config_t conf)
{
    int32_t retVal = 0;
    uint16_t config = 0;
    uint8_t data[2];

    if((conf.filter > LMH6518_FILTER_MAX_VAL) ||
        (conf.atten > LMH6518_ATTEN_MAX_VAL))
    {
        LOG_ERROR("Invalid Param Filter %x Atten %x", conf.filter, conf.atten);
        retVal = TS_STATUS_ERROR;
        return retVal;
    }

    config |= LMH6518_POWER_MODE(conf.pm);
    config |= LMH6518_FILTER(conf.filter);
    config |= LMH6518_PREAMP(conf.preamp);
    config |= LMH6518_ATTEN(conf.atten);

    data[0] = (config >> 8) & 0xFF;
    data[1] = config & 0xFF;

    LOG_DEBUG("Set VGA Config 0x%04x", config);

    spi_busy_wait(dev);
    retVal = spi_write(dev, LMH6518_CMD_WRITE, data, 2);
    return retVal; 
}