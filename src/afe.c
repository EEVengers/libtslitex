/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Control the AFE for a channel in the Thunderscope
 * LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#include "afe.h"

#include "ts_common.h"
#include "platform.h"
#include "lmh6518.h"
#include "spi.h"
#include "i2c.h"
#include "gpio.h"

typedef struct afe_s
{
    spi_dev_t amp;
    lmh6518Config_t ampConf;
    //Trim DAC
    //Trim DPot
    gpio_t termPin;
    gpio_t attenuatorPin;
    gpio_t couplingPin;
}afe_t;

static afe_t g_afeControl[TS_NUM_CHANNELS];

//Amp SPI Dev, Trim DAC, Trim DPot, term, attenuation, DC Switch
int32_t ts_afe_init_channel(uint8_t channel, spi_dev_t afe_amp, i2c_t trimDac,
            i2c_t trimPot, gpio_t termination, gpio_t attenuator, gpio_t coupling)
{
    int32_t retVal;
    lmh6518Config_t defaultAmpConf = LMH6518_CONFIG_INIT;
    
    if(channel >= TS_NUM_CHANNELS)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }

    g_afeControl[channel].amp = afe_amp;
    g_afeControl[channel].ampConf = defaultAmpConf;
    //Trim DAC
    //Trim DPot
    g_afeControl[channel].termPin = termination;
    g_afeControl[channel].attenuatorPin = attenuator;
    g_afeControl[channel].couplingPin = coupling;

    //Set Initial Configuration
    retVal = lmh6518_apply_config(g_afeControl[channel].ampConf);
    gpio_clear(termination);
    gpio_clear(attenuator);
    gpio_clear(coupling);

    return retVal;
}

int32_t ts_afe_set_gain(uint8_t channel, int32_t gain_mdB)
{
    int32_t gain_actual = 0;
    if(channel >= TS_NUM_CHANNELS)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }

    gain_actual = lmh6518_calc_gain_config(&g_afe[channel].ampConf, gain_mdB);

    if((gain_actual == 0) ||
        (TS_STATUS_OK != lmh6518_apply_config(g_afe[channel].amp, g_afe[channel].ampConf)))
    {
        return TS_STATUS_ERROR;
    }

    return gain_actual;
}

int32_t ts_afe_set_bw_filter(uint8_t channel, uint32_t bw_MHz)
{
    int32_t retVal = 0;
    uint32_t bw_actual = 0;
    if(channel >= TS_NUM_CHANNELS)
    {
        return TS_STATUS_ERROR;
    }
    bw_actual = lmh6518_set_bandwidth_filter(&g_afe[channel].ampConf, bw_MHz);

    if((bw_actual == 0) ||
        (TS_STATUS_OK != lmh6518_apply_config(g_afe[channel].amp, g_afe[channel].ampConf))
    {
        retVal = TS_STATUS_ERROR;
    }
    else
    {
        retVal = bw_actual;
    }

    return retVal;
}


int32_t ts_afe_termination_control(uint8_t channel, uint8_t enable)
{
    if(channel >= TS_NUM_CHANNELS)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }
    if(enable)
    {
        gpio_set(g_afeControl[channel].termPin);
    }
    else
    {
        gpio_clear(g_afeControl[channel].termPin);
    }
    return TS_STATUS_OK;
}

int32_t ts_afe_attenuation_control(uint8_t channel, uint8_t enable)
{
    if(channel >= TS_NUM_CHANNELS)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }
    if(enable)
    {
        gpio_set(g_afeControl[channel].attenuatorPin);
    }
    else
    {
        gpio_clear(g_afeControl[channel].attenuatorPin);
    }
    return TS_STATUS_OK;
}

int32_t ts_afe_coupling_control(uint8_t channel, uint8_t enable)
{
    if(channel >= TS_NUM_CHANNELS)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }
    if(enable)
    {
        gpio_set(g_afeControl[channel].couplingPin);
    }
    else
    {
        gpio_clear(g_afeControl[channel].couplingPin);
    }
    return TS_STATUS_OK;
}

