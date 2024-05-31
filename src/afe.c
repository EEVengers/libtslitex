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

#include <stddef.h>


//Amp SPI Dev, Trim DAC, Trim DPot, term, attenuation, DC Switch
int32_t ts_afe_init(ts_afe_t* afe, uint8_t channel, spi_dev_t afe_amp, i2c_t trimDac,
            i2c_t trimPot, gpio_t termination, gpio_t attenuator, gpio_t coupling)
{
    int32_t retVal;
    lmh6518Config_t defaultAmpConf = LMH6518_CONFIG_INIT;
    
    if(channel >= TS_NUM_CHANNELS)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }

    afe->amp = afe_amp;
    afe->ampConf = defaultAmpConf;
    //Trim DAC
    //Trim DPot
    afe->termPin = termination;
    afe->attenuatorPin = attenuator;
    afe->couplingPin = coupling;

    //Set Initial Configuration
    retVal = lmh6518_apply_config(afe->amp, afe->ampConf);
    gpio_clear(termination);
    gpio_clear(attenuator);
    gpio_clear(coupling);

    return retVal;
}

int32_t ts_afe_set_gain(ts_afe_t* afe, int32_t gain_mdB)
{
    int32_t gain_actual = 0;
    bool need_atten = false;
    if(NULL == afe)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }

    // Update Attenuation if needed
    if(gain_mdB > TS_ATTENUATION_VALUE_mdB)
    {
        need_atten = true;
        ts_afe_termination_control(afe, 1);
        gain_mdB -= TS_ATTENUATION_VALUE_mdB;
    }

    gain_actual = lmh6518_calc_gain_config(&afe->ampConf, gain_mdB);

    if((gain_actual == 0) ||
        (TS_STATUS_OK != lmh6518_apply_config(afe->amp, afe->ampConf)))
    {
        return TS_STATUS_ERROR;
    }

    if(need_atten)
    {
        gain_actual += TS_ATTENUATION_VALUE_mdB;
        ts_afe_termination_control(afe, 0);
    }

    return gain_actual;
}

int32_t ts_afe_set_bw_filter(ts_afe_t* afe, uint32_t bw_MHz)
{
    int32_t retVal = 0;
    uint32_t bw_actual = 0;
    if(NULL == afe)
    {
        return TS_STATUS_ERROR;
    }
    bw_actual = lmh6518_set_bandwidth_filter(&afe->ampConf, bw_MHz);

    if((bw_actual == 0) ||
        (TS_STATUS_OK != lmh6518_apply_config(afe->amp, afe->ampConf)))
    {
        retVal = TS_STATUS_ERROR;
    }
    else
    {
        retVal = bw_actual;
    }

    return retVal;
}


int32_t ts_afe_termination_control(ts_afe_t* afe, uint8_t enable)
{
    if(NULL == afe)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }
    if(enable)
    {
        gpio_set(afe->termPin);
    }
    else
    {
        gpio_clear(afe->termPin);
    }
    return TS_STATUS_OK;
}

int32_t ts_afe_attenuation_control(ts_afe_t* afe, uint8_t enable)
{
    if(NULL == afe)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }
    if(enable)
    {
        gpio_set(afe->attenuatorPin);
    }
    else
    {
        gpio_clear(afe->attenuatorPin);
    }
    return TS_STATUS_OK;
}

int32_t ts_afe_coupling_control(ts_afe_t* afe, uint8_t enable)
{
    if(NULL == afe)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }
    if(enable)
    {
        gpio_set(afe->couplingPin);
    }
    else
    {
        gpio_clear(afe->couplingPin);
    }
    return TS_STATUS_OK;
}

