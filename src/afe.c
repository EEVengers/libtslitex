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
#include "mcp4728.h"
#include "mcp443x.h"
#include "spi.h"
#include "i2c.h"
#include "gpio.h"
#include "util.h"

#include <stddef.h>
#include <math.h>


//Amp SPI Dev, Trim DAC, Trim DPot, term, attenuation, DC Switch
int32_t ts_afe_init(ts_afe_t* afe, uint8_t channel, spi_dev_t afe_amp, i2c_t trimDac, uint8_t dacCh,
            i2c_t trimPot, uint8_t potCh, gpio_t termination, gpio_t attenuator, gpio_t coupling)
{
    int32_t retVal;
    lmh6518Config_t defaultAmpConf = LMH6518_CONFIG_INIT;
    //Aux Output is not used
    defaultAmpConf.pm = PM_AUX_HIZ;
    
    if(channel >= TS_NUM_CHANNELS)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }

    afe->amp = afe_amp;
    afe->ampConf = defaultAmpConf;
    afe->trimDac = trimDac;
    afe->trimDacCh = dacCh;
    afe->trimPot = trimPot;
    afe->trimPotCh = potCh;
    afe->termPin = termination;
    afe->attenuatorPin = attenuator;
    afe->couplingPin = coupling;

    // Default states for AFE signals
    afe->termination = TS_TERM_1M;
    afe->coupling = TS_COUPLE_AC;
    afe->isAttenuated = true;
    
    // Default calibration
    afe->cal.buffer_mV = TS_VBUFFER_NOMINAL_MV;
    afe->cal.bias_mV = TS_VBIAS_NOMINAL_MV;
    afe->cal.attenuatorGain1M_mdB = TS_ATTENUATION_1M_GAIN_mdB;
    afe->cal.attenuatorGain50_mdB = TS_TERMINATION_50OHM_GAIN_mdB;
    afe->cal.bufferGain_mdB = TS_BUFFER_GAIN_NOMINAL_mdB;
    afe->cal.trimRheostat_range = MCP4432_FULL_SCALE_OHM;
    afe->cal.preampLowGainError_mdB = 0;
    afe->cal.preampHighGainError_mdB = 0;
    afe->cal.preampOutputGainError_mdB = 0;
    afe->cal.preampLowOffset_mV = 0;
    afe->cal.preampHighOffset_mV = 0;
    
    Mcp4728ChannelConfig_t trimConf = {0};
    trimConf.vref = MCP4728_VREF_VDD;
    trimConf.power = MCP4728_PD_NORMAL;
    trimConf.gain = MCP4728_GAIN_1X;
    trimConf.value = TS_TRIM_DAC_DEFAULT;

    //Set Initial Configuration
    retVal = lmh6518_apply_config(afe->amp, afe->ampConf);
    mcp4728_channel_set(afe->trimDac, afe->trimDacCh, trimConf);
    //TODO: Trim Dpot Defaults
    gpio_clear(termination);
    gpio_clear(attenuator);
    gpio_clear(coupling);

    return retVal;
}

int32_t ts_afe_set_gain(ts_afe_t* afe, int32_t gain_mdB)
{
    int32_t gain_actual = 0;
    int32_t gain_request = gain_mdB;
    afe->isAttenuated = false;
    if(NULL == afe)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }

    //Remove Buffer gain calibration value
    gain_request -= afe->cal.bufferGain_mdB;
    
    //Remove Preamp Output gain calibration value
    gain_request -= afe->cal.preampOutputGainError_mdB;

    // If 50-Ohm mode in use, limit gain to TBD
    if(afe->termination == TS_TERM_50)
    {
        gain_request -= afe->cal.attenuatorGain50_mdB;
    }
    else if(gain_request < LMH6518_MIN_GAIN_mdB)
    {
        // Update Attenuation if needed
        afe->isAttenuated = true;
        ts_afe_attenuation_control(afe, true);
        gain_request -= afe->cal.attenuatorGain1M_mdB;
    }

    gain_actual = lmh6518_calc_gain_config(&afe->ampConf, gain_request);

    if((gain_actual == 0) ||
        (TS_STATUS_OK != lmh6518_apply_config(afe->amp, afe->ampConf)))
    {
        return TS_STATUS_ERROR;
    }

    if(afe->isAttenuated)
    {
        gain_actual += afe->cal.attenuatorGain1M_mdB;
    }
    else
    {
        ts_afe_attenuation_control(afe, false);
        if(afe->termination == TS_TERM_50)
        {
            gain_actual += afe->cal.attenuatorGain50_mdB;
        }
    }

    if(afe->ampConf.preamp == PREAMP_LG)
    {
        gain_actual += afe->cal.preampLowGainError_mdB;
    }
    else
    {
        gain_actual += afe->cal.preampHighGainError_mdB;
    }

    gain_actual += afe->cal.bufferGain_mdB;
    gain_actual += afe->cal.preampOutputGainError_mdB;

    LOG_DEBUG("AFE Gain request: %d mdB actual: %d mdB", gain_mdB, gain_actual);

    return gain_actual;
}

int32_t ts_afe_set_offset(ts_afe_t* afe, int32_t offset_mV, int32_t* offset_actual)
{
    uint16_t offsetVal = TS_TRIM_DAC_DEFAULT;
    int32_t V_dac = 0;
    int32_t R_trim = 0;
    int32_t gain_afe = 0;
    int32_t V_zero = 0;
    int32_t gain_preamp = 0;

    if(NULL == afe || NULL == offset_actual)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }

    // Determine offset calculation
    if(afe->termination == TS_TERM_50)
    {
        gain_afe += afe->cal.attenuatorGain50_mdB;
    }
    else if(afe->isAttenuated)
    {
        gain_afe += afe->cal.attenuatorGain1M_mdB;
    }

    V_zero = afe->cal.buffer_mV;

    gain_preamp = lmh6518_gain_from_config(afe->ampConf);
    if(afe->ampConf.preamp == PREAMP_LG)
    {
        gain_preamp += afe->cal.preampLowGainError_mdB;
        V_zero += (int32_t)((double)afe->cal.preampLowOffset_mV / pow(10, ((double)gain_preamp/20000.0)));
    }
    else
    {
        gain_preamp += afe->cal.preampHighGainError_mdB;
        V_zero += (int32_t)((double)afe->cal.preampHighOffset_mV / pow(10, ((double)gain_preamp/20000.0)));
    } 

    // Desired Trim Voltage
    LOG_DEBUG("AFE Offset Request %d mv with %d mdB Input Gain", offset_mV, gain_afe);
    int32_t V_trim = V_zero + (uint32_t)((double)offset_mV * pow(10.0, (double)gain_afe/20000.0));
    LOG_DEBUG("AFE Offset target V_trim %d mV compared to V_zero of %d mV", V_trim, V_zero);
    
    // Progressively reduce R_trim until V_dac is within range of 0-VDD
    uint8_t trimPotVal = MCP4432_MAX;
    do
    {
        // R_trim = MCP4432_503_OHM(trimPotVal);
        R_trim = ((((trimPotVal) * afe->cal.trimRheostat_range) / MCP4432_MAX) + MCP4432_RWIPER);

        /** 
         * (V_trim - V_dac) / R_trim = (V_bias - V_trim) / 500
         *  Solved for V_dac becomes:
         *  V_dac = V_trim - ((V_bias - V_trim) * R_trim) / 500
         */
        V_dac = V_trim - ((R_trim * ((int32_t)afe->cal.bias_mV - V_trim)) / TS_BIAS_RESISTOR_NOMINAL);
        if(V_dac > 0 && V_dac < TS_AFE_TRIM_VDD_NOMINAL)
        {
            LOG_DEBUG("Setting Vdac to %d mV, Rtrim to %d Ohm", V_dac, R_trim);
            offsetVal = (V_dac * MCP4728_FULL_SCALE_VAL) / TS_AFE_TRIM_VDD_NOMINAL;
            break;
        }

        if(trimPotVal == 0)
        {
            LOG_ERROR("AFE Unable to produce Trim voltage %d for requested offset %d", V_trim, offset_mV);
            if(V_trim > afe->cal.bias_mV)
            {
                V_dac = TS_AFE_TRIM_VDD_NOMINAL;
                offsetVal = MCP4728_FULL_SCALE_VAL;
            }
            else
            {
                V_dac = 0;
            }
            break;
        }
    } while(trimPotVal-- > 0);

    Mcp4728ChannelConfig_t trimConf = {0};
    trimConf.vref = MCP4728_VREF_VDD;
    trimConf.power = MCP4728_PD_NORMAL;
    trimConf.gain = MCP4728_GAIN_1X;
    trimConf.value = offsetVal;

    if(TS_STATUS_OK != mcp4728_channel_set(afe->trimDac, afe->trimDacCh, trimConf))
    {
        return TS_STATUS_ERROR;
    }

    if(TS_STATUS_OK != mcp443x_set_wiper(afe->trimPot, afe->trimPotCh, trimPotVal))
    {
        return TS_STATUS_ERROR;
    }

    // Reverse offset calc
    V_dac = (offsetVal * TS_AFE_TRIM_VDD_NOMINAL) / MCP4728_FULL_SCALE_VAL;
    V_trim = V_dac + ((((int32_t)afe->cal.bias_mV - V_dac) * R_trim) / ( TS_BIAS_RESISTOR_NOMINAL + R_trim));
    *offset_actual = (int32_t)(((double)V_trim - ((double)V_zero)) / pow(10.0, (double)gain_afe/20000.0));
    LOG_DEBUG("AFE Offset actual V_trim %d mv, Offset %d mV", V_trim, *offset_actual);
    return TS_STATUS_OK;
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


int32_t ts_afe_termination_control(ts_afe_t* afe, tsChannelTerm_t term)
{
    if(NULL == afe)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }

    switch(term)
    {
    case TS_TERM_50:
    {
        LOG_DEBUG("Set Termination %x", afe->termPin.bit_mask);
        gpio_set(afe->termPin);
        break;
    }
    case TS_TERM_1M:
    {
        LOG_DEBUG("Clear Termination %x", afe->termPin.bit_mask);
        gpio_clear(afe->termPin);
        break;
    }
    default:
    {
        LOG_ERROR("Invalid AFE Termination Setting %x", term);
        return TS_INVALID_PARAM;
    }
    }

    afe->termination = term;
    return TS_STATUS_OK;
}

int32_t ts_afe_attenuation_control(ts_afe_t* afe, uint8_t isAttenuated)
{
    if(NULL == afe)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }
    if(!isAttenuated)
    {
        //Enabling this relay disables the AFE attenuation
        LOG_DEBUG("Set Attenuation %x", afe->attenuatorPin.bit_mask);
        gpio_set(afe->attenuatorPin);
    }
    else
    {
        //The AFE is attenuated when the relay is off
        LOG_DEBUG("Clear Attenuation %x", afe->attenuatorPin.bit_mask);
        gpio_clear(afe->attenuatorPin);
    }

    afe->isAttenuated = isAttenuated;
    return TS_STATUS_OK;
}

int32_t ts_afe_coupling_control(ts_afe_t* afe, tsChannelCoupling_t coupled)
{
    if(NULL == afe)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }

    switch(coupled)
    {
    case TS_COUPLE_DC:
    {
        LOG_DEBUG("Clear Coupling %x", afe->couplingPin.bit_mask);
        gpio_clear(afe->couplingPin);
        break;
    }
    case TS_COUPLE_AC:
    {
        LOG_DEBUG("Set Coupling %x", afe->couplingPin.bit_mask);
        gpio_set(afe->couplingPin);
        break;
    }
    default:
    {
        LOG_ERROR("Invalid AFE Coupling Setting %x", coupled);
        return TS_INVALID_PARAM;
    }
    }

    afe->coupling = coupled;
    return TS_STATUS_OK;
}

