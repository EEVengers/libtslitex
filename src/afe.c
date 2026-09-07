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

#define TS_THRESH_ADJ       (1E-6) // Fudge Float compares by 1uV so we can reliably reapply the same settings

/**
 * @brief Calculate the the maximum possible offset for a single AFE config
 * 
 * @param cal AFE Calibration setting
 * @param temp_C Current temperature in Celcius
 * 
 * @return Maximum possible offset for the given AFE parameters
 */
static inline double ts_afe_offset_max(tsAfePathCalibration_t cal, double temp_C);

/**
 * @brief Calculate the the minimum possible offset for a single AFE config
 * 
 * @param cal AFE Calibration setting
 * @param temp_C Current temperature in Celcius
 * 
 * @return Minimum possible offset for the given AFE parameters
 */
static inline double ts_afe_offset_min(tsAfePathCalibration_t cal, double temp_C);


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
    if(isBetaDevice(trimPot.fd))
    {
        afe->trimPotBits = MCP4432_NUM_BITS;
    }
    else
    {
        afe->trimPotBits = MCP4452_NUM_BITS;

        if((litepcie_readl(coupling.fd, CSR_DEV_STATUS_HW_ID_ADDR) & TS_HW_ID_REV_MASK) > 0)
        {
            afe->couplingInverted = true;
        }
    }
    afe->termPin = termination;
    afe->attenuatorPin = attenuator;
    afe->couplingPin = coupling;

    // Default states for AFE signals
    afe->termination = TS_TERM_1M;
    afe->coupling = TS_COUPLE_AC;
    afe->isAttenuated = true;
    
    // Default calibration
    afe->cal = TS_AFE_DEFAULT_CAL;
    
    Mcp4728ChannelConfig_t trimConf = {0};
    trimConf.vref = MCP4728_VREF_VDD;
    trimConf.power = MCP4728_PD_NORMAL;
    trimConf.gain = MCP4728_GAIN_1X;
    trimConf.value = TS_TRIM_DAC_DEFAULT;

    //Set Initial Configuration
    retVal = lmh6518_apply_config(afe->amp, afe->ampConf);
    mcp4728_channel_set(afe->trimDac, afe->trimDacCh, trimConf);
    mcp443x_set_wiper(afe->trimPot, afe->trimPotCh, TS_TRIM_DPOT_DEFAULT);
    gpio_clear(termination);
    gpio_clear(attenuator);
    gpio_clear(coupling);

    return retVal;
}


int32_t ts_afe_set_ch_config(ts_afe_t* afe, double temp_C, double afe_Vpp, double offset, double* gain_actual, double* offset_actual)
{
    double reqScale = afe_Vpp;
    double reqOffset = offset;
    double attenScale = 1.0;
    double offsetScale;
    double offsetZero;
    bool needsAtten = false;
    lmh6518Preamp_t preamp = PREAMP_LG;
    uint32_t pathIdx = 0;
    uint8_t trimPotVal = 0;
    Mcp4728ChannelConfig_t trimConf = {.vref = MCP4728_VREF_VDD,
                                        .power = MCP4728_PD_NORMAL,
                                        .gain = MCP4728_GAIN_1X,
                                        .value = 0};

    if(NULL == afe || NULL == offset_actual || NULL == gain_actual)
    {
        //ERROR
        return TS_STATUS_ERROR;
    }

    afe->isAttenuated = false;

    // Test if requested Vpp is too large
    if(afe->termination == TS_TERM_50)
    {
        if (afe_Vpp > TS_AFE_50OHM_SAFE_INPUT_VPP)
        {
            LOG_ERROR("Cannot set requested voltage, too high");
            return TS_INVALID_PARAM;
        }
        
        if(isBetaDevice(afe->termPin.fd))
        {
            // If 50-Ohm mode in use, limit gain to TBD
            attenScale *= TS_AFE_BETA_TERM_SCALE;
        }
    }
    else
    {
        if (afe_Vpp > TS_AFE_1MOHM_SAFE_INPUT_VPP)
        {
            LOG_ERROR("Cannot set requested voltage, too high");
            return TS_INVALID_PARAM;
        } 
    }
    
    if((reqScale > afe->cal.lowPgaPathCal[TS_CAL_NUM_PATHS - 1].bufferInputVpp) ||
        (offset > ts_afe_offset_max(afe->cal.lowPgaPathCal[TS_CAL_NUM_PATHS - 1], temp_C)) ||
        (offset < ts_afe_offset_min(afe->cal.lowPgaPathCal[TS_CAL_NUM_PATHS - 1], temp_C)))
    {
        // Update Attenuation if needed
        needsAtten = true;
        attenScale *= afe->cal.attenuatorScale;
    }

    // Calculate Actual FSV
    reqScale /= attenScale;
    reqOffset /= attenScale;

    // Check if we need high or low gain range
    if ( (afe->cal.highPgaPathCal[TS_CAL_NUM_PATHS - 1].bufferInputVpp + TS_THRESH_ADJ) > reqScale)
    {
        preamp = PREAMP_HG;
    }

    LOG_DEBUG("Searching for: ");
    LOG_DEBUG("\tPreamp:    %s", preamp == PREAMP_HG ? "HIGH" : "LOW");
    LOG_DEBUG("\tBufferVpp: %f", reqScale);
    LOG_DEBUG("\tOffset:    %f", reqOffset);
    LOG_DEBUG("\tTemp:      %f C", temp_C);

    // Loop through PGA cal settings
    do {
        //Check offset range is valid
        if ((preamp == PREAMP_LG && 
                ((reqOffset > ts_afe_offset_max(afe->cal.lowPgaPathCal[pathIdx], temp_C)) ||
                 (reqOffset < ts_afe_offset_min(afe->cal.lowPgaPathCal[pathIdx], temp_C)))) ||
            (preamp == PREAMP_HG && 
                ((reqOffset > ts_afe_offset_max(afe->cal.highPgaPathCal[pathIdx], temp_C)) ||
                 (reqOffset < ts_afe_offset_min(afe->cal.highPgaPathCal[pathIdx], temp_C)))))
        {
            // Offset invalid. Try the next one
            continue;
        }

        // Check for Vpp range
        if (((preamp == PREAMP_LG) && ((afe->cal.lowPgaPathCal[pathIdx].bufferInputVpp + TS_THRESH_ADJ) > reqScale)) ||
            ((preamp == PREAMP_HG) && ((afe->cal.highPgaPathCal[pathIdx].bufferInputVpp + TS_THRESH_ADJ) > reqScale)))
        {
            //Use this one
            break;
        }

    } while(++pathIdx < TS_CAL_NUM_PATHS);

    if (pathIdx == TS_CAL_NUM_PATHS)
    {
        LOG_ERROR("Cannot set requested voltage, unable to find acceptable config");
        return TS_INVALID_PARAM;
    }

    // Set attenuator
    afe->isAttenuated = needsAtten;
    ts_afe_attenuation_control(afe, needsAtten);
    
    // Configure PGA
    afe->ampConf.atten = pathIdx;
    afe->ampConf.preamp = preamp;
    if(TS_STATUS_OK != lmh6518_apply_config(afe->amp, afe->ampConf))
    {
        return TS_STATUS_ERROR;
    }
    
    if (preamp == PREAMP_LG)
    {
        *gain_actual = attenScale * afe->cal.lowPgaPathCal[pathIdx].bufferInputVpp;
    }
    else
    {
        *gain_actual = attenScale * afe->cal.highPgaPathCal[pathIdx].bufferInputVpp;
    }

    // Adjust Trim DAC
    if (preamp == PREAMP_LG)
    {
        offsetScale = afe->cal.lowPgaPathCal[pathIdx].trimOffsetDacScale / (afe->cal.lowPgaPathCal[pathIdx].bufferInputVpp);
        offsetZero = ((afe->cal.lowPgaPathCal[pathIdx].trimOffsetDacZeroM * temp_C) + afe->cal.lowPgaPathCal[pathIdx].trimOffsetDacZeroC);
        trimPotVal = (uint8_t)afe->cal.lowPgaPathCal[pathIdx].trimDPot;
        
    }
    else
    {
        offsetScale = afe->cal.highPgaPathCal[pathIdx].trimOffsetDacScale / (afe->cal.highPgaPathCal[pathIdx].bufferInputVpp);
        offsetZero = ((afe->cal.highPgaPathCal[pathIdx].trimOffsetDacZeroM * temp_C) + afe->cal.highPgaPathCal[pathIdx].trimOffsetDacZeroC);
        trimPotVal = (uint8_t) afe->cal.highPgaPathCal[pathIdx].trimDPot;
    }
    
    trimConf.value = (uint16_t) (offsetZero + (reqOffset * offsetScale));
    
    //Limit DAC value between 0-4095
    if (trimConf.value < 0)
        trimConf.value = 0;
    if (trimConf.value > 4095)
        trimConf.value = 4095;

    *offset_actual = (((double)trimConf.value - offsetZero) / offsetScale) * attenScale;

    if (TS_STATUS_OK != mcp4728_channel_set(afe->trimDac, afe->trimDacCh, trimConf) ||
        (TS_STATUS_OK != mcp443x_set_wiper(afe->trimPot, afe->trimPotCh, trimPotVal)))
    {
        return TS_STATUS_ERROR;
    }
    
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
        if(afe->couplingInverted)
            gpio_set(afe->couplingPin);
        else
            gpio_clear(afe->couplingPin);

        break;
    }
    case TS_COUPLE_AC:
    {
        LOG_DEBUG("Set Coupling %x", afe->couplingPin.bit_mask);
        if(afe->couplingInverted)
            gpio_clear(afe->couplingPin);
        else
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


static inline double ts_afe_offset_max(tsAfePathCalibration_t cal, double temp_C)
{
    double offsetMax = 0.0;
    double dacRangePos = MCP4728_FULL_SCALE_VAL - ((cal.trimOffsetDacZeroM * temp_C) + cal.trimOffsetDacZeroC);
    
    if (dacRangePos < 0)
        dacRangePos = 0;
    else if (dacRangePos > MCP4728_FULL_SCALE_VAL)
        dacRangePos = MCP4728_FULL_SCALE_VAL;

    offsetMax = cal.bufferInputVpp * dacRangePos / cal.trimOffsetDacScale;

    return offsetMax;
}

static inline double ts_afe_offset_min(tsAfePathCalibration_t cal, double temp_C)
{
    double offsetMin = 0.0;
    double dacRangeNeg = cal.trimOffsetDacZeroM * temp_C + cal.trimOffsetDacZeroC;
    if (dacRangeNeg < 0)
        dacRangeNeg = 0;
    else if (dacRangeNeg > MCP4728_FULL_SCALE_VAL)
        dacRangeNeg = MCP4728_FULL_SCALE_VAL;

    offsetMin = - cal.bufferInputVpp * dacRangeNeg / cal.trimOffsetDacScale;

    return offsetMin;
}
