/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Channel management functions for the Thunderscope
 * LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include "ts_channel.h"
#include "ts_calibration.h"

#include "thunderscope.h"
#include "platform.h"
#include "spi.h"
#include "i2c.h"
#include "gpio.h"
#include "afe.h"
#include "adc.h"
#include "util.h"
#include "mcp443x.h"
#include "mcp4728.h"
#include "mcp_zl3026x.h"
#include "mcp_clkgen.h"


typedef struct ts_channel_s {
    struct {
        uint8_t channelNo;
        tsChannelParam_t params;
        ts_afe_t afe;
        tsChannelCalibration_t cal;
    } chan[TS_NUM_CHANNELS];
    ts_adc_t adc;
    spi_bus_t spibus;
    struct {
        i2c_t clkGen;
        gpio_t nRst;
        zl3026x_clk_config_t clkConf;
    }pll;
    gpio_t afe_power;
    gpio_t acq_power;
    file_t ctrl_handle;
    tsScopeState_t status;
} ts_channel_t;


struct ts_channel_hw_conf_s {
    uint32_t afe_amp_cs;
    uint32_t afe_term_reg;
    uint32_t afe_term_mask;
    uint32_t afe_cpl_reg;
    uint32_t afe_cpl_mask;
    uint32_t afe_atten_reg;
    uint32_t afe_atten_mask;
    uint8_t afe_dac_ch;
    uint8_t afe_dpot_ch;
    uint8_t adc_input;
    uint8_t adc_invert;
} g_channelConf[TS_NUM_CHANNELS] = {
    // Channel 1
    {
        TS_AFE_0_AMP_CS,
        TS_AFE_0_TERM_REG,     TS_AFE_0_TERM_MASK,
        TS_AFE_0_COUPLING_REG, TS_AFE_0_COUPLING_MASK,
        TS_AFE_0_ATTEN_REG,    TS_AFE_0_ATTEN_MASK,
        TS_AFE_0_TRIM_DAC,     TS_AFE_0_TRIM_DPOT,
        HMCAD15_ADC_IN4,       TS_ADC_CH_INVERT

    },
    // Channel 2
    {
        TS_AFE_1_AMP_CS,
        TS_AFE_1_TERM_REG,     TS_AFE_1_TERM_MASK,
        TS_AFE_1_COUPLING_REG, TS_AFE_1_COUPLING_MASK,
        TS_AFE_1_ATTEN_REG,    TS_AFE_1_ATTEN_MASK,
        TS_AFE_1_TRIM_DAC,     TS_AFE_1_TRIM_DPOT,
        HMCAD15_ADC_IN3,       TS_ADC_CH_INVERT
    },
    // Channel 3
    {
        TS_AFE_2_AMP_CS,
        TS_AFE_2_TERM_REG,     TS_AFE_2_TERM_MASK,
        TS_AFE_2_COUPLING_REG, TS_AFE_2_COUPLING_MASK,
        TS_AFE_2_ATTEN_REG,    TS_AFE_2_ATTEN_MASK,
        TS_AFE_2_TRIM_DAC,     TS_AFE_2_TRIM_DPOT,
        HMCAD15_ADC_IN2,       TS_ADC_CH_INVERT
    },
    // Channel 4
    {
        TS_AFE_3_AMP_CS,
        TS_AFE_3_TERM_REG,     TS_AFE_3_TERM_MASK,
        TS_AFE_3_COUPLING_REG, TS_AFE_3_COUPLING_MASK,
        TS_AFE_3_ATTEN_REG,    TS_AFE_3_ATTEN_MASK,
        TS_AFE_3_TRIM_DAC,     TS_AFE_3_TRIM_DPOT,
        HMCAD15_ADC_IN1,       TS_ADC_CH_INVERT
    }
};

const static tsChannelParam_t g_tsParamsDefault = {.active = false,
                                                   .bandwidth = 0,
                                                   .coupling = TS_COUPLE_DC,
                                                   .term = TS_TERM_1M,
                                                   .volt_offset_mV = 0,
                                                   .volt_scale_mV = 700};

static int32_t ts_channel_update_params(ts_channel_t* pTsHdl, uint32_t chanIdx, tsChannelParam_t* param, bool force);
static int32_t ts_channel_health_update(ts_channel_t* pTsHdl);

int32_t ts_channel_init(tsChannelHdl_t* pTsChannels, file_t ts)
{
    int32_t retVal = TS_STATUS_OK;

    if(pTsChannels == NULL)
    {
        retVal = TS_STATUS_ERROR;
        return retVal;
    }

    ts_channel_t* pChan = (ts_channel_t*)calloc(sizeof(ts_channel_t),1);

    if(pChan == NULL)
    {
        retVal = TS_STATUS_ERROR;
        return retVal;
    }

    //Initialize Status
    pChan->ctrl_handle = ts;
    pChan->status.adc_lost_buffer_count = 0;
    pChan->status.adc_sample_rate = 1000000000;
    pChan->status.adc_sample_bits = 8;
    pChan->status.adc_sample_resolution = 256;

    //Enable Power Rails
    pChan->afe_power.fd = ts;
    pChan->afe_power.reg = TS_AFE_POWER_REG;
    pChan->afe_power.bit_mask = TS_AFE_POWER_MASK;
    gpio_set(pChan->afe_power);
    pChan->status.afe_state = 1;

    pChan->acq_power.fd = ts;
    pChan->acq_power.reg = TS_ACQ_POWER_REG;
    pChan->acq_power.bit_mask = TS_ACQ_POWER_MASK;
    gpio_set(pChan->acq_power);
    pChan->status.power_state = 1;

    //Initialize PLL Clock Gen
    // Toggle reset pin
    pChan->pll.nRst.fd = ts;
    pChan->pll.nRst.reg = TS_PLL_NRST_ADDR;
    pChan->pll.nRst.bit_mask = TS_PLL_NRST_MASK;
    gpio_clear(pChan->pll.nRst);
    //sleep 10 ms
    NS_DELAY(10000000);
    gpio_set(pChan->pll.nRst);
    pChan->status.pll_state = 1;
    NS_DELAY(10000000);

    pChan->pll.clkGen.fd = ts;
    pChan->pll.clkGen.devAddr = TS_PLL_I2C_ADDR;

    //Set I2C Clock
    i2c_rate_set(pChan->pll.clkGen, TS_I2C_CLK_RATE);

    pChan->pll.clkConf.in_clks[TS_PLL_INPUT_IDX].enable = 1;
    pChan->pll.clkConf.in_clks[TS_PLL_INPUT_IDX].input_freq = TS_PLL_INPUT_RATE;
    pChan->pll.clkConf.input_select = TS_PLL_INPUT_SEL;
    pChan->pll.clkConf.out_clks[TS_PLL_REFOUT_CLK_IDX].enable = 1;
    pChan->pll.clkConf.out_clks[TS_PLL_REFOUT_CLK_IDX].output_freq = TS_PLL_REFOUT_RATE_DEFAULT;
    pChan->pll.clkConf.out_clks[TS_PLL_REFOUT_CLK_IDX].output_mode = TS_PLL_REFOUT_CLK_MODE;
    pChan->pll.clkConf.out_clks[TS_PLL_REFOUT_CLK_IDX].output_pll_select = TS_PLL_REFOUT_PLL_MODE;
    pChan->pll.clkConf.out_clks[TS_PLL_SAMPLE_CLK_IDX].enable = 1;
    pChan->pll.clkConf.out_clks[TS_PLL_SAMPLE_CLK_IDX].output_freq = TS_PLL_SAMPLE_RATE_DEFAULT;
    pChan->pll.clkConf.out_clks[TS_PLL_SAMPLE_CLK_IDX].output_mode = TS_PLL_SAMPLE_CLK_MODE;
    pChan->pll.clkConf.out_clks[TS_PLL_SAMPLE_CLK_IDX].output_pll_select = TS_PLL_SAMPLE_PLL_MODE;

    mcp_clkgen_conf_t clk_regs[MCP_CLKGEN_ARR_MAX_LEN] = {0};
    int32_t clk_len = mcp_zl3026x_build_config(clk_regs, MCP_CLKGEN_ARR_MAX_LEN, pChan->pll.clkConf);
    if(clk_len > 0)
    {
        retVal = mcp_clkgen_config(pChan->pll.clkGen, clk_regs, clk_len);
    }
    else
    {
        LOG_ERROR("Failed to generate PLL Configuration: %d", clk_len);
        retVal = TS_STATUS_ERROR;
    }
    if(retVal != TS_STATUS_OK)
    {
        goto channel_init_error;
    }

    i2c_t trimDac = {ts, TS_TRIM_DAC_I2C_ADDR};
    i2c_t trimPot = {ts, TS_TRIM_DPOT_I2C_ADDR};

    retVal = spi_bus_init(&pChan->spibus, ts,
                            TS_SPI_BUS_BASE_ADDR, TS_SPI_BUS_CS_NUM);
    if(retVal != TS_STATUS_OK)
    {
        goto channel_init_error;
    }

    spi_dev_t adcDev;
    retVal = spi_dev_init(&adcDev, &pChan->spibus, TS_ADC_CS);
    if(retVal != TS_STATUS_OK)
    {
        goto channel_init_error;
    }
    retVal = ts_adc_init(&pChan->adc, adcDev, ts);
    if(retVal != TS_STATUS_OK)
    {
        goto channel_init_error;
    }

    for(uint32_t chanIdx = 0; chanIdx < TS_NUM_CHANNELS; chanIdx++)
    {
        pChan->chan[chanIdx].channelNo = chanIdx;
        ts_adc_set_gain(&pChan->adc, chanIdx, TS_ADC_CH_COARSE_GAIN_DEFAULT, TS_ADC_CH_FINE_GAIN_DEFAULT);
        retVal = ts_adc_set_channel_conf(&pChan->adc, chanIdx, g_channelConf[chanIdx].adc_input,
                                            g_channelConf[chanIdx].adc_invert);
        if(retVal != TS_STATUS_OK)
        {
            goto channel_init_error;
        }

        spi_dev_t afe_amp;
        retVal = spi_dev_init(&afe_amp, &pChan->spibus,
                                g_channelConf[chanIdx].afe_amp_cs);
        if(retVal != TS_STATUS_OK)
        {
            goto channel_init_error;
        }

        gpio_t afe_term = {ts, g_channelConf[chanIdx].afe_term_reg,
                                g_channelConf[chanIdx].afe_term_mask};
        gpio_t afe_coupling = {ts, g_channelConf[chanIdx].afe_cpl_reg,
                                g_channelConf[chanIdx].afe_cpl_mask};
        gpio_t afe_atten = {ts, g_channelConf[chanIdx].afe_atten_reg,
                                g_channelConf[chanIdx].afe_atten_mask};

        retVal = ts_afe_init(&pChan->chan[chanIdx].afe, chanIdx, 
                                afe_amp, trimDac, g_channelConf[chanIdx].afe_dac_ch, trimPot,
                                g_channelConf[chanIdx].afe_dpot_ch, afe_term, afe_atten, afe_coupling);
        if(retVal != TS_STATUS_OK)
        {
            goto channel_init_error;
        }

        pChan->chan[chanIdx].params = g_tsParamsDefault;
    }

    if( TS_STATUS_OK != ts_channel_health_update(pChan))
    {
        LOG_ERROR("Failed to read System Health Statistics");
        goto channel_init_error;
    }


    if(retVal == TS_STATUS_OK)
    {
        *pTsChannels = pChan;
        return retVal;
    }

channel_init_error:
    *pTsChannels = NULL;
    gpio_clear(pChan->pll.nRst);
    gpio_clear(pChan->acq_power);
    gpio_clear(pChan->afe_power);
    free(pChan);
    return retVal;
}

int32_t ts_channel_destroy(tsChannelHdl_t tsChannels)
{
    ts_channel_t* pChan = (ts_channel_t*)tsChannels;

    //TODO Any cleanup on AFE/ADC as needed
    ts_adc_shutdown(&pChan->adc);

    //Hold PLL in reset
    gpio_clear(pChan->pll.nRst);
    
    //Power down
    gpio_clear(pChan->acq_power);
    gpio_clear(pChan->afe_power);
    
    free(tsChannels);

    return TS_STATUS_OK;
}

int32_t ts_channel_run(tsChannelHdl_t tsChannels, uint8_t en)
{
    if(!tsChannels)
    {
        return TS_STATUS_ERROR;
    }
    ts_channel_t* pChan = (ts_channel_t*)tsChannels;

    pChan->status.adc_state = en ? 1 : 0;
    return ts_adc_run(&pChan->adc, en);

}

int32_t ts_channel_params_set(tsChannelHdl_t tsChannels, uint32_t chanIdx, tsChannelParam_t* param)
{

    if(tsChannels == NULL || param == NULL)
    {
        return TS_STATUS_ERROR;
    }

    if(chanIdx >= TS_NUM_CHANNELS)
    {
        return TS_INVALID_PARAM;
    }

    ts_channel_t* pInst = (ts_channel_t*)tsChannels;

    return ts_channel_update_params(pInst, chanIdx, param, false);

}

static int32_t ts_channel_update_params(ts_channel_t* pTsHdl, uint32_t chanIdx, tsChannelParam_t* param, bool force)
{

    int32_t retVal = TS_STATUS_OK;
    bool needUpdateGain = false, needUpdateOffset = false, needUpdateSampleRate = false;
    //Set AFE Bandwidth
    if(param->bandwidth != pTsHdl->chan[chanIdx].params.bandwidth || force)
    {
        retVal = ts_afe_set_bw_filter(&pTsHdl->chan[chanIdx].afe, param->bandwidth);
        if(retVal > 0)
        {
            LOG_DEBUG("Channel %d AFE BW set to %i MHz", chanIdx, retVal);
            pTsHdl->chan[chanIdx].params.bandwidth = retVal;
        }
        else
        {
            LOG_ERROR("Unable to set Channel %d bandwidth %d", chanIdx, retVal);
            return TS_INVALID_PARAM;
        }
    }

    //Set AC/DC Coupling
    if(param->coupling != pTsHdl->chan[chanIdx].params.coupling || force)
    {
        if(TS_STATUS_OK == ts_afe_coupling_control(&pTsHdl->chan[chanIdx].afe,
                                            (tsChannelCoupling_t)param->coupling))
        {
            
            LOG_DEBUG("Channel %d AFE set to %s coupling", chanIdx, param->coupling == TS_COUPLE_DC ? "DC" : "AC");
            pTsHdl->chan[chanIdx].params.coupling = param->coupling;
        }
        else
        {
            LOG_ERROR("Unable to set Channel %d AC/DC Coupling: %x", chanIdx, param->coupling);
            return TS_INVALID_PARAM;
        }
    }

    //Set Termination
    if(param->term != pTsHdl->chan[chanIdx].params.term || force)
    {
        if(TS_STATUS_OK == ts_afe_termination_control(&pTsHdl->chan[chanIdx].afe,
                                            (tsChannelTerm_t)param->term))
        {
            LOG_DEBUG("Channel %d AFE termination set to %s", chanIdx, param->term == TS_TERM_1M ? "1M" : "50");
            pTsHdl->chan[chanIdx].params.term = param->term;
            needUpdateGain = true;
        }
        else
        {
            LOG_ERROR("Unable to set Channel %d Termination: %x", chanIdx, param->term);
            return TS_INVALID_PARAM;
        }
    }

    //Set Voltage Scale
    if(needUpdateGain || (param->volt_scale_mV != pTsHdl->chan[chanIdx].params.volt_scale_mV) || force)
    {
        //Calculate dB gain value
        //TODO: Set both AFE and ADC gain?
        int32_t afe_gain_mdB = (int32_t)(20000 * log10(TS_AFE_OUTPUT_NOMINAL_mVPP / (double)param->volt_scale_mV));

        LOG_DEBUG("Channel %d AFE request %i mdB gain", chanIdx, afe_gain_mdB);

        retVal = ts_afe_set_gain(&pTsHdl->chan[chanIdx].afe, afe_gain_mdB);
        if(TS_STATUS_ERROR == retVal)
        {
            LOG_ERROR("Unable to set Channel %d voltage scale: %x", chanIdx, param->volt_scale_mV);
            return TS_INVALID_PARAM;
        }
        else
        {
            LOG_DEBUG("Channel %d AFE set to %i mdB gain", chanIdx, retVal);
            retVal = (int32_t)(TS_AFE_OUTPUT_NOMINAL_mVPP / pow(10.0, (double)retVal / 20000.0));
            LOG_DEBUG("Channel %d voltage scale Request: %d Actual: %d", chanIdx, param->volt_scale_mV, retVal);
            pTsHdl->chan[chanIdx].params.volt_scale_mV = retVal;
            needUpdateOffset = true;
        }
    }

    //Set Voltage Offset
    if(needUpdateOffset || (param->volt_offset_mV != pTsHdl->chan[chanIdx].params.volt_offset_mV) || force)
    {
        //Adjust Trim DAC
        int32_t offset_actual = 0;
        retVal = ts_afe_set_offset(&pTsHdl->chan[chanIdx].afe, param->volt_offset_mV, &offset_actual);
        if(TS_STATUS_OK != retVal)
        {
            LOG_ERROR("Unable to set Channel %d voltage offset: %i", chanIdx, param->volt_offset_mV);
            return TS_INVALID_PARAM;
        }
        else
        {
            LOG_DEBUG("Channel %d AFE set to %i mV offset", chanIdx, offset_actual);
            // pTsHdl->chan[chanIdx].params.volt_offset_mV = offset_actual;
            pTsHdl->chan[chanIdx].params.volt_offset_mV = param->volt_offset_mV;
        }
    }

    //Set Active
    if(param->active != pTsHdl->chan[chanIdx].params.active)
    {
        //ADC Run will be reenabled when updating the sample rate
        ts_adc_run(&pTsHdl->adc, 0);
        retVal = ts_adc_channel_enable(&pTsHdl->adc, chanIdx, param->active);

        if(TS_STATUS_OK != retVal)
        {
            LOG_ERROR("Unable to %s Channel %d: %d", (param->active == 0 ? "disable" : "enable"),
                        chanIdx, retVal);
            return retVal;
        }
        else
        {
            LOG_DEBUG("Channel %d %s", chanIdx, (param->active == 0 ? "disabled" : "enabled"));
            pTsHdl->chan[chanIdx].params.active = param->active;
        }

        //Update Sample Rate
        retVal = ts_channel_sample_rate_set((tsChannelHdl_t)pTsHdl, pTsHdl->status.adc_sample_rate, pTsHdl->status.adc_sample_resolution);
    }

    return retVal;
}

int32_t ts_channel_params_get(tsChannelHdl_t tsChannels, uint32_t chanIdx, tsChannelParam_t* param)
{
    if(tsChannels == NULL || param == NULL)
    {
        return TS_STATUS_ERROR;
    }

    if(chanIdx >= TS_NUM_CHANNELS)
    {
        return TS_INVALID_PARAM;
    }

    memcpy(param, &((ts_channel_t*)tsChannels)->chan[chanIdx].params, sizeof(tsChannelParam_t));
    return TS_STATUS_OK;
}

tsScopeState_t ts_channel_scope_status(tsChannelHdl_t tsChannels)
{
    if(tsChannels == NULL)
    {
        //Return empty state
        tsScopeState_t state = {0};
        return state;
    }

    //Update XADC values
    ts_channel_health_update((ts_channel_t*)tsChannels);

    return ((ts_channel_t*)tsChannels)->status;
}

int32_t ts_channel_sample_rate_set(tsChannelHdl_t tsChannels, uint32_t rate, uint32_t resolution)
{
    if(tsChannels == NULL)
    {
        return TS_STATUS_ERROR;
    }
    ts_channel_t* ts =  (ts_channel_t*)tsChannels;
    uint64_t actual_rate = 0;

    //TODO - Support valid rate/resolution combinations
    if((rate < TS_MIN_SAMPLE_RATE) || (rate > TS_MAX_SAMPLE_RATE)
         || (resolution != 256))
    {
        return TS_INVALID_PARAM;
    }

    //Input validation
    if(ts->adc.adcDev.mode == HMCAD15_SINGLE_CHANNEL)
    {
        actual_rate = rate;
    }
    else if(ts->adc.adcDev.mode == HMCAD15_DUAL_CHANNEL)
    {
        //Limit upper rate
        if(rate > TS_MAX_DUAL_CH_RATE)
        {
            rate = TS_MAX_DUAL_CH_RATE;
        }
        actual_rate = rate * 2;
    }
    else
    {
        //Limit upper rate
        if(rate > TS_MAX_QUAD_CH_RATE)
        {
            rate = TS_MAX_QUAD_CH_RATE;
        }
        actual_rate = rate * 4;
    }

    if(actual_rate != ts->pll.clkConf.out_clks[TS_PLL_SAMPLE_CLK_IDX].output_freq)
    {
        ts_adc_run(&ts->adc, 0);
        
        // Apply resolution,rate configuration
        zl3026x_clk_config_t newConf = ts->pll.clkConf;
        newConf.out_clks[TS_PLL_SAMPLE_CLK_IDX].output_freq = actual_rate;
        
        mcp_clkgen_conf_t clk_regs[MCP_CLKGEN_ARR_MAX_LEN] = {0};
        int32_t clk_len = mcp_zl3026x_build_config(clk_regs, MCP_CLKGEN_ARR_MAX_LEN, newConf);
        if(clk_len > 0)
        {
            if(TS_STATUS_OK != mcp_clkgen_config(ts->pll.clkGen, clk_regs, clk_len))
            {
                return TS_STATUS_ERROR;
            }
            ts->pll.clkConf = newConf;
        }
        else
        {
            LOG_ERROR("Failed to generate PLL Configuration: %d", clk_len);
            return clk_len;
        }

        ts->status.adc_sample_rate = rate;
        ts->status.adc_sample_resolution = resolution;
        ts->status.adc_sample_bits = resolution == 256 ? 8 : 16;

        ts_adc_set_sample_mode(&ts->adc, rate, resolution);
        ts_adc_run(&ts->adc, ts->status.adc_state);
    }

    return  TS_STATUS_OK;
}


int32_t ts_channel_calibration_set(tsChannelHdl_t tsChannels, uint32_t chanIdx, tsChannelCalibration_t* cal)
{
    ts_channel_t* ts =  (ts_channel_t*)tsChannels;
    if(tsChannels == NULL || cal == NULL)
    {
        LOG_ERROR("Invalid handle");
        return TS_STATUS_ERROR;
    }

    if(chanIdx >= TS_NUM_CHANNELS)
    {
        return TS_INVALID_PARAM;
    }

    //TODO Calibration value bounds checking
    ts->chan[chanIdx].afe.cal = *cal;

    LOG_DEBUG("Received Calibration for channel %d", chanIdx);
    LOG_DEBUG("\tBuffer Output:                 %d mV", cal->buffer_mV);
    LOG_DEBUG("\t+VBIAS:                        %d mV", cal->bias_mV);
    LOG_DEBUG("\t1M Attenuator Gain:            %d mdB", cal->attenuatorGain1M_mdB);
    LOG_DEBUG("\t50 Ohm Terminator Gain:        %d mdB", cal->attenuatorGain50_mdB);
    LOG_DEBUG("\tBuffer Output Gain:            %d mdB", cal->bufferGain_mdB);
    LOG_DEBUG("\tTrim Rheostat:                 %d Ohm", cal->trimRheostat_range);
    LOG_DEBUG("\tPreamp Low Input Gain Error:   %d mdB", cal->preampLowGainError_mdB);
    LOG_DEBUG("\tPreamp High Input Gain Error:  %d mdB", cal->preampHighGainError_mdB);
    LOG_DEBUG("\tPreamp High Input Gain Error:  %d mdB", cal->preampOutputGainError_mdB);
    LOG_DEBUG("\tPreamp Low Output Offset:      %d mV", cal->preampLowOffset_mV);
    LOG_DEBUG("\tPreamp High Output Offset:     %d mV", cal->preampHighOffset_mV);
    LOG_DEBUG("\tPreamp Input Bias Current:     %d uA", cal->preampInputBias_uA);

    //Force afe to recalculate gain/offsets
    ts_channel_update_params(ts, chanIdx, &ts->chan[chanIdx].params, true);

    return TS_STATUS_OK;
}

int32_t ts_channel_calibration_manual(tsChannelHdl_t tsChannels, uint32_t chanIdx, tsChannelCtrl_t ctrl)
{
    int32_t retVal = TS_STATUS_OK;
    ts_channel_t* ts =  (ts_channel_t*)tsChannels;
    if(tsChannels == NULL)
    {
        LOG_ERROR("Invalid handle");
        return TS_STATUS_ERROR;
    }

    if(chanIdx >= TS_NUM_CHANNELS)
    {
        return TS_INVALID_PARAM;
    }


    //Set AFE Bandwidth
    retVal = ts_afe_set_bw_filter(&ts->chan[chanIdx].afe, ctrl.pga_bw);
    if(retVal > 0)
    {
        LOG_DEBUG("Channel %d AFE BW set to %i MHz", chanIdx, retVal);
    }
    else
    {
        LOG_ERROR("Unable to set Channel %d bandwidth %d", chanIdx, retVal);
        return TS_INVALID_PARAM;
    }


    //Set AC/DC Coupling
    if(TS_STATUS_OK == ts_afe_coupling_control(&ts->chan[chanIdx].afe,
                                        ctrl.dc_couple == 1 ? TS_COUPLE_DC : TS_COUPLE_AC ))
    {
        
        LOG_DEBUG("Channel %d AFE set to %s coupling", chanIdx, ctrl.dc_couple == 1 ? "DC" : "AC");
    }
    else
    {
        LOG_ERROR("Unable to set Channel %d AC/DC Coupling: %x", chanIdx, ctrl.dc_couple);
        return TS_INVALID_PARAM;
    }
    
    //Set Attenuator
    if(TS_STATUS_OK == ts_afe_attenuation_control(&ts->chan[chanIdx].afe, ctrl.atten))
    {
        LOG_DEBUG("Channel %d AFE attenuation set to %i", chanIdx, ctrl.atten);
    }
    else
    {
        LOG_ERROR("Unable to set Channel %d Attenuation: %x", chanIdx, ctrl.atten);
        return TS_INVALID_PARAM;
    }

    //Set Termination
    if(TS_STATUS_OK == ts_afe_termination_control(&ts->chan[chanIdx].afe,
                                        ctrl.term == 1 ? TS_TERM_50 : TS_TERM_1M))
    {
        LOG_DEBUG("Channel %d AFE termination set to %s", chanIdx, ctrl.term == 0 ? "1M" : "50");
    }
    else
    {
        LOG_ERROR("Unable to set Channel %d Termination: %x", chanIdx, ctrl.term);
        return TS_INVALID_PARAM;
    }

    //Set Preamp
    lmh6518Config_t preamp = LMH6518_CONFIG_INIT;
    preamp.atten = ctrl.pga_atten;
    preamp.filter = ctrl.pga_bw;
    preamp.preamp = ctrl.pga_high_gain = 0 ? PREAMP_LG : PREAMP_HG;
    preamp.pm = PM_AUX_HIZ;

    retVal = lmh6518_apply_config(ts->chan[chanIdx].afe.amp, preamp);
    if(TS_STATUS_ERROR == retVal)
    {
        LOG_ERROR("Unable to set Channel %d Preamp", chanIdx);
        return TS_INVALID_PARAM;
    }
    else
    {
        LOG_DEBUG("Channel %d Preamp set to %i bw, %i atten, and %s", chanIdx,
                    preamp.filter, preamp.atten, preamp.preamp == PREAMP_LG ? "Low Gain" : "High Gain");
    }

    //Set Trim
    int32_t offset_actual = 0;
    Mcp4728ChannelConfig_t conf;
    conf.gain = MCP4728_GAIN_1X;
    conf.vref = MCP4728_VREF_VDD;
    conf.power = MCP4728_PD_NORMAL;
    conf.value = ctrl.dac;
    
    retVal = mcp4728_channel_set(ts->chan[chanIdx].afe.trimDac, ts->chan[chanIdx].afe.trimDacCh, conf);
    retVal |= mcp443x_set_wiper(ts->chan[chanIdx].afe.trimPot, ts->chan[chanIdx].afe.trimPotCh, ctrl.dpot);
    if(TS_STATUS_OK != retVal)
    {
        LOG_ERROR("Unable to set Channel %d Trim Voltage", chanIdx);
        return TS_INVALID_PARAM;
    }
    else
    {
        LOG_DEBUG("Channel %d DAC set to %i, DPOT set to %i", chanIdx, ctrl.dac, ctrl.dpot);
    }

    return TS_STATUS_OK;
}

int32_t ts_channel_set_adc_test(tsChannelHdl_t tsChannels, hmcad15xxTestMode_t mode, uint16_t pattern1, uint16_t pattern2)
{
    
    return hmcad15xx_set_test_pattern(&((ts_channel_t*)tsChannels)->adc.adcDev, mode, pattern1, pattern2);
}

static int32_t ts_channel_health_update(ts_channel_t* pTsHdl)
{
    pTsHdl->status.sys_health.temp_c = (uint32_t)((double)(litepcie_readl(pTsHdl->ctrl_handle, CSR_XADC_TEMPERATURE_ADDR) * 503.975 / 4096 - 273.15)*1000);
    pTsHdl->status.sys_health.vcc_int = (uint32_t)(((double)litepcie_readl(pTsHdl->ctrl_handle, CSR_XADC_VCCINT_ADDR) / 4096 * 3)*1000);
    pTsHdl->status.sys_health.vcc_aux = (uint32_t)(((double)litepcie_readl(pTsHdl->ctrl_handle, CSR_XADC_VCCAUX_ADDR) / 4096 * 3)*1000);
    pTsHdl->status.sys_health.vcc_bram = (uint32_t)(((double)litepcie_readl(pTsHdl->ctrl_handle, CSR_XADC_VCCBRAM_ADDR) / 4096 * 3)*1000);

    return TS_STATUS_OK;
}