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
    tsAdcCalibration_t adcCal;
    spi_bus_t adcSpibus;
    spi_bus_t spibus;
    struct {
        i2c_t clkGen;
        gpio_t nRst;
        zl3026x_clk_config_t clkConf;
    }pll;
    gpio_t afe_power;
    gpio_t acq_power;
    file_t ctrl_handle;
    tsSampleFormat_t sampleMode;
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
        HMCAD15_ADC_IN4,       TS_ADC_CH_NO_INVERT

    },
    // Channel 2
    {
        TS_AFE_1_AMP_CS,
        TS_AFE_1_TERM_REG,     TS_AFE_1_TERM_MASK,
        TS_AFE_1_COUPLING_REG, TS_AFE_1_COUPLING_MASK,
        TS_AFE_1_ATTEN_REG,    TS_AFE_1_ATTEN_MASK,
        TS_AFE_1_TRIM_DAC,     TS_AFE_1_TRIM_DPOT,
        HMCAD15_ADC_IN3,       TS_ADC_CH_NO_INVERT
    },
    // Channel 3
    {
        TS_AFE_2_AMP_CS,
        TS_AFE_2_TERM_REG,     TS_AFE_2_TERM_MASK,
        TS_AFE_2_COUPLING_REG, TS_AFE_2_COUPLING_MASK,
        TS_AFE_2_ATTEN_REG,    TS_AFE_2_ATTEN_MASK,
        TS_AFE_2_TRIM_DAC,     TS_AFE_2_TRIM_DPOT,
        HMCAD15_ADC_IN2,       TS_ADC_CH_NO_INVERT
    },
    // Channel 4
    {
        TS_AFE_3_AMP_CS,
        TS_AFE_3_TERM_REG,     TS_AFE_3_TERM_MASK,
        TS_AFE_3_COUPLING_REG, TS_AFE_3_COUPLING_MASK,
        TS_AFE_3_ATTEN_REG,    TS_AFE_3_ATTEN_MASK,
        TS_AFE_3_TRIM_DAC,     TS_AFE_3_TRIM_DPOT,
        HMCAD15_ADC_IN1,       TS_ADC_CH_NO_INVERT
    }
};

const static tsChannelParam_t g_tsParamsDefault = {.active = false,
                                                   .bandwidth = 0,
                                                   .coupling = TS_COUPLE_DC,
                                                   .term = TS_TERM_1M,
                                                   .volt_offset_uV = 0,
                                                   .volt_scale_uV = 700000};

static int32_t ts_channel_update_params(ts_channel_t* pTsHdl, uint32_t chanIdx, tsChannelParam_t* param, bool force);
static int32_t ts_channel_health_update(ts_channel_t* pTsHdl);
static tsChannelsActive_t ts_channel_active_get(ts_channel_t* pTsHdl);
static uint32_t ts_channel_active_index(tsChannelsActive_t active, uint32_t chanIdx);

int32_t ts_channel_init(tsChannelHdl_t* pTsChannels, file_t ts)
{
    int32_t retVal = TS_STATUS_OK;
    bool betaDevice = false;

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

    uint32_t id = litepcie_readl(ts, CSR_DEV_STATUS_HW_ID_ADDR);
    if(0 == (id & (1 << CSR_DEV_STATUS_HW_ID_HW_VALID_OFFSET)))
    {
        betaDevice = true;
    }

    //Units prior to HWID 2 (Gamma HW Rev 5.3) have the ADC P/N pairs swapped
    if((id & (( 1UL << CSR_DEV_STATUS_HW_ID_HW_REV_SIZE) - 1)) < 0x2)
    {
        g_channelConf[0].adc_invert = TS_ADC_CH_INVERT;
        g_channelConf[1].adc_invert = TS_ADC_CH_INVERT;
        g_channelConf[2].adc_invert = TS_ADC_CH_INVERT;
        g_channelConf[3].adc_invert = TS_ADC_CH_INVERT;
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
    if(betaDevice)
    {
        pChan->pll.clkGen.peripheral_baseaddr = TS_PLL_BUS_BETA;
    }
    else
    {
        pChan->pll.clkGen.peripheral_baseaddr = TS_PLL_BUS_DEV;
    }

    //Set I2C Clock
    i2c_rate_set(pChan->pll.clkGen, TS_I2C_CLK_RATE);

    pChan->pll.clkConf.in_clks[TS_PLL_LOCAL_OSC_IDX].enable = 1;
    pChan->pll.clkConf.in_clks[TS_PLL_LOCAL_OSC_IDX].input_freq = TS_PLL_LOCAL_OSC_RATE;
    pChan->pll.clkConf.in_clks[TS_PLL_LOCAL_OSC_IDX].input_divider = 0;
    pChan->pll.clkConf.input_select = TS_PLL_LOCAL_OSC_SEL;
    pChan->pll.clkConf.alternate_select = TS_PLL_INPUT_NONE_SEL;
    pChan->pll.clkConf.in_clks[TS_PLL_REFIN_IDX].enable = 0;
    pChan->pll.clkConf.in_clks[TS_PLL_REFIN_IDX].input_divider = 0;
    pChan->pll.clkConf.out_clks[TS_PLL_REFOUT_CLK_IDX].enable = 0;
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

    i2c_t trimDac = {ts, TS_TRIM_DAC_BUS, TS_TRIM_DAC_I2C_ADDR};
    i2c_t trimPot = {ts, TS_TRIM_DPOT_BUS, TS_TRIM_DPOT_I2C_ADDR};

    if(betaDevice)
    {
        retVal = spi_bus_init(&pChan->spibus, ts,
                    TS_SPI_BUS_BASE_ADDR, TS_SPI_BUS_BETA_CS_NUM);
    }
    else
    {
        retVal = spi_bus_init(&pChan->spibus, ts,
            TS_SPI_BUS_BASE_ADDR, TS_SPI_BUS_DEV_CS_NUM);
    }
    if(retVal != TS_STATUS_OK)
    {
        goto channel_init_error;
    }

    if(!betaDevice)
    {
        retVal = spi_bus_init(&pChan->adcSpibus, ts,
            TS_ADC_SPI_BUS_BASE_ADDR, TS_ADC_SPI_BUS_CS_NUM);
        if(retVal != TS_STATUS_OK)
        {
            goto channel_init_error;
        }
    }

    spi_dev_t adcDev;
    if(betaDevice)
    {
        retVal = spi_dev_init(&adcDev, &pChan->spibus, TS_BETA_ADC_CS);
    }
    else
    {
        retVal = spi_dev_init(&adcDev, &pChan->adcSpibus, TS_ADC_CS);
    }

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
        ts_adc_set_gain(&pChan->adc, chanIdx, TS_ADC_CH_COARSE_GAIN_DEFAULT);
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
    bool needUpdateGain = false, needUpdateOffset = false, needActiveUpdate = false;
    tsChannelsActive_t active = TS_CHAN_NONE;
    
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

        needActiveUpdate = true;
    }

    if ((0 != param->active) &&
        ((param->volt_scale_uV != pTsHdl->chan[chanIdx].params.volt_scale_uV) ||
         (param->volt_offset_uV != pTsHdl->chan[chanIdx].params.volt_offset_uV)))
    {
        needUpdateGain = true;
    }

    active = ts_channel_active_get(pTsHdl);

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
    if(needUpdateGain || force)
    {
        // Calculate gain value
        double gain = 1.0;
        double requestVpp = ((double)param->volt_scale_uV) * 0.000001;

        // 1. Adjust for ADC Load scale
        double loadScale = 1.0;
        uint32_t scaleIdx = ts_channel_active_index(active, chanIdx);

        // Check the scaleIdx is valid
        if (scaleIdx < TS_NUM_CHANNELS)
        {
            for (uint32_t load = 0; load < TS_CAL_NUM_LOADS; load++)
            {
                if (pTsHdl->adcCal.loadCal[load].channels == active)
                {
                    for(uint32_t rateIdx = 0; rateIdx < TS_CAL_NUM_RATES; rateIdx++)
                    {
                        if (pTsHdl->adcCal.loadCal[load].conf[rateIdx].rate == pTsHdl->status.adc_sample_rate)
                        {
                            loadScale = pTsHdl->adcCal.loadCal[load].conf[rateIdx].scale[scaleIdx];
                            break;
                        }
                    }
                    break;
                }
            }
        }

        // 2. Update AFE Settings
        double offset_actual = 0.0;
        LOG_DEBUG("Channel %d AFE request %f Vpp", chanIdx, requestVpp);
        retVal = ts_afe_set_ch_config(&pTsHdl->chan[chanIdx].afe,
                                        (pTsHdl->status.sys_health.temp_c / 1000.0),
                                        (requestVpp / loadScale),
                                        param->volt_offset_uV / 1000000.0,
                                        &gain, &offset_actual);
        if(TS_STATUS_OK != retVal)
        {
            LOG_ERROR("Unable to set Channel %d voltage scale: %i", chanIdx, param->volt_scale_uV);
            LOG_ERROR("                        voltage offset: %i", chanIdx, param->volt_offset_uV);
            return TS_INVALID_PARAM;
        }
        else
        {
            pTsHdl->chan[chanIdx].params.volt_scale_uV = (uint32_t) (gain * loadScale * 1000000.0);
            LOG_DEBUG("Channel %d voltage scale Request: %d Actual: %d",
                        chanIdx, param->volt_scale_uV,
                        pTsHdl->chan[chanIdx].params.volt_scale_uV);
            pTsHdl->chan[chanIdx].params.volt_offset_uV = (uint32_t) (offset_actual * 1000000.0);
            LOG_DEBUG("Channel %d AFE set to %.06f V offset", chanIdx, offset_actual);
        }
    }

    //Set Active
    if(needActiveUpdate)
    {
        // Reconfigure parameters for other active channels
        for (uint8_t ch = 0; ch < TS_NUM_CHANNELS; ch++)
        {
            //TODO: Refactor this function to remove reentrant behavior
            //  This is only reached if the given active param differs from the chan[] param, so it should
            //  only cause a single reentrant call here
            if (ch != chanIdx && pTsHdl->chan[ch].params.active)
            {
                ts_channel_update_params(pTsHdl, ch, &pTsHdl->chan[ch].params, true);
            }
        }

        //Update Sample Rate
        retVal = ts_channel_sample_rate_set((tsChannelHdl_t)pTsHdl, pTsHdl->status.adc_sample_rate, pTsHdl->sampleMode);
    }

    return retVal;
}

static tsChannelsActive_t ts_channel_active_get(ts_channel_t* pTsHdl)
{
    tsChannelsActive_t channels = TS_CHAN_NONE;
    for (int i = 0; i < TS_NUM_CHANNELS; i++)
    {
        if (pTsHdl->chan[i].params.active)
        {
            channels |= (tsChannelsActive_t)( 1UL << i );
        }
    }
    return channels;
}

static uint32_t ts_channel_active_index(tsChannelsActive_t active, uint32_t chanIdx)
{
    uint32_t index = 0, ch = 0;

    if ((active & (1 << chanIdx)) == 0)
    {
        // Channel is not active
        index = TS_NUM_CHANNELS;
    }
    else
    {
        while (ch != chanIdx)
        {
            if ((1 << ch) & active)
            {
                // Count active channels before requested channel
                index++;
            }
            ch++;
        }
    }
    return index;
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
    ts_channel_t* pTsHdl = (ts_channel_t*)tsChannels;

    pTsHdl->status.adc_sync = (litepcie_readl(pTsHdl->ctrl_handle, CSR_ADC_STATUS_ADDR) & (1 << CSR_ADC_STATUS_FRAME_SYNC_OFFSET)) ? 1 : 0;

    //Update XADC values
    ts_channel_health_update(pTsHdl);

    //Update Clock Status
    int32_t clock_status =  mcp_clkgen_status(pTsHdl->pll.clkGen, TS_PLL_STATUS, TS_PLL_STATUS_LEN);
    if(clock_status < 0)
    {
        LOG_ERROR("Failed to read PLL Clock Status %d", clock_status);
    }
    else
    {
        pTsHdl->status.local_osc_clk = (clock_status & (1 << TS_PLL_STATUS_IC2_VALID)) ? 1:0;
        pTsHdl->status.ref_in_clk = (clock_status & (1 << TS_PLL_STATUS_IC1_VALID)) ? 1:0;
        pTsHdl->status.pll_lock = (clock_status & (1 << TS_PLL_STATUS_APLL_LOCK)) ? 1:0;
        pTsHdl->status.pll_low = (clock_status & (1 << TS_PLL_STATUS_APLL_LOW)) ? 1:0;
        pTsHdl->status.pll_high = (clock_status & (1 << TS_PLL_STATUS_APLL_HIGH)) ? 1:0;
        pTsHdl->status.pll_alt = (clock_status & (1 << TS_PLL_STATUS_APLL_ALT)) ? 1:0;
    }
    return pTsHdl->status;
}

int32_t ts_channel_sample_rate_set(tsChannelHdl_t tsChannels, uint32_t rate, tsSampleFormat_t mode)
{
    if(tsChannels == NULL)
    {
        return TS_STATUS_ERROR;
    }
    ts_channel_t* ts =  (ts_channel_t*)tsChannels;
    uint64_t actual_rate = 0;
    uint64_t max_rate = 0;


    switch(mode)
    {
    case TS_8_BIT:
    {
        max_rate = TS_MAX_8BIT_SAMPLE_RATE;
        ts->status.adc_sample_resolution = 256;
        break;
    }
    case TS_12_BIT_LSB:
    {
        max_rate = TS_MAX_12BIT_SAMPLE_RATE;
        ts->status.adc_sample_resolution = 4096;
        break;
    }
    case TS_12_BIT_MSB:
    {
        max_rate = TS_MAX_12BIT_SAMPLE_RATE;
        ts->status.adc_sample_resolution = 65536;
        break;
    }
    case TS_14_BIT:
    {
        max_rate = TS_MAX_14BIT_SAMPLE_RATE;
        ts->status.adc_sample_resolution = 65536;
        break;
    }
    default:
        return TS_INVALID_PARAM;
    }

    if((rate < TS_MIN_SAMPLE_RATE) || (rate > max_rate))
    {
        return TS_INVALID_PARAM;
    }

    ts->sampleMode = mode;

    // Use 1:1 rate for precision mode (14_bit)
    if((ts->adc.adcDev.mode == HMCAD15_SINGLE_CHANNEL) || (ts->sampleMode == TS_14_BIT))
    {
        actual_rate = rate;
    }
    else if(ts->adc.adcDev.mode == HMCAD15_DUAL_CHANNEL)
    {
        if(rate > (max_rate/2))
        {
            rate = (max_rate/2);
        }
        actual_rate = rate * 2;
    }
    else
    {
        if(rate > (max_rate/4))
        {
            rate = (max_rate/4);
        }
        actual_rate = rate * 4;
    }

    ts_adc_run(&ts->adc, 0);

    if(actual_rate != ts->pll.clkConf.out_clks[TS_PLL_SAMPLE_CLK_IDX].output_freq)
    {
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
    }

    ts->status.adc_sample_rate = rate;
    ts->status.adc_sample_bits = (mode == TS_8_BIT) ? 8 : 16;

    ts_adc_set_sample_mode(&ts->adc, rate, mode);

    for (int i = 0; i < TS_CAL_NUM_LOADS; i++)
    {
        if(ts->adcCal.branchFineGain[i].channels == ts_channel_active_get(ts))
        {
            for (int j = 0; j < TS_CAL_NUM_RATES; j++)
            {
                if(ts->adcCal.branchFineGain[i].conf[j].rate == rate)
                {
                    uint8_t fineGain[8] = {0};
                    for (int k = 0; k < 8; k++)
                    {
                        fineGain[k] = (uint8_t)ts->adcCal.branchFineGain[i].conf[j].gain[k];
                    }
                    ts_adc_cal_set(&ts->adc, fineGain);
                    break;
                }
            }
            break;
        }
    }

    ts_adc_run(&ts->adc, ts->status.adc_state);

    return  TS_STATUS_OK;
}

int32_t ts_channel_ext_clock_config(tsChannelHdl_t tsChannels, tsRefClockMode_t mode, uint32_t refclk_freq)
{
    if(tsChannels == NULL)
    {
        return TS_STATUS_ERROR;
    }
    ts_channel_t* ts =  (ts_channel_t*)tsChannels;
    zl3026x_clk_config_t newConf = ts->pll.clkConf;
    bool clkout_en = (mode == TS_REFCLK_OUT);
    bool clkin_en = (mode == TS_REFCLK_IN);
    uint8_t clkin_divider = 0;
    uint32_t input_freq = TS_PLL_LOCAL_OSC_RATE;

    //Validate Settings
    if (clkout_en && refclk_freq == 0)
    {
        LOG_ERROR("Invalid Clock Out Frequency, cannot be 0");
        return TS_INVALID_PARAM;
    }
    else if(clkin_en && refclk_freq < ZL3026X_INPUT_CLK_MIN)
    {
        LOG_ERROR("Invalid Clock Input Frequency %d Hz.  Must be a minimum of %d Hz", refclk_freq, ZL3026X_INPUT_CLK_MIN);
        return TS_INVALID_PARAM;
    }

    if(clkin_en)
    {
        //Divide Input Clock Frequency if needed
        while((refclk_freq / (1UL << clkin_divider)) > ZL3026X_INPUT_CLK_MAX)
        {
            clkin_divider++;
            if(clkin_divider > ZL3026X_IN_DIV_8)
            {
                LOG_ERROR("Unable to configure external clock input frequency %d Hz", refclk_freq);
                return TS_INVALID_PARAM;
            }
        }

        //Set Input Clock Configuration
        newConf.in_clks[TS_PLL_REFIN_IDX].enable = 1;
        newConf.in_clks[TS_PLL_REFIN_IDX].input_freq = refclk_freq / (1 << clkin_divider);
        newConf.in_clks[TS_PLL_REFIN_IDX].input_divider = (zl3026x_input_div_t)clkin_divider;
        newConf.input_select = TS_PLL_REFIN_SEL;
        newConf.alternate_select = TS_PLL_LOCAL_OSC_SEL;

        //Input frequency on bypass path
        input_freq = newConf.in_clks[TS_PLL_REFIN_IDX].input_freq;
    }
    else
    {
        //Use Internal Reference Clock
        newConf.in_clks[TS_PLL_REFIN_IDX].enable = 0;
        newConf.input_select = TS_PLL_LOCAL_OSC_SEL;
        newConf.alternate_select = TS_PLL_INPUT_NONE_SEL;
    }

    //Set Output Clock Configuration
    if(clkout_en)
    {
        //Validate Output Clock Frequency
        if(refclk_freq > ZL3026X_MAX_PLL_OUT)
        {
            LOG_ERROR("Invalid Clock Output Frequency %d Hz.  Must be a maximum of %d Hz", refclk_freq, ZL3026X_MAX_PLL_OUT);
            return TS_INVALID_PARAM;
        }

        newConf.out_clks[TS_PLL_REFOUT_CLK_IDX].enable = 1;
        newConf.out_clks[TS_PLL_REFOUT_CLK_IDX].output_freq = refclk_freq;
        if(refclk_freq > input_freq)
        {
            newConf.out_clks[TS_PLL_REFOUT_CLK_IDX].output_pll_select = ZL3026X_PLL_INT_DIV;
        }
        else
        {
            newConf.out_clks[TS_PLL_REFOUT_CLK_IDX].output_pll_select = ZL3026X_PLL_BYPASS;
        }
    }
    else
    {
        //Disable Output Ref Clock
        newConf.out_clks[TS_PLL_REFOUT_CLK_IDX].enable = 0;
    }

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

    return TS_STATUS_OK;
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
    LOG_DEBUG("\tAttenuator Scale               %.03f", cal->attenuatorScale);
    LOG_DEBUG("\tHigh Gain PGA");
    for (int path = 0; path < TS_CAL_NUM_PATHS; path++)
    {
        LOG_DEBUG("\tPGA Attenuator: %d", path);
        LOG_DEBUG("\t\tBuffer Vpp:                  %.03f V", cal->highPgaPathCal[path].bufferInputVpp);
        LOG_DEBUG("\t\tTrim Rheostat:               %d", cal->highPgaPathCal[path].trimDPot);
        LOG_DEBUG("\t\tTrim DAC Scale:              %.03f", cal->highPgaPathCal[path].trimOffsetDacScale);
        LOG_DEBUG("\t\tTrim DAC Zero:               %.03f", cal->highPgaPathCal[path].trimOffsetDacZeroC);
        LOG_DEBUG("\t\tTrim DAC Zero Slope:         %.03f", cal->highPgaPathCal[path].trimOffsetDacZeroM);
    }
    LOG_DEBUG("\tLow Gain PGA");
    for (int path = 0; path < TS_CAL_NUM_PATHS; path++)
    {
        LOG_DEBUG("\tPGA Attenuator: %d", path);
        LOG_DEBUG("\t\tBuffer Vpp:                  %.03f V", cal->lowPgaPathCal[path].bufferInputVpp);
        LOG_DEBUG("\t\tTrim Rheostat:               %d", cal->lowPgaPathCal[path].trimDPot);
        LOG_DEBUG("\t\tTrim DAC Scale:              %.03f", cal->lowPgaPathCal[path].trimOffsetDacScale);
        LOG_DEBUG("\t\tTrim DAC Zero:               %.03f", cal->lowPgaPathCal[path].trimOffsetDacZeroC);
        LOG_DEBUG("\t\tTrim DAC Zero Slope:         %.03f", cal->lowPgaPathCal[path].trimOffsetDacZeroM);
    }

    //Force afe to recalculate gain/offsets
    ts_channel_update_params(ts, chanIdx, &ts->chan[chanIdx].params, true);

    return TS_STATUS_OK;
}

int32_t ts_channel_calibration_get(tsChannelHdl_t tsChannels, uint32_t chanIdx, tsChannelCalibration_t* cal)
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

    *cal = ts->chan[chanIdx].afe.cal;

    return TS_STATUS_OK;
}

int32_t ts_channel_adc_calibration_set(tsChannelHdl_t tsChannels, tsAdcCalibration_t* cal)
{
    ts_channel_t* ts =  (ts_channel_t*)tsChannels;
    if(tsChannels == NULL || cal == NULL)
    {
        LOG_ERROR("Invalid handle");
        return TS_STATUS_ERROR;
    }
    
    ts->adcCal = *cal;
    
    ts_channel_sample_rate_set(tsChannels, ts->status.adc_sample_rate, ts->sampleMode);

    return TS_STATUS_OK;
}

int32_t ts_channel_adc_calibration_get(tsChannelHdl_t tsChannels, tsAdcCalibration_t* cal)
{
    ts_channel_t* ts =  (ts_channel_t*)tsChannels;
    if(tsChannels == NULL || cal == NULL)
    {
        LOG_ERROR("Invalid handle");
        return TS_STATUS_ERROR;
    }

    *cal = ts->adcCal;

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
    preamp.preamp = ctrl.pga_high_gain == 0 ? PREAMP_LG : PREAMP_HG;
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

int32_t ts_channel_calibration_manual_fine_gain(tsChannelHdl_t tsChannels, uint8_t fineGain[8])
{
    ts_channel_t* ts =  (ts_channel_t*)tsChannels;
    if(tsChannels == NULL || fineGain == NULL)
    {
        LOG_ERROR("Invalid handle");
        return TS_STATUS_ERROR;
    }
        
    return ts_adc_cal_set(&ts->adc, fineGain);
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
    pTsHdl->status.sys_health.frontend_power_good = (uint8_t)litepcie_readl(pTsHdl->ctrl_handle, CSR_FRONTEND_STATUS_ADDR) & (1 << CSR_FRONTEND_STATUS_FE_PG_OFFSET);
    pTsHdl->status.sys_health.acq_power_good = (uint8_t)litepcie_readl(pTsHdl->ctrl_handle, CSR_ADC_STATUS_ADDR) & (1 << CSR_ADC_STATUS_ACQ_PG_OFFSET);

    return TS_STATUS_OK;
}