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

#include "thunderscope.h"
#include "platform.h"
#include "spi.h"
#include "i2c.h"
#include "gpio.h"
#include "afe.h"
#include "adc.h"
#include "util.h"


#define TS_ADC_CH_NO_INVERT     (0)
#define TS_ADC_CH_INVERT        (1)

typedef struct ts_channel_s {
    struct {
        uint8_t channelNo;
        tsChannelParam_t params;
        ts_afe_t afe;
    } chan[TS_NUM_CHANNELS];
    ts_adc_t adc;
    spi_bus_t spibus;
    struct {
        i2c_t clkGen;
        gpio_t nRst;
    }pll;
    gpio_t afe_power;
    gpio_t acq_power;
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
    uint8_t adc_input;
    uint8_t adc_invert;
} g_channelConf[TS_NUM_CHANNELS] = {
    // Channel 1
    {
        TS_AFE_0_AMP_CS,
        TS_AFE_0_TERM_REG,     TS_AFE_0_TERM_MASK,
        TS_AFE_0_COUPLING_REG, TS_AFE_0_COUPLING_MASK,
        TS_AFE_0_ATTEN_REG,    TS_AFE_0_ATTEN_MASK,
        HMCAD15_ADC_IN1,       TS_ADC_CH_INVERT

    },
    // Channel 2
    {
        TS_AFE_1_AMP_CS,
        TS_AFE_1_TERM_REG,     TS_AFE_1_TERM_MASK,
        TS_AFE_1_COUPLING_REG, TS_AFE_1_COUPLING_MASK,
        TS_AFE_1_ATTEN_REG,    TS_AFE_1_ATTEN_MASK,
        HMCAD15_ADC_IN2,       TS_ADC_CH_INVERT
    },
    // Channel 3
    {
        TS_AFE_2_AMP_CS,
        TS_AFE_2_TERM_REG,     TS_AFE_2_TERM_MASK,
        TS_AFE_2_COUPLING_REG, TS_AFE_2_COUPLING_MASK,
        TS_AFE_2_ATTEN_REG,    TS_AFE_2_ATTEN_MASK,
        HMCAD15_ADC_IN3,       TS_ADC_CH_INVERT
    },
    // Channel 4
    {
        TS_AFE_3_AMP_CS,
        TS_AFE_3_TERM_REG,     TS_AFE_3_TERM_MASK,
        TS_AFE_3_COUPLING_REG, TS_AFE_3_COUPLING_MASK,
        TS_AFE_3_ATTEN_REG,    TS_AFE_3_ATTEN_MASK,
        HMCAD15_ADC_IN4,       TS_ADC_CH_INVERT
    }
};

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

    //Enable Power Rails
    pChan->afe_power.fd = ts;
    pChan->afe_power.reg = TS_AFE_POWER_REG;
    pChan->afe_power.bit_mask = TS_AFE_POWER_MASK;
    gpio_set(pChan->afe_power);

    pChan->acq_power.fd = ts;
    pChan->acq_power.reg = TS_ACQ_POWER_REG;
    pChan->acq_power.bit_mask = TS_ACQ_POWER_MASK;
    gpio_set(pChan->acq_power);


    //Initialize PLL Clock Gen
    // Toggle reset pin
    pChan->pll.nRst.fd = ts;
    pChan->pll.nRst.reg = TS_PLL_NRST_ADDR;
    pChan->pll.nRst.bit_mask = TS_PLL_NRST_MASK;
    gpio_clear(pChan->pll.nRst);
    //sleep 10 ms
    NS_DELAY(10000000);
    gpio_set(pChan->pll.nRst);
    NS_DELAY(10000000);

    pChan->pll.clkGen.fd = ts;
    pChan->pll.clkGen.devAddr = TS_PLL_I2C_ADDR;
    // retVal = mcp_clkgen_config(pChan->pll.clkGen, TS_PLL_CONF, TS_PLL_CONF_SIZE);
    if(retVal != TS_STATUS_OK)
    {
        goto channel_init_error;
    }


    //TODO: Placeholder. Replace with DAC and DPot driver instances?
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
                                afe_amp, trimDac, trimPot,
                                afe_term, afe_atten, afe_coupling);
        if(retVal != TS_STATUS_OK)
        {
            goto channel_init_error;
        }
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

    return ts_adc_run(&pChan->adc, en);

}

int32_t ts_channel_params_set(tsChannelHdl_t tsChannels, uint32_t chanIdx, tsChannelParam_t* param)
{
    int32_t retVal = TS_STATUS_OK;

    if(tsChannels == NULL || param == NULL)
    {
        return TS_STATUS_ERROR;
    }

    if(chanIdx >= TS_NUM_CHANNELS)
    {
        return TS_INVALID_PARAM;
    }

    ts_channel_t* pInst = (ts_channel_t*)tsChannels;

    //Set AFE Bandwidth
    if(param->bandwidth != pInst->chan[chanIdx].params.bandwidth)
    {
        retVal = ts_afe_set_bw_filter(&pInst->chan[chanIdx].afe, param->bandwidth);
        if(retVal > 0)
        {
            pInst->chan[chanIdx].params.bandwidth = retVal;
        }
        else
        {
            LOG_ERROR("Unable to set Channel %d bandwidth %d", chanIdx, retVal);
            return TS_INVALID_PARAM;
        }
    }

    //Set AC/DC Coupling
    if(param->coupling != pInst->chan[chanIdx].params.coupling)
    {
        if(TS_STATUS_OK == ts_afe_coupling_control(&pInst->chan[chanIdx].afe, 
                                    (param->coupling == TS_COUPLE_DC ? 1 : 0)))
        {
            pInst->chan[chanIdx].params.coupling = param->coupling;
        }
        else
        {
            LOG_ERROR("Unable to set Channel %d AC/DC Coupling: %x", chanIdx, param->coupling);
            return TS_INVALID_PARAM;
        }
    }

    //Set Termination
    if(param->term != pInst->chan[chanIdx].params.term)
    {
        if(TS_STATUS_OK == ts_afe_termination_control(&pInst->chan[chanIdx].afe, 
                                    (param->term == TS_TERM_50 ? 1 : 0)))
        {
            pInst->chan[chanIdx].params.coupling = param->coupling;
        }
        else
        {
            LOG_ERROR("Unable to set Channel %d Termination: %x", chanIdx, param->coupling);
            return TS_INVALID_PARAM;
        }
    }

    //Set Voltage Scale
    if(param->volt_scale_mV != pInst->chan[chanIdx].params.volt_scale_mV)
    {
        //Calculate dB gain value
        //TODO: Set both AFE and ADC gain?
        int32_t afe_gain_mdB = (int32_t)(20000 * log10((double)param->volt_scale_mV / 700.0));
        retVal = ts_afe_set_gain(&pInst->chan[chanIdx].afe, afe_gain_mdB);
        if(TS_STATUS_ERROR == retVal)
        {
            LOG_ERROR("Unable to set Channel %d voltage scale: %x", chanIdx, param->coupling);
            return TS_INVALID_PARAM;
        }
        else
        {
            LOG_DEBUG("Channel %d AFE set to %i mdB gain", chanIdx, retVal);
            retVal = (int32_t)pow(10.0, (double)retVal/20000.0);
            pInst->chan[chanIdx].params.volt_scale_mV = retVal;
        }
    }

    //Set Voltage Offset
    //TODO

    //Set Active
    if(param->active != pInst->chan[chanIdx].params.active)
    {
        retVal = ts_adc_channel_enable(&pInst->adc, chanIdx, param->active);

        if(TS_STATUS_OK != retVal)
        {
            LOG_ERROR("Unable to %s Channel %d: %d", (param->active == 0 ? "disable" : "enable"),
                        chanIdx, retVal);
            return retVal;
        }
        else
        {
            LOG_DEBUG("Channel %d %s", chanIdx, (param->active == 0 ? "disabled" : "enabled"));
        }
    }

    return TS_STATUS_OK;
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
    //TODO: Update XADC values

    return ((ts_channel_t*)tsChannels)->status;
}

int32_t ts_channel_sample_rate_set(tsChannelHdl_t tsChannels, uint32_t rate, uint32_t resolution)
{
    if(tsChannels == NULL)
    {
        //Return empty state
        tsScopeState_t state = {0};
        return TS_STATUS_ERROR;
    }
    //Input validation
    //TODO - Support valid rate/resolution combinations
    if((rate != 1000000000) || (resolution != 256))
    {
        return TS_INVALID_PARAM;
    }
    //TODO - Apply resolution,rate configuration

    return  TS_STATUS_OK;
}
