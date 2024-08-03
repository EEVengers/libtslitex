/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Control the ADC in the Thunderscope LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#include <stddef.h>

#include "adc.h"
#include "platform.h"
#include "util.h"
#include "liblitepcie.h"


typedef enum adc_shuffle_e
{
    ADC_SHUFFLE_1CH = 0,
    ADC_SHUFFLE_2CH = 1,
    ADC_SHUFFLE_4CH = 2,
} adc_shuffle_t;


int32_t ts_adc_init(ts_adc_t* adc, spi_dev_t spi, file_t fd)
{
    int32_t retVal = TS_STATUS_ERROR;

    if(adc != NULL)
    {
        adc->ctrl = fd;
        retVal = hmcad15xx_init(&adc->adcDev, spi);
    }

    if(retVal == TS_STATUS_OK)
    {
        litepcie_writel(adc->ctrl, CSR_ADC_HAD1511_CONTROL_ADDR, 1 << CSR_ADC_HAD1511_CONTROL_FRAME_RST_OFFSET);
        litepcie_writel(adc->ctrl, CSR_ADC_HAD1511_DOWNSAMPLING_ADDR, 1);
        retVal = hmcad15xx_full_scale_adjust(&adc->adcDev, TS_ADC_FULL_SCALE_ADJUST_DEFAULT);
    }

    return retVal;
}

int32_t ts_adc_set_channel_conf(ts_adc_t* adc, uint8_t channel, uint8_t input, uint8_t invert)
{
    int32_t retVal = TS_STATUS_ERROR;

    if(adc != NULL)
    {
        retVal = TS_STATUS_OK;
        adc->tsChannels[channel].input = input;
        adc->tsChannels[channel].invert = invert;

        if(adc->tsChannels[channel].active)
        {
            for(uint8_t i = 0; i < TS_NUM_CHANNELS; i++)
            {
                if(adc->tsChannels[channel].input == adc->adcDev.channelCfg[i].input)
                {
                    adc->adcDev.channelCfg[i].invert = invert;
                    break;
                }
            }
            retVal = hmcad15xx_set_channel_config(&adc->adcDev);
            litepcie_writel(adc->ctrl, CSR_ADC_HAD1511_CONTROL_ADDR, 1 << CSR_ADC_HAD1511_CONTROL_FRAME_RST_OFFSET);
        }
    }

    return retVal;
}

int32_t ts_adc_set_gain(ts_adc_t* adc, uint8_t channel, int32_t gainCoarse, int32_t gainFine)
{
    int32_t retVal = TS_STATUS_OK;

    if(adc == NULL)
    {
        retVal = TS_STATUS_ERROR;
    }
    else
    {
        adc->tsChannels[channel].coarse = gainCoarse;
        adc->tsChannels[channel].fine = gainFine;

        for(uint8_t i = 0; i < TS_NUM_CHANNELS; i++)
        {
            if(adc->tsChannels[channel].input == adc->adcDev.channelCfg[i].input)
            {
                adc->adcDev.channelCfg[i].coarse = gainCoarse;
                adc->adcDev.channelCfg[i].fine = gainFine;
                break;
            }
        }
        retVal = hmcad15xx_set_channel_config(&adc->adcDev);
        litepcie_writel(adc->ctrl, CSR_ADC_HAD1511_CONTROL_ADDR, 1 << CSR_ADC_HAD1511_CONTROL_FRAME_RST_OFFSET);
    }

    if(retVal == TS_STATUS_OK)
    {
        retVal = hmcad15xx_full_scale_adjust(&adc->adcDev, TS_ADC_FULL_SCALE_ADJUST_DEFAULT);
    }

    return retVal;
}

int32_t ts_adc_channel_enable(ts_adc_t* adc, uint8_t channel, uint8_t enable)
{
    int32_t retVal;
    uint8_t activeCount = 0;
    adc_shuffle_t shuffleMode = ADC_SHUFFLE_1CH;

    adc->tsChannels[channel].active = enable;

    for(uint8_t i=0; i < TS_NUM_CHANNELS; i++)
    {
        if(adc->tsChannels[i].active)
        {
            // Copy Active Channel Configs to ADC in order
            LOG_DEBUG("Enabling IN %d as CH %d", activeCount, i);
            adc->adcDev.channelCfg[activeCount] = adc->tsChannels[i];
            activeCount++;
        }
    }

    //Disable Unused channels in config
    for(uint8_t i=activeCount; i < HMCAD15_NUM_CHANNELS; i++)
    {
        LOG_DEBUG("Disable CH %d", i);

        adc->adcDev.channelCfg[i].active = 0;
    }

    if(activeCount == 0)
    {
        // Put ADC to sleep
        retVal = hmcad15xx_power_mode(&adc->adcDev, HMCAD15_CH_SLEEP);
    }
    else 
    {

        if(activeCount == 1)
        {
            adc->adcDev.mode = HMCAD15_SINGLE_CHANNEL;
            shuffleMode = ADC_SHUFFLE_1CH;
        }
        else if(activeCount == 2)
        {
            adc->adcDev.mode = HMCAD15_DUAL_CHANNEL;
            shuffleMode = ADC_SHUFFLE_2CH;
        }
        else
        {
            adc->adcDev.mode = HMCAD15_QUAD_CHANNEL;
            shuffleMode = ADC_SHUFFLE_4CH;
        }
        retVal = hmcad15xx_set_channel_config(&adc->adcDev);
        
        litepcie_writel(adc->ctrl, CSR_ADC_HAD1511_CONTROL_ADDR, 1 << CSR_ADC_HAD1511_CONTROL_FRAME_RST_OFFSET);
        litepcie_writel(adc->ctrl, CSR_ADC_HAD1511_DATA_CHANNELS_ADDR, shuffleMode << CSR_ADC_HAD1511_DATA_CHANNELS_SHUFFLE_OFFSET);
    }

    return retVal;
}

int32_t ts_adc_shutdown(ts_adc_t* adc)
{
    int32_t retVal = TS_STATUS_ERROR;
    
    if(adc)
    {
        retVal = hmcad15xx_reset(&adc->adcDev);
    }

    if(retVal == TS_STATUS_OK)
    {
        hmcad15xx_power_mode(&adc->adcDev, HMCAD15_CH_POWERDN);
        litepcie_writel(adc->ctrl, CSR_ADC_HAD1511_CONTROL_ADDR, 1 << CSR_ADC_HAD1511_CONTROL_FRAME_RST_OFFSET);
    }

    return retVal; 
}

int32_t ts_adc_run(ts_adc_t* adc, uint8_t en)
{
    if(!adc)
    {
        return TS_STATUS_ERROR;
    }
    //Enable Trigger
    litepcie_writel(adc->ctrl, CSR_ADC_TRIGGER_CONTROL_ADDR, en);
    return TS_STATUS_OK;
}