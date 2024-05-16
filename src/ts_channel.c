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

#include "ts_channel.h"

#include "thunderscope.h"
#include "platform.h"
#include "spi.h"
#include "i2c.h"
#include "gpio.h"
#include "afe.h"
#include "adc.h"


#define TS_ADC_CH_NO_INVERT     (0)
#define TS_ADC_CH_INVERT        (1)

typedef struct ts_channel_s {
    struct {
        uint8_t channelNo;
        ts_afe_t afe;
    } chan[TS_NUM_CHANNELS];
    ts_adc_t adc;
    spi_bus_t spibus;
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
    retVal = ts_adc_init(&pChan->adc, adcDev);
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
    free(pChan);
    return retVal;
}

int32_t ts_channel_destroy(tsChannelHdl_t tsChannels)
{
    ts_channel_t* pChan = (ts_channel_t*)tsChannels;

    //TODO Any cleanup on AFE/ADC as needed
    ts_adc_shutdown(&pChan->adc);


    free(tsChannels);

    return TS_STATUS_OK;
}