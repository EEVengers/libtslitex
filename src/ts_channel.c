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

typedef struct ts_channel_s {
    uint8_t channelNo;
    ts_afe_t afe;
    spi_bus_t spibus;
} ts_channel_t;


struct ts_channel_conf_s {
    uint32_t afe_amp_cs;
    uint32_t afe_term_reg;
    uint32_t afe_term_mask;
    uint32_t afe_cpl_reg;
    uint32_t afe_cpl_mask;
    uint32_t afe_atten_reg;
    uint32_t afe_atten_mask;
} g_channelConf[TS_NUM_CHANNELS] = {
    // Channel 1
    {
        TS_AFE_0_AMP_CS,
        TS_AFE_0_TERM_REG,     TS_AFE_0_TERM_MASK,
        TS_AFE_0_COUPLING_REG, TS_AFE_0_COUPLING_MASK,
        TS_AFE_0_ATTEN_REG,    TS_AFE_0_ATTEN_MASK
    },
    // Channel 2
    {
        TS_AFE_1_AMP_CS,
        TS_AFE_1_TERM_REG,     TS_AFE_1_TERM_MASK,
        TS_AFE_1_COUPLING_REG, TS_AFE_1_COUPLING_MASK,
        TS_AFE_1_ATTEN_REG,    TS_AFE_1_ATTEN_MASK
    },
    // Channel 3
    {
        TS_AFE_2_AMP_CS,
        TS_AFE_2_TERM_REG,     TS_AFE_2_TERM_MASK,
        TS_AFE_2_COUPLING_REG, TS_AFE_2_COUPLING_MASK,
        TS_AFE_2_ATTEN_REG,    TS_AFE_2_ATTEN_MASK
    },
    // Channel 4
    {
        TS_AFE_3_AMP_CS,
        TS_AFE_3_TERM_REG,     TS_AFE_3_TERM_MASK,
        TS_AFE_3_COUPLING_REG, TS_AFE_3_COUPLING_MASK,
        TS_AFE_3_ATTEN_REG,    TS_AFE_3_ATTEN_MASK
    }
};

int32_t ts_channel_init(tsChannelHdl_t* pTsChannels, tsHandle_t ts)
{
    int32_t retVal = TS_STATUS_OK;

    if(pTsChannels == NULL)
    {
        retVal = TS_STATUS_ERROR;
        return retVal;
    }

    ts_channel_t* pChan = (ts_channel_t*)calloc(sizeof(ts_channel_t)*TS_NUM_CHANNELS,1);

    if(pChan == NULL)
    {
        retVal = TS_STATUS_ERROR;
        return retVal;
    }

    //TODO: Placeholder. Replace with DAC and DPot driver instances?
    i2c_t trimDac = {(file_t)ts, TS_TRIM_DAC_I2C_ADDR};
    i2c_t trimPot = {(file_t)ts, TS_TRIM_DPOT_I2C_ADDR};
    
    for(uint32_t chanIdx = 0; chanIdx < TS_NUM_CHANNELS; chanIdx++)
    {
        retVal = spi_bus_init(&pChan[chanIdx].spibus, (file_t)ts,
                                TS_SPI_BUS_BASE_ADDR, TS_SPI_BUS_CS_NUM);
        if(retVal != TS_STATUS_OK)
        {
            return retVal;
        }

        spi_dev_t afe_amp;
        retVal = spi_dev_init(&afe_amp, &pChan[chanIdx].spibus,
                                g_channelConf[chanIdx].afe_amp_cs);
        if(retVal != TS_STATUS_OK)
        {
            return retVal;
        }

        gpio_t afe_term = {(file_t)ts, g_channelConf[chanIdx].afe_term_reg,
                                g_channelConf[chanIdx].afe_term_mask};
        gpio_t afe_coupling = {(file_t)ts, g_channelConf[chanIdx].afe_cpl_reg,
                                g_channelConf[chanIdx].afe_cpl_mask};
        gpio_t afe_atten = {(file_t)ts, g_channelConf[chanIdx].afe_atten_reg,
                                g_channelConf[chanIdx].afe_atten_mask};

        retVal = ts_afe_init(&pChan[chanIdx].afe, chanIdx, 
                                afe_amp, trimDac, trimPot,
                                afe_term, afe_atten, afe_coupling);
        if(retVal != TS_STATUS_OK)
        {
            break;
        }
    }

    if(retVal == TS_STATUS_OK)
    {
        *pTsChannels = pChan;
    }

    return retVal;
}

int32_t ts_channel_destroy(tsChannelHdl_t tsChannels)
{
    //TODO Any cleanup on AFE/ADC as needed

    free(tsChannels);

    return TS_STATUS_OK;
}