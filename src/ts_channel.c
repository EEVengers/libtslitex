/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Channel management functions for the Thunderscope
 * LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#include "ts_channel.h"
#include "spi.h"
#include "i2c.h"
#include "gpio.h"

static spi_bus_t ts_spibus;

int32_t ts_channel_init_all(tsHandle_t ts)
{
    int32_t retVal = TS_STATUS_OK;

    retVal = spi_bus_init(&ts_spibus, (file_t)ts, TS_SPI_BUS_BASE_ADDR);
    if(retVal != TS_STATUS_OK)
    {
        return retVal;
    }

    i2c_t trimDac = {(file_t)ts, TS_TRIM_DAC_I2C_ADDR};
    i2c_t trimPot = {(file_t)ts, TS_TRIM_DPOT_I2C_ADDR};
    
    spi_dev_t afe0_amp;
    retVal = spi_dev_init(&afe0_amp, &ts_spibus, TS_AFE_0_AMP_CS);
    if(retVal != TS_STATUS_OK)
    {
        return retVal;
    }

    gpio_t afe0_term = {(file_t)ts, TS_AFE_0_TERM_REG, TS_AFE_0_TERM_MASK};
    gpio_t afe0_coupling = {(file_t)ts, TS_AFE_0_COUPLING_REG, TS_AFE_0_COUPLING_MASK};
    gpio_t afe0_atten = {(file_t)ts, TS_AFE_0_ATTEN_REG, TS_AFE_0_ATTEN_MASK};

    retVal = ts_afe_init_channel(0, afe0_amp, trimDac, trimPot, afe0_term, afe0_atten, afe0_coupling);
    if(retVal != TS_STATUS_OK)
    {
        return retVal;
    }

    spi_dev_t afe1_amp;
    retVal = spi_dev_init(&afe1_amp, &ts_spibus, TS_AFE_1_AMP_CS);
    if(retVal != TS_STATUS_OK)
    {
        return retVal;
    }
    
    gpio_t afe1_term = {(file_t)ts, TS_AFE_1_TERM_REG, TS_AFE_1_TERM_MASK};
    gpio_t afe1_coupling = {(file_t)ts, TS_AFE_1_COUPLING_REG, TS_AFE_1_COUPLING_MASK};
    gpio_t afe1_atten = {(file_t)ts, TS_AFE_1_ATTEN_REG, TS_AFE_1_ATTEN_MASK};

    retVal = ts_afe_init_channel(1, afe1_amp, trimDac, trimPot, afe1_term, afe1_atten, afe1_coupling);
    if(retVal != TS_STATUS_OK)
    {
        return retVal;
    }

    spi_dev_t afe2_amp;
    retVal = spi_dev_init(&afe2_amp, &ts_spibus, TS_AFE_2_AMP_CS);
    if(retVal != TS_STATUS_OK)
    {
        return retVal;
    }
    
    gpio_t afe2_term = {(file_t)ts, TS_AFE_2_TERM_REG, TS_AFE_2_TERM_MASK};
    gpio_t afe2_coupling = {(file_t)ts, TS_AFE_2_COUPLING_REG, TS_AFE_2_COUPLING_MASK};
    gpio_t afe2_atten = {(file_t)ts, TS_AFE_2_ATTEN_REG, TS_AFE_2_ATTEN_MASK};

    retVal = ts_afe_init_channel(2, afe2_amp, trimDac, trimPot, afe2_term, afe2_atten, afe2_coupling);
    if(retVal != TS_STATUS_OK)
    {
        return retVal;
    }

    spi_dev_t afe3_amp;
    retVal = spi_dev_init(&afe3_amp, &ts_spibus, TS_AFE_3_AMP_CS);
    if(retVal != TS_STATUS_OK)
    {
        return retVal;
    }
    
    gpio_t afe3_term = {(file_t)ts, TS_AFE_3_TERM_REG, TS_AFE_3_TERM_MASK};
    gpio_t afe3_coupling = {(file_t)ts, TS_AFE_3_COUPLING_REG, TS_AFE_3_COUPLING_MASK};
    gpio_t afe3_atten = {(file_t)ts, TS_AFE_3_ATTEN_REG, TS_AFE_3_ATTEN_MASK};

    retVal = ts_afe_init_channel(3, afe3_amp, trimDac, trimPot, afe3_term, afe3_atten, afe3_coupling);

    return retVal;
}
