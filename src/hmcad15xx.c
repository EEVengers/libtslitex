/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * A driver for the HMCAD1511/HMCAD1520 as used
 * for the Thunderscope ADC.  Datasheet reference at 
 * https://www.analog.com/media/en/technical-documentation/data-sheets/hmcad1520.pdf 
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#include <stdint.h>

#include "ts_common.h"
#include "hmcad15xx.h"
#include "spi.h"
#include "util.h"

static void hmcad15xxRegWrite(hmcad15xxADC_t* adc, uint8_t reg, uint16_t data);
static void hmcad15xxApplyLvdsMode(hmcad15xxADC_t* adc);
static void hmcad15xxApplySampleMode(hmcad15xxADC_t* adc);
static void hmcad15xxApplyChannelMap(hmcad15xxADC_t* adc);
static void hmcad15xxApplyChannelGain(hmcad15xxADC_t* adc);

int32_t hmcad15xx_init(hmcad15xxADC_t* adc, spi_dev_t dev)
{
    if(!adc)
    {
        return TS_STATUS_ERROR;
    }

    adc->dev = dev;
    
    // Set initial configuration
    adc->mode = HMCAD15_SINGLE_CHANNEL;
    adc->width = HMCAD15_8_BIT;
    adc->channelCfg[0].active = 1;
    adc->channelCfg[0].input = HMCAD15_ADC_IN1;
    adc->channelCfg[0].invert = 1;
    adc->channelCfg[0].coarse = 0;
    adc->channelCfg[0].fine = 0;
    adc->clockDiv = HMCAD15_CLK_DIV_1;
    adc->fullScale_x10 = HMCAD15_FULL_SCALE_DEFAULT;
    adc->drive = HMCAD15_LVDS_DS_15;
    adc->lvdsPhase = HMCAD15_LVDS_PHASE_DEFAULT;

    //Reset
    hmcad15xx_reset(adc);

    //Power Down
    hmcad15xx_power_mode(adc, HMCAD15_CH_POWERDN);

    //LVDS Mode
    hmcad15xxApplyLvdsMode(adc);

    //Gain dB mode
    hmcad15xxRegWrite(adc, HMCAD15_REG_GAIN_SEL, 0);

    //Channel Conf
    hmcad15xxApplySampleMode(adc);
    hmcad15xxApplyChannelMap(adc);
    hmcad15xxApplyChannelGain(adc);
    
    //Set Sleep mode
    hmcad15xx_power_mode(adc, HMCAD15_CH_SLEEP);

    return TS_STATUS_OK;
}

int32_t hmcad15xx_reset(hmcad15xxADC_t* adc)
{
    if(!adc)
    {
        return TS_STATUS_ERROR;
    }

    hmcad15xxRegWrite(adc, HMCAD15_REG_SW_RST, 1);
    return TS_STATUS_OK;
}

int32_t hmcad15xx_power_mode(hmcad15xxADC_t* adc, hmcad15xxPower_t power)
{
    uint16_t data = 0;
    if(!adc)
    {
        return TS_STATUS_ERROR;
    }
    
    switch(power)
    {
        case HMCAD15_CH_ACTIVE:
            switch(adc->mode)
            {
                case HMCAD15_SINGLE_CHANNEL:
                    data |= adc->channelCfg[0].active == 0 ? 
                                HMCAD15_SINGLE_CH_SLP: 0;
                    break;
                case HMCAD15_DUAL_CHANNEL:
                    data |= adc->channelCfg[0].active == 0 ? 
                                HMCAD15_DUAL_CH_0_SLP : 0;
                    data |= adc->channelCfg[1].active == 0 ? 
                                HMCAD15_DUAL_CH_1_SLP : 0;
                    break;
                case HMCAD15_QUAD_CHANNEL:
                    data |= adc->channelCfg[0].active == 0 ? 
                                HMCAD15_QUAD_CH_0_SLP : 0;
                    data |= adc->channelCfg[1].active == 0 ? 
                                HMCAD15_QUAD_CH_1_SLP : 0;
                    data |= adc->channelCfg[2].active == 0 ? 
                                HMCAD15_QUAD_CH_2_SLP : 0;
                    data |= adc->channelCfg[3].active == 0 ?
                                HMCAD15_QUAD_CH_3_SLP : 0;
                    break;
                default:
                    break;
            }
            break;
        case HMCAD15_CH_SLEEP:
            data = HMCAD15_PWR_MODE_SLEEP;
            break;
        case HMCAD15_CH_POWERDN:
        default:
            data = HMCAD15_PWR_MODE_POWERDN;
            break;
    }

    hmcad15xxRegWrite(adc, HMCAD15_REG_POWER_CTRL, data);
    return TS_STATUS_OK;
}

int32_t hmcad15xx_set_channel_config(hmcad15xxADC_t* adc)
{
    uint16_t data = 0;
    if(!adc)
    {
        return TS_STATUS_ERROR;
    }

    //Power Down
    hmcad15xx_power_mode(adc, HMCAD15_CH_POWERDN);

    //Change Mode
    hmcad15xxApplySampleMode(adc);
    hmcad15xxApplyChannelGain(adc);

    //Power Up
    hmcad15xx_power_mode(adc, HMCAD15_CH_ACTIVE);

    //Select Inputs
    hmcad15xxApplyChannelMap(adc);

    return TS_STATUS_OK;
}

int32_t hmcad15xx_full_scale_adjust(hmcad15xxADC_t* adc, int8_t adjustment)
{
    if(!adc)
    {
        return TS_STATUS_ERROR;
    }

    if((HMCAD15_FULL_SCALE_MAX < adjustment) ||
        (adjustment < HMCAD15_FULL_SCALE_MIN))
    {
        return TS_INVALID_PARAM;   
    }

    hmcad15xxRegWrite(adc, HMCAD15_REG_ADC_FULL_SCALE, 
            HMCAD15_FULL_SCALE_SET(adjustment));
    
    adc->fullScale_x10 = adjustment;

    return TS_STATUS_OK;
}

int32_t hmcad15xx_set_test_pattern(hmcad15xxADC_t* adc, hmcad15xxTestMode_t mode, uint16_t testData1, uint16_t testData2)
{
    if(!adc)
    {
        return TS_STATUS_ERROR;
    }
    //Clear both test modes
    hmcad15xxRegWrite(adc, HMCAD15_REG_TEST_MODE, 0);
    hmcad15xxRegWrite(adc, HMCAD15_REG_PAT_MODE, 0);

    switch(mode)
    {
        case HMCAD15_TEST_DISABLE:
        //Test mode already cleared. Nothing else to do.
        break;
        case HMCAD15_TEST_RAMP:
        hmcad15xxRegWrite(adc, HMCAD15_REG_TEST_MODE, HMCAD15_TEST_MODE_RAMP);
        break;
        case HMCAD15_TEST_SINGLE:
        hmcad15xxRegWrite(adc, HMCAD15_REG_TEST_PAT1, testData1);
        hmcad15xxRegWrite(adc, HMCAD15_REG_TEST_MODE, HMCAD15_TEST_MODE_SINGLE);
        break;
        case HMCAD15_TEST_DUAL:
        hmcad15xxRegWrite(adc, HMCAD15_REG_TEST_PAT1, testData1);
        hmcad15xxRegWrite(adc, HMCAD15_REG_TEST_PAT2, testData2);
        hmcad15xxRegWrite(adc, HMCAD15_REG_TEST_MODE, HMCAD15_TEST_MODE_DUAL);
        break;
        case HMCAD15_TEST_DESKEW:
        hmcad15xxRegWrite(adc, HMCAD15_REG_PAT_MODE, HMCAD15_TEST_PAT_DESKEW);
        break;
        case HMCAD15_TEST_SYNC:
        hmcad15xxRegWrite(adc, HMCAD15_REG_PAT_MODE, HMCAD15_TEST_PAT_SYNC);
        break;
        default:
        LOG_ERROR("Invalid Test Mode");
    }

    return TS_STATUS_OK;
}

static void hmcad15xxRegWrite(hmcad15xxADC_t* adc, uint8_t reg, uint16_t data)
{
    uint8_t bytes[2];
    bytes[0] = (data >> 8) & 0xFF;
    bytes[1] = data & 0xFF;
    spi_busy_wait(adc->dev);
    spi_write(adc->dev, reg, bytes, 2);
    LOG_DEBUG("hmcad SPI R: 0x%02X V: 0x%04X", reg, data);
}

static void hmcad15xxApplyLvdsMode(hmcad15xxADC_t* adc)
{
    uint16_t data = 0;

    // Set LVDS Drive Strength
    data = (HMCAD15_LVDS_DS_LCLK(adc->drive) |
            HMCAD15_LVDS_DS_FRAME(adc->drive) |
            HMCAD15_LVDS_DS_DATA(adc->drive));
    hmcad15xxRegWrite(adc, HMCAD15_REG_LVDS_CURRENT, data);

    // Set LVDS Data Width (bits per sample)
    data = HMCAD15_DATA_WIDTH(adc->width);
    hmcad15xxRegWrite(adc, HMCAD15_REG_LVDS_MISC, data);

    // Set LVDS DDR Phase
    data = HMCAD15_LVDS_PHASE(adc->lvdsPhase);
    hmcad15xxRegWrite(adc, HMCAD15_REG_LCLK_PHASE, data);
}

static void hmcad15xxApplySampleMode(hmcad15xxADC_t* adc)
{
    uint16_t data = 0;

    switch(adc->mode)
    {
        case HMCAD15_SINGLE_CHANNEL:
            adc->clockDiv = HMCAD15_CLK_DIV_1;
            data = HMCAD15_SAMPLE_MODE_SET(adc->mode) |
                    HMCAD15_CLK_DIV_SET(adc->clockDiv);
            break;
        case HMCAD15_DUAL_CHANNEL:
            adc->clockDiv = HMCAD15_CLK_DIV_2;
            data = HMCAD15_SAMPLE_MODE_SET(adc->mode) |
                    HMCAD15_CLK_DIV_SET(adc->clockDiv);
            break;
        case HMCAD15_QUAD_CHANNEL:
        case HMCAD15_14BIT_QUAD_CHANNEL:
            adc->clockDiv = HMCAD15_CLK_DIV_4;
            data = HMCAD15_SAMPLE_MODE_SET(adc->mode) |
                    HMCAD15_CLK_DIV_SET(adc->clockDiv);
            break;
    }
    hmcad15xxRegWrite(adc, HMCAD15_REG_CHAN_MODE, data);
}

static void hmcad15xxApplyChannelMap(hmcad15xxADC_t* adc)
{
    uint16_t in12 = 0, in34 = 0, inv = 0;

    switch(adc->mode)
    {
        case HMCAD15_SINGLE_CHANNEL:
            in12 = HMCAD15_SEL_CH_1(adc->channelCfg[0].input);
            in12 |= HMCAD15_SEL_CH_2(adc->channelCfg[0].input);
            in34 = HMCAD15_SEL_CH_3(adc->channelCfg[0].input);
            in34 |= HMCAD15_SEL_CH_4(adc->channelCfg[0].input);
            inv = HMCAD15_CH_INVERT_S1(adc->channelCfg[0].invert);
            break;
        case HMCAD15_DUAL_CHANNEL:
            in12 = HMCAD15_SEL_CH_1(adc->channelCfg[0].input);
            in12 |= HMCAD15_SEL_CH_2(adc->channelCfg[0].input);
            in34 = HMCAD15_SEL_CH_3(adc->channelCfg[1].input);
            in34 |= HMCAD15_SEL_CH_4(adc->channelCfg[1].input);
            inv = HMCAD15_CH_INVERT_D1(adc->channelCfg[0].invert) | 
                    HMCAD15_CH_INVERT_D2(adc->channelCfg[1].invert);
            break;
        case HMCAD15_QUAD_CHANNEL:
            in12 = HMCAD15_SEL_CH_1(adc->channelCfg[0].input);
            in12 |= HMCAD15_SEL_CH_2(adc->channelCfg[1].input);
            in34 = HMCAD15_SEL_CH_3(adc->channelCfg[2].input);
            in34 |= HMCAD15_SEL_CH_4(adc->channelCfg[3].input);
            inv = (HMCAD15_CH_INVERT_Q1(adc->channelCfg[0].invert) | 
                    HMCAD15_CH_INVERT_Q2(adc->channelCfg[1].invert) |
                    HMCAD15_CH_INVERT_Q3(adc->channelCfg[2].invert) |
                    HMCAD15_CH_INVERT_Q4(adc->channelCfg[3].invert));
        break;
    }

    hmcad15xxRegWrite(adc, HMCAD15_REG_IN_SEL_1_2, in12);
    hmcad15xxRegWrite(adc, HMCAD15_REG_IN_SEL_3_4, in34);
    hmcad15xxRegWrite(adc, HMCAD15_REG_CHAN_INVERT, inv);
}

static void hmcad15xxApplyChannelGain(hmcad15xxADC_t* adc)
{
    uint16_t cgain, fgain = 0;

    switch(adc->mode)
    {
        case HMCAD15_SINGLE_CHANNEL:
            cgain = HMCAD15_CGAIN_S1(adc->channelCfg[0].coarse);
            hmcad15xxRegWrite(adc, HMCAD15_REG_COARSE_GAIN_2, cgain);
            break;
        case HMCAD15_DUAL_CHANNEL:
            cgain = HMCAD15_CGAIN_D1(adc->channelCfg[0].coarse);
            cgain |= HMCAD15_CGAIN_D2(adc->channelCfg[1].coarse);
            hmcad15xxRegWrite(adc, HMCAD15_REG_COARSE_GAIN_2, cgain);
            break;
        case HMCAD15_QUAD_CHANNEL:
            cgain = HMCAD15_CGAIN_Q1(adc->channelCfg[0].coarse);
            cgain |= HMCAD15_CGAIN_Q2(adc->channelCfg[1].coarse);
            cgain |= HMCAD15_CGAIN_Q3(adc->channelCfg[2].coarse);
            cgain |= HMCAD15_CGAIN_Q4(adc->channelCfg[3].coarse);
            hmcad15xxRegWrite(adc, HMCAD15_REG_COARSE_GAIN_1, cgain);
        break;
    }
}