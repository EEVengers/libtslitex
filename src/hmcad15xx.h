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
#ifndef _HMCAD15XX_H_
#define _HMCAD15XX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "spi.h"


/************
 * REGISTERS
 ************/
#define HMCAD15_REG_SW_RST              0x00
#define HMCAD15_REG_POWER_CTRL          0x0F
#define HMCAD15_REG_LVDS_CURRENT        0x11
#define HMCAD15_REG_TERM                0x12
#define HMCAD15_REG_CHAN_INVERT         0x24
#define HMCAD15_REG_TEST_MODE           0x25
#define HMCAD15_REG_TEST_PAT1           0x26
#define HMCAD15_REG_TEST_PAT2           0x27
#define HMCAD15_REG_COARSE_GAIN_1       0x2A
#define HMCAD15_REG_COARSE_GAIN_2       0x2B
#define HMCAD15_REG_CLK_JITTER          0x30
#define HMCAD15_REG_CHAN_MODE           0x31
#define HMCAD15_REG_GAIN_SEL            0x33
#define HMCAD15_REG_FINE_GAIN_1_2       0x34
#define HMCAD15_REG_FINE_GAIN_3_4       0x35
#define HMCAD15_REG_FINE_GAIN_5_6       0x36
#define HMCAD15_REG_FINE_GAIN_7_8       0x37
#define HMCAD15_REG_IN_SEL_1_2          0x3A
#define HMCAD15_REG_IN_SEL_3_4          0x3B
#define HMCAD15_REG_LCLK_PHASE          0x42
#define HMCAD15_REG_PAT_MODE            0x45
#define HMCAD15_REG_DATA_FMT            0x46
#define HMCAD15_REG_DRIVE_STRENGTH      0x50
#define HMCAD15_REG_LVDS_POWERDOWN      0x52
#define HMCAD15_REG_LVDS_MISC           0x53
#define HMCAD15_REG_ADC_FULL_SCALE      0x55
#define HMCAD15_REG_STARTUP_TIME        0x56

#define HMCAD15_NUM_ADC             (4)
#define HMCAD15_NUM_CHANNELS        (4)

#define HMCAD15_FULL_SCALE_SET(x)   (((((x+2) - HMCAD15_FULL_SCALE_MIN) * 0x3F) / \
                                        (HMCAD15_FULL_SCALE_MAX - HMCAD15_FULL_SCALE_MIN)) & 0x3F)
#define HMCAD15_FULL_SCALE_DEFAULT  (0)
#define HMCAD15_FULL_SCALE_MAX      (97)
#define HMCAD15_FULL_SCALE_MIN      (-100)

#define HMCAD15_SAMPLE_MODE_SET(x)  ((x) & 0x0F)

#define HMCAD15_CLK_DIV_SET(x)      (((x) & 0x03) << 8)
#define HMCAD15_CLK_DIV_1           (0)
#define HMCAD15_CLK_DIV_2           (1)
#define HMCAD15_CLK_DIV_4           (2)
#define HMCAD15_CLK_DIV_8           (3)

#define HMCAD15_ADC_IN1             (1)
#define HMCAD15_ADC_IN2             (2)
#define HMCAD15_ADC_IN3             (4)
#define HMCAD15_ADC_IN4             (8)

#define HMCAD15_SEL_CH_1(x)         (((x) & 0x0F) << 1)
#define HMCAD15_SEL_CH_2(x)         (((x) & 0x0F) << 9)
#define HMCAD15_SEL_CH_3(x)         (((x) & 0x0F) << 1)
#define HMCAD15_SEL_CH_4(x)         (((x) & 0x0F) << 9)

#define HMCAD15_CGAIN_Q1(x)         ((x) & 0xF)
#define HMCAD15_CGAIN_Q2(x)         (((x) & 0xF) << 4)
#define HMCAD15_CGAIN_Q3(x)         (((x) & 0xF) << 8)
#define HMCAD15_CGAIN_Q4(x)         (((x) & 0xF) << 12)

#define HMCAD15_CGAIN_D1(x)         ((x) & 0xF)
#define HMCAD15_CGAIN_D2(x)         (((x) & 0xF) << 4)

#define HMCAD15_CGAIN_S1(x)         (((x) & 0xF) << 8)

#define HMCAD15_CH_INVERT_Q1(x)     (((x) & 0x01) << 0)
#define HMCAD15_CH_INVERT_Q2(x)     (((x) & 0x01) << 1)
#define HMCAD15_CH_INVERT_Q3(x)     (((x) & 0x01) << 2)
#define HMCAD15_CH_INVERT_Q4(x)     (((x) & 0x01) << 3)

#define HMCAD15_CH_INVERT_D1(x)     (((x) & 0x01) << 4)
#define HMCAD15_CH_INVERT_D2(x)     (((x) & 0x01) << 5)

#define HMCAD15_CH_INVERT_S1(x)     (((x) & 0x01) << 6)
#define HMCAD15_CH_INVERT_ALL(x)    ((x) * 0x7f)


#define HMCAD15_LVDS_DS_LCLK(x)     ((x) & 0x07)
#define HMCAD15_LVDS_DS_FRAME(x)    (((x) & 0x07) << 4)
#define HMCAD15_LVDS_DS_DATA(x)     (((x) & 0x07) << 8)

#define HMCAD15_DATA_WIDTH(x)       ((x) & 0x07)

#define HMCAD15_LVDS_PHASE(x)       (((x) & 0x03) << 5)

#define HMCAD15_DATA_FMT_BTC(x)     (((x) & 0x1) << 2)

#define HMCAD15_SINGLE_CH_SLP       (1 << 6)
#define HMCAD15_DUAL_CH_1_SLP       (1 << 5)
#define HMCAD15_DUAL_CH_0_SLP       (1 << 4)
#define HMCAD15_QUAD_CH_3_SLP       (1 << 3)
#define HMCAD15_QUAD_CH_2_SLP       (1 << 2)
#define HMCAD15_QUAD_CH_1_SLP       (1 << 1)
#define HMCAD15_QUAD_CH_0_SLP       (1 << 0)

#define HMCAD15_PWR_MODE_SLEEP      (1 << 8)
#define HMCAD15_PWR_MODE_POWERDN    (1 << 9)

#define HMCAD15_TEST_MODE_SINGLE    (1 << 4)
#define HMCAD15_TEST_MODE_DUAL      (1 << 5)
#define HMCAD15_TEST_MODE_RAMP      (1 << 6)

#define HMCAD15_TEST_PAT_DESKEW     (1 << 0)
#define HMCAD15_TEST_PAT_SYNC       (1 << 1)

#define HMCAD15_LVDS_PHASE_270DEG   (0)
#define HMCAD15_LVDS_PHASE_180DEG   (1)
#define HMCAD15_LVDS_PHASE_90DEG    (2)
#define HMCAD15_LVDS_PHASE_0DEG     (3)

#define HMCAD15_LVDS_DS_35          (0)
#define HMCAD15_LVDS_DS_25          (1)
#define HMCAD15_LVDS_DS_15          (2)
#define HMCAD15_LVDS_DS_05          (3)
#define HMCAD15_LVDS_DS_75          (4)
#define HMCAD15_LVDS_DS_65          (5)
#define HMCAD15_LVDS_DS_55          (6)
#define HMCAD15_LVDS_DS_45          (7)


#define HMCAD15_CLK_DIV_DEFAULT     (1)
#define HMCAD15_LVDS_PHASE_DEFAULT  (HMCAD15_LVDS_PHASE_0DEG)

typedef enum hmcad15xxMode_e
{
    HMCAD15_SINGLE_CHANNEL = 1,
    HMCAD15_DUAL_CHANNEL = 2,
    HMCAD15_QUAD_CHANNEL = 4,
    HMCAD15_14BIT_QUAD_CHANNEL = 8
} hmcad15xxMode_t;

typedef enum hmcad15xxDataWidth_e
{
    HMCAD15_8_BIT,
    HMCAD15_12_BIT,
    HMCAD15_14_BIT
} hmcad15xxDataWidth_t;

typedef enum hmcad15xxPower_e
{
    HMCAD15_CH_ACTIVE,
    HMCAD15_CH_SLEEP,
    HMCAD15_CH_POWERDN
}hmcad15xxPower_t;

typedef enum hmcad15xxTestMode_e
{
    HMCAD15_TEST_DISABLE,
    HMCAD15_TEST_SINGLE,
    HMCAD15_TEST_DUAL,
    HMCAD15_TEST_RAMP,
    HMCAD15_TEST_DESKEW,
    HMCAD15_TEST_SYNC
} hmcad15xxTestMode_t;

typedef enum hmcad15xxBtcFmt_e
{
    HMCAD15_BTC_FMT_OFFSET,
    HMCAD15_BTC_FMT_TWOS_COMPL
} hmcad15xxBtcFmt_t;

typedef struct hmcad15xxChCfg_s
{
    uint8_t active;
    uint8_t input;
    uint8_t coarse;
    uint8_t fine;
    uint8_t invert;
} hmcad15xxChCfg_t;

typedef struct hmcad15xxADC_s
{
    spi_dev_t dev;
    hmcad15xxChCfg_t channelCfg[HMCAD15_NUM_CHANNELS];
    hmcad15xxMode_t mode;
    hmcad15xxBtcFmt_t format;
    hmcad15xxDataWidth_t width;
    int32_t fullScale_x10;
    uint8_t clockDiv;
    uint8_t lvdsPhase;
    uint8_t drive;
} hmcad15xxADC_t;

/**
 * @brief Initialize the HMCAD15xx Device.  This will perform a soft reset and
 *        place the chip into a sleep state
 * 
 * @param adc Pointer to an ADC instance
 * @param dev SPI Device instance
 * @return int32_t TS_STATUS_OK if successful
 */
int32_t hmcad15xx_init(hmcad15xxADC_t* adc, spi_dev_t dev);

/**
 * @brief Perform a soft reset on the HMCAD15xx device
 * 
 * @param adc Pointer to an ADC instance
 * @return int32_t TS_STATUS_OK if successful
 */
int32_t hmcad15xx_reset(hmcad15xxADC_t* adc);

/**
 * @brief Set the Power Mode of the HMCAD15xx.  If Active, individual inactive
 * channels will be placed in the sleep mode.
 * 
 * @param adc Pointer to an ADC instance
 * @param power Power Mode to put the ADC in
 * @return int32_t TS_STATUS_OK if successful
 */
int32_t hmcad15xx_power_mode(hmcad15xxADC_t* adc, hmcad15xxPower_t power);

/**
 * @brief Update the Channel Configuration for the HMCAD15xx
 * 
 * @param adc Pointer to an ADC instance
 * @return int32_t TS_STATUS_OK if successful
 */
int32_t hmcad15xx_set_channel_config(hmcad15xxADC_t* adc);

/**
 * @brief Set the full scale adjustment
 * 
 * @param adc Pointer to an ADC instance
 * @param adjustment Full Scale Adjustment factor, in tenths of a percent
 * @return int32_t TS_STATUS_OK if successful, TS_INVALID_PARAM if the value is
 * out of the allowable range
 */
int32_t hmcad15xx_full_scale_adjust(hmcad15xxADC_t* adc, int8_t adjustment);

/**
 * @brief Put the HMCAD15xx into a test mode that sends test data out the LVDS
 * interface
 * 
 * @param adc Pointer to an ADC Instance
 * @param mode Test Pattern mode
 * @param testData1 Test Data Pattern 1
 * @param testData2 Test Data Pattern 2
 * @return int32_t TS_STATUS_OK if successful
 */
int32_t hmcad15xx_set_test_pattern(hmcad15xxADC_t* adc, hmcad15xxTestMode_t mode, uint16_t testData1, uint16_t testData2);

#ifdef __cplusplus
}
#endif

#endif
