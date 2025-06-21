/* SPDX-License-Identifier: BSD-2-Clause
 *
 * Thunderscope library
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ts_common.h"
#include "mcp_clkgen.h"
#include "mcp_zl3026x.h"
#include "i2c.h"
#include "csr.h"

#define TS_STATUS_LED_ADDR                      (CSR_DEV_STATUS_LEDS_ADDR)
#define TS_STATUS_LED_COUNT                     (3)
#define TS_STATUS_LED_MASK                      ((1 << TS_STATUS_LED_COUNT) - 1)

#define TS_ADC_FULL_SCALE_ADJUST_DEFAULT        (0x20) /**< Full Scale Adjust set 2V */
#define TS_ADC_CH_COARSE_GAIN_DEFAULT           (9)
#define TS_ADC_CH_FINE_GAIN_DEFAULT             (0)

#define TS_ADC_CH_NO_INVERT     (0)
#define TS_ADC_CH_INVERT        (1)

#define TS_AFE_OUTPUT_NOMINAL_uVPP              (700000.0)
#define TS_ATTENUATION_1M_GAIN_mdB              (-33979) /**< 50x Attenuation = 20 * log(1/50) * 1000 */
#define TS_TERMINATION_50OHM_GAIN_mdB           (-13979) /**< 5x Attenuation from 50Ohm mode.  20 * log(1/5) * 1000 */

#define TS_VBUFFER_NOMINAL_UV                   (2500000)
#define TS_VBIAS_NOMINAL_UV                     (2500000)
#define TS_AFE_TRIM_VDD_NOMINAL                 (5000000)
#define TS_BUFFER_GAIN_NOMINAL_mdB              (-250)
#define TS_BIAS_RESISTOR_NOMINAL                (500)
#define TS_PREAMP_INPUT_BIAS_CURRENT_uA         (40)

#define TS_AFE_PREAMP_ATTEN_0_mdB               (0)
#define TS_AFE_PREAMP_ATTEN_1_mdB               (-2000)
#define TS_AFE_PREAMP_ATTEN_2_mdB               (-4000)
#define TS_AFE_PREAMP_ATTEN_3_mdB               (-6000)
#define TS_AFE_PREAMP_ATTEN_4_mdB               (-8000)
#define TS_AFE_PREAMP_ATTEN_5_mdB               (-10000)
#define TS_AFE_PREAMP_ATTEN_6_mdB               (-12000)
#define TS_AFE_PREAMP_ATTEN_7_mdB               (-14000)
#define TS_AFE_PREAMP_ATTEN_8_mdB               (-16000)
#define TS_AFE_PREAMP_ATTEN_9_mdB               (-18000)
#define TS_AFE_PREAMP_ATTEN_10_mdB              (-20000)

#define TS_SPI_BUS_BASE_ADDR    (CSR_SPIBUS_SPI0_CONTROL_ADDR)
#define TS_SPI_BUS_BETA_CS_NUM  (5)
#define TS_SPI_BUS_DEV_CS_NUM   (4)
#define TS_AFE_0_AMP_CS         (0)
#define TS_AFE_1_AMP_CS         (1)
#define TS_AFE_2_AMP_CS         (2)
#define TS_AFE_3_AMP_CS         (3)
#define TS_BETA_ADC_CS          (4)

#define TS_ADC_SPI_BUS_BASE_ADDR    (CSR_SPIBUS_SPI1_CONTROL_ADDR)
#define TS_ADC_SPI_BUS_CS_NUM       (1)
#define TS_ADC_CS                   (0)

#define TS_I2C_CLK_RATE         I2C_400KHz

#define TS_TRIM_DAC_BUS         (CSR_I2CBUS_I2C0_PHY_SPEED_MODE_ADDR)
#define TS_TRIM_DAC_I2C_ADDR    (0x60)
#define TS_TRIM_DAC_DEFAULT     (0x800)

#define TS_AFE_0_TRIM_DAC       (0)
#define TS_AFE_1_TRIM_DAC       (1)
#define TS_AFE_2_TRIM_DAC       (2)
#define TS_AFE_3_TRIM_DAC       (3)

#define TS_TRIM_DPOT_BUS        (CSR_I2CBUS_I2C0_PHY_SPEED_MODE_ADDR)
#define TS_TRIM_DPOT_I2C_ADDR   (0x2C)
#define TS_TRIM_DPOT_DEFAULT    (0x40)

#define TS_AFE_0_TRIM_DPOT      (2)
#define TS_AFE_1_TRIM_DPOT      (0)
#define TS_AFE_2_TRIM_DPOT      (1)
#define TS_AFE_3_TRIM_DPOT      (3)

#define ZL30250_I2C_ADDR        (0x6C)
#define ZL30260_I2C_ADDR        (0x74)

extern const mcp_clkgen_conf_t ZL30250_CONF[];
extern const uint32_t ZL30250_CONF_SIZE;

extern const mcp_clkgen_conf_t ZL30260_CONF[];
extern const uint32_t ZL30260_CONF_SIZE;

#ifdef TS_REV_3
#define TS_PLL_I2C_ADDR         ZL30250_I2C_ADDR
#define TS_PLL_CONF             ZL30250_CONF
#define TS_PLL_CONF_SIZE        ZL30250_CONF_SIZE
#else
#define TS_PLL_BUS_BETA         CSR_I2CBUS_I2C0_PHY_SPEED_MODE_ADDR
#define TS_PLL_BUS_DEV          CSR_I2CBUS_I2C1_PHY_SPEED_MODE_ADDR
#define TS_PLL_I2C_ADDR         ZL30260_I2C_ADDR
#define TS_PLL_CONF             ZL30260_CONF
#define TS_PLL_CONF_SIZE        ZL30260_CONF_SIZE
#define TS_PLL_INPUT_IDX        (1)
#define TS_PLL_INPUT_RATE       (10000000)
#define TS_PLL_INPUT_SEL        (ZL3026X_INPUT_IC2)

#define TS_PLL_REFIN_IDX        (0)
#define TS_PLL_REFIN_SEL        (ZL3026X_INPUT_IC1)

#define TS_PLL_REFOUT_CLK_IDX       (0)
#define TS_PLL_REFOUT_RATE_DEFAULT  (10000000)
#define TS_PLL_REFOUT_CLK_MODE      (ZL3026X_OUT_CMOS_P)
#define TS_PLL_REFOUT_PLL_MODE      (ZL3026X_PLL_BYPASS)

#define TS_PLL_SAMPLE_CLK_IDX       (5)
#define TS_PLL_SAMPLE_RATE_DEFAULT  (1000000000)
#define TS_PLL_SAMPLE_CLK_MODE      (ZL3026X_OUT_DIFF)
#define TS_PLL_SAMPLE_PLL_MODE      (ZL3026X_PLL_INT_DIV)
#endif

#define TS_PLL_NRST_ADDR        CSR_ADC_CONTROL_ADDR
#define TS_PLL_NRST_MASK        (1 << (CSR_ADC_CONTROL_OSC_EN_OFFSET))

#define TS_AFE_POWER_REG        CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_POWER_MASK       (1 << (CSR_FRONTEND_CONTROL_FE_EN_OFFSET))

#define TS_ACQ_POWER_REG        CSR_ADC_CONTROL_ADDR
#define TS_ACQ_POWER_MASK       (1 << (CSR_ADC_CONTROL_ACQ_EN_OFFSET))

#define TS_AFE_0_TERM_REG       CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_0_TERM_MASK      (1 << CSR_FRONTEND_CONTROL_TERMINATION_OFFSET)

#define TS_AFE_1_TERM_REG       CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_1_TERM_MASK      (1 << (CSR_FRONTEND_CONTROL_TERMINATION_OFFSET + 1))

#define TS_AFE_2_TERM_REG       CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_2_TERM_MASK      (1 << (CSR_FRONTEND_CONTROL_TERMINATION_OFFSET + 2))

#define TS_AFE_3_TERM_REG       CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_3_TERM_MASK      (1 << (CSR_FRONTEND_CONTROL_TERMINATION_OFFSET + 3))

#define TS_AFE_0_ATTEN_REG      CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_0_ATTEN_MASK     (1 << (CSR_FRONTEND_CONTROL_ATTENUATION_OFFSET))

#define TS_AFE_1_ATTEN_REG      CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_1_ATTEN_MASK     (1 << (CSR_FRONTEND_CONTROL_ATTENUATION_OFFSET + 1))

#define TS_AFE_2_ATTEN_REG      CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_2_ATTEN_MASK     (1 << (CSR_FRONTEND_CONTROL_ATTENUATION_OFFSET + 2))

#define TS_AFE_3_ATTEN_REG      CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_3_ATTEN_MASK     (1 << (CSR_FRONTEND_CONTROL_ATTENUATION_OFFSET + 3))

#define TS_AFE_0_COUPLING_REG   CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_0_COUPLING_MASK  (1 << (CSR_FRONTEND_CONTROL_COUPLING_OFFSET))

#define TS_AFE_1_COUPLING_REG   CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_1_COUPLING_MASK  (1 << (CSR_FRONTEND_CONTROL_COUPLING_OFFSET + 1))

#define TS_AFE_2_COUPLING_REG   CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_2_COUPLING_MASK  (1 << (CSR_FRONTEND_CONTROL_COUPLING_OFFSET + 2))

#define TS_AFE_3_COUPLING_REG   CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_3_COUPLING_MASK  (1 << (CSR_FRONTEND_CONTROL_COUPLING_OFFSET + 3))


typedef struct led_signals_s {
    uint32_t error;
    uint32_t ready;
    uint32_t active;
    // Others TBD
    uint32_t disabled;
} led_signals_t;

extern const led_signals_t ts_beta_leds;
extern const led_signals_t ts_dev_leds;

typedef struct flash_layout_s {
    uint32_t factory_bitstream_start;
    uint32_t factory_bitstream_end;
    uint32_t factory_config_start;
    uint32_t factory_config_end;
    uint32_t user_bitstream_start;
    uint32_t user_bitstream_end;
    uint32_t user_config_start;
    uint32_t user_config_end;
} flash_layout_t;

extern const flash_layout_t ts_256Mb_layout;
extern const flash_layout_t ts_64Mb_layout;

#define TS_FLASH_256M_MFG   (0x01) //Spansion
#define TS_FLASH_256M_ID    (0x0219) //256Mb SPI Flash
#define TS_FLASH_64M_MFG    (0xC2) //Macronix
#define TS_FLASH_64M_ID     (0x2537) //64Mb SPI Flash

static inline bool isBetaDevice(file_t ts) { return (0 == (litepcie_readl(ts, CSR_DEV_STATUS_HW_ID_ADDR) & (1 << CSR_DEV_STATUS_HW_ID_HW_VALID_OFFSET)));}

#ifdef __cplusplus
}
#endif
#endif