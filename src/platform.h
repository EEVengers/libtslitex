/* SPDX-License-Identifier: BSD-2-Clause
 *
 * Thunderscope library
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include "ts_common.h"
#include "mcp_clkgen.h"
#include "mcp_zl3026x.h"
#include "i2c.h"
#include "csr.h"


#define TS_ADC_FULL_SCALE_ADJUST_DEFAULT        (0x20) /**< Full Scale Adjust set 2V */
#define TS_ADC_CH_COARSE_GAIN_DEFAULT           (9)
#define TS_ADC_CH_FINE_GAIN_DEFAULT             (0)

#define TS_ADC_CH_NO_INVERT     (0)
#define TS_ADC_CH_INVERT        (1)

#define TS_AFE_OUTPUT_NOMINAL_mVPP              (700.0)
#define TS_ATTENUATION_1M_GAIN_mdB              (-33979) /**< 50x Attenuation = 20 * log(1/50) * 1000 */
#define TS_TERMINATION_50OHM_GAIN_mdB           (-13979) /**< 5x Attenuation from 50Ohm mode.  20 * log(1/5) * 1000 */

#define TS_VBUFFER_NOMINAL_MV                   (2500)
#define TS_VBIAS_NOMINAL_MV                     (2500)
#define TS_AFE_TRIM_VDD_NOMINAL                 (5000)
#define TS_BUFFER_GAIN_NOMINAL_mdB              (-250)
#define TS_BIAS_RESISTOR_NOMINAL                (500)
#define TS_PREAMP_INPUT_BIAS_CURRENT_uA         (40)

#define TS_SPI_BUS_BASE_ADDR    CSR_MAIN_SPI_BASE    
#define TS_SPI_BUS_CS_NUM       (CSR_MAIN_SPI_CS_SEL_SIZE)
#define TS_AFE_0_AMP_CS         (0)
#define TS_AFE_1_AMP_CS         (1)
#define TS_AFE_2_AMP_CS         (2)
#define TS_AFE_3_AMP_CS         (3)
#define TS_ADC_CS               (4)

#define TS_I2C_BASE_ADDR        CSR_I2C_BASE
#define TS_I2C_CLK_RATE         I2C_400KHz

#define TS_TRIM_DAC_I2C_ADDR    (0x60)
#define TS_TRIM_DAC_DEFAULT     (0x800)

#define TS_AFE_0_TRIM_DAC       (0)
#define TS_AFE_1_TRIM_DAC       (1)
#define TS_AFE_2_TRIM_DAC       (2)
#define TS_AFE_3_TRIM_DAC       (3)

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

#endif