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
#include "csr.h"


#define TS_ADC_FULL_SCALE_ADJUST_DEFAULT        (0x0010)

#define TS_ADC_CH_NO_INVERT     (0)
#define TS_ADC_CH_INVERT        (1)

#define TS_ATTENUATION_THRESHOLD_MV             (700)
#define TS_ATTENUATION_VALUE_mdB                (33979) /**< 50x Attenuation = 20 * log(50) * 1000 */

#define TS_SPI_BUS_BASE_ADDR    CSR_MAIN_SPI_BASE    
#define TS_SPI_BUS_CS_NUM       (CSR_MAIN_SPI_CS_SEL_SIZE)
#define TS_AFE_0_AMP_CS         (0)
#define TS_AFE_1_AMP_CS         (1)
#define TS_AFE_2_AMP_CS         (2)
#define TS_AFE_3_AMP_CS         (3)
#define TS_ADC_CS               (4)

#define TS_I2C_BASE_ADDR        CSR_I2C_BASE

#define TS_TRIM_DAC_I2C_ADDR    (0x60)
#define TS_TRIM_DAC_DEFAULT     (0x800)

#define TS_AFE_0_TRIM_DAC       (0)
#define TS_AFE_1_TRIM_DAC       (1)
#define TS_AFE_2_TRIM_DAC       (2)
#define TS_AFE_3_TRIM_DAC       (3)

#define TS_TRIM_DPOT_I2C_ADDR   (0x2C)
#define TS_TRIM_DPOT_DEFAULT    (0x00)

#define TS_AFE_0_TRIM_DPOT      (0)
#define TS_AFE_1_TRIM_DPOT      (1)
#define TS_AFE_2_TRIM_DPOT      (2)
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
#endif

#define TS_PLL_NRST_ADDR        CSR_ADC_CONTROL_ADDR
#define TS_PLL_NRST_MASK        (1 << (CSR_ADC_CONTROL_OSC_EN_OFFSET))

#define TS_AFE_POWER_REG        CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_POWER_MASK       (1 << (CSR_FRONTEND_CONTROL_FE_EN_OFFSET))

#define TS_ACQ_POWER_REG        CSR_ADC_CONTROL_ADDR
#define TS_ACQ_POWER_MASK       (1 << (CSR_ADC_CONTROL_ACQ_EN_OFFSET))

#define TS_AFE_0_TERM_REG       CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_0_TERM_MASK      (1 << 20)

#define TS_AFE_1_TERM_REG       CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_1_TERM_MASK      (1 << 21)

#define TS_AFE_2_TERM_REG       CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_2_TERM_MASK      (1 << 22)

#define TS_AFE_3_TERM_REG       CSR_FRONTEND_CONTROL_ADDR
#define TS_AFE_3_TERM_MASK      (1 << 23)

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


#endif