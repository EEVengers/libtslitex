/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Driver for the zl30260 Clock Generator as used
 * in the Thunderscope.  Datasheet reference at 
 * https://ww1.microchip.com/downloads/aemDocuments/documents/TCG/ProductDocuments/DataSheets/ZL30260-1-2-3-1-APLL-6-or-10-Output-Any-to-Any-Clock-Multiplier-and-Frequency-Synthesizer-DS20006554.pdf
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _MCP_ZL3026X_H_
#define _MCP_ZL3026X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mcp_clkgen.h"

//TODO: Register Map

#define ZL3026X_NUM_INPUT_CLK   (3)
#define ZL3026X_NUM_OUTPUT_CLK  (10)

#define ZL3026X_MIN_PLL_OUT     (267000000)
#define ZL3026X_MAX_PLL_OUT     (1050000000)

#define ZL3026X_OUT_MDIV_MIN_CLK    (375000000)

#define ZL3026X_INPUT_CLK_MIN   (9720000)
#define ZL3026X_INPUT_CLK_MAX   (156250000)


typedef enum zl3026x_input_e {
    ZL3026X_INPUT_IC1,
    ZL3026X_INPUT_IC2,
    ZL3026X_INPUT_IC3,
    ZL3026X_INPUT_XO,
    ZL3026X_INPUT_XO_DBL
} zl3026x_input_t;

typedef enum zl3026x_input_div_e {
    ZL3026X_IN_DIV_1 = 0,
    ZL3026X_IN_DIV_2 = 1,
    ZL3026X_IN_DIV_4 = 2,
    ZL3026X_IN_DIV_8 = 3
} zl3026x_input_div_t;

typedef enum zl3026x_clk_src_e {
    ZL3026X_PLL_INT_DIV,
    ZL3026X_PLL_FRAC_DIV,
    ZL3026X_PLL_BYPASS,
    ZL3026X_PLL_BYPASS_2,
} zl3026x_clk_src_t;

typedef enum zl3026x_out_mode_e {
    ZL3026X_OUT_DISABLED = 0,
    ZL3026X_OUT_LVDS = 1,
    ZL3026X_OUT_DIFF = 2,
    ZL3026X_OUT_HSTL = 3,
    ZL3026X_OUT_CMOS_2 = 4,
    ZL3026X_OUT_CMOS_P = 5,
    ZL3026X_OUT_CMOS_N = 6,
    ZL3026X_OUT_CMOS_OPP = 7,
    ZL3026X_OUT_HCSL = 10,
} zl3026x_out_mode_t;

typedef struct zl3026x_input_xo_s {
    uint8_t enable;
    uint64_t xo_freq;
    //TODO Add Crystal Oscillator Config
} zl3026x_input_xo_t;

typedef struct zl3026x_input_clk_s {
    uint8_t enable;
    uint64_t input_freq;
    zl3026x_input_div_t input_divider;
} zl3026x_input_clk_t;

typedef struct zl3026x_output_clk_s {
    uint8_t enable;
    uint64_t output_freq;
    zl3026x_out_mode_t output_mode;
    zl3026x_clk_src_t output_pll_select;
} zl3026x_output_clk_t;

typedef struct zl3026x_clk_config_s {
    zl3026x_input_xo_t in_xo;
    zl3026x_input_clk_t in_clks[ZL3026X_NUM_INPUT_CLK];
    zl3026x_output_clk_t out_clks[ZL3026X_NUM_OUTPUT_CLK];
    zl3026x_input_t input_select;
    //TODO Add GPIO Config
} zl3026x_clk_config_t;

int32_t mcp_zl3026x_build_config(mcp_clkgen_conf_t* confData, uint32_t len, zl3026x_clk_config_t conf);

#ifdef __cplusplus
}
#endif
#endif