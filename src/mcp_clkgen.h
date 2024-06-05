/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Control the ZL30260/ZL30250 in the Thunderscope LiteX design
 *
 * Copyright (c) 2024 John Simons <jammsimons@gmail.com>
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 */
#ifndef _MCP_CLKGEN_H_
#define _MCP_CLKGEN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

typedef enum mcp_clkgen_action_e {
    MCP_CLKGEN_DELAY,
    MCP_CLKGEN_WRITE_REG
}mcp_clkgen_action_t;

typedef struct mcp_clkgen_conf_s {
    mcp_clkgen_action_t action;
    union {
        uint32_t delay_us;
        struct {
            uint16_t addr;
            uint8_t value;
        };
    };
}mcp_clkgen_conf_t;

/**
 * @brief Configure the PLL with a list of registers.
 * 
 * @param device I2C device for the zl30260/zl30250
 * @param confData Array of configuration register values
 * @param confLen Length of the configuration array
 * @return int32_t TS_STATUS_OK if the configuration was applied, else TS_STATUS_ERROR
 */
int32_t mcp_clkgen_config(i2c_t device, const mcp_clkgen_conf_t* confData, uint32_t confLen);

void mcp_clkgen_regdump(i2c_t device, const mcp_clkgen_conf_t* confData, uint32_t confLen);

#ifdef __cplusplus
}
#endif
#endif