/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Definitions of data and configuration for the Thunderscope devices
 *
 * Copyright (c) 2024 John Simons <jammsimons@gmail.com>
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 */

#include <stdint.h>

#include "platform.h"
#include "mcp_clkgen.h"

const mcp_clkgen_conf_t ZL30260_CONF[] = {
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0423, .value=0x08},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0003, .value=0x01},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0004, .value=0x02},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0005, .value=0x21},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0007, .value=0x01},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0100, .value=0x42},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0101, .value=0x00},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0102, .value=0x01},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0106, .value=0x00},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0107, .value=0x00},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0108, .value=0x00},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0109, .value=0x00},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x010A, .value=0x20},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x010B, .value=0x03},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0121, .value=0x60},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0127, .value=0x90},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0141, .value=0x00},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0142, .value=0x00},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0143, .value=0x00},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0144, .value=0x00},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0145, .value=0xA0},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0153, .value=0x00},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0154, .value=0x50},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0155, .value=0xCE},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0180, .value=0x00},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0200, .value=0x80},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0201, .value=0x05},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0250, .value=0x80},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0251, .value=0x02},
    // {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0430, .value=0x0C},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0430, .value=0x00},
    {.action=MCP_CLKGEN_DELAY, .delay_us=10000},
    // {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0100, .value=0x02},
    // {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0100, .value=0x42},
    // {.action=MCP_CLKGEN_DELAY, .delay_us=10000}
};

const uint32_t ZL30260_CONF_SIZE = sizeof(ZL30260_CONF)/sizeof(ZL30260_CONF[0]);

const mcp_clkgen_conf_t ZL30250_CONF[] = {
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0009, .value=0x02},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0621, .value=0x08},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0631, .value=0x40},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0100, .value=0x06},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0101, .value=0x20},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0102, .value=0x02},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0103, .value=0x80},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x010A, .value=0x20},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x010B, .value=0x03},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0114, .value=0x0D},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0120, .value=0x06},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0125, .value=0xC0},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0126, .value=0x60},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0127, .value=0x7F},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0129, .value=0x04},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x012A, .value=0xB3},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x012B, .value=0xC0},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x012C, .value=0x80},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x001C, .value=0x10},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x001D, .value=0x80},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0340, .value=0x03},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0201, .value=0x41},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0221, .value=0x35},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x0222, .value=0x40},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x000C, .value=0x02},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x000B, .value=0x01},
    {.action=MCP_CLKGEN_DELAY, .delay_us=10000},
    {.action=MCP_CLKGEN_WRITE_REG, .addr=0x000D, .value=0x05}, //Need 10ms beforehand, will be fine through jtag
    {.action=MCP_CLKGEN_DELAY, .delay_us=10000}
};

const uint32_t ZL30250_CONF_SIZE = sizeof(ZL30250_CONF)/sizeof(ZL30250_CONF[0]);
