/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Trim DAC Register Configuration for the MCP4728
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#include <stdint.h>
#include "i2c.h"

typedef enum Mcp4728Vref_e
{
    MCP4728_VREF_VDD = 0,
    MCP4728_VREF_INTERNAL
} Mcp4728Vref_t;

typedef enum Mcp4728PD_e
{
    MCP4728_PD_NORMAL = 0,
    MCP4728_PD_1K,
    MCP4728_PD_100K,
    MCP4728_PD_500K
} Mcp4728PD_t;

typedef enum Mcp4728Gain_e
{
    MCP4728_GAIN_1X = 0,
    MCP4728_GAIN_2X
} Mcp4728Gain_t;

typedef struct Mcp4728ChannelConfig_s
{
    Mcp4728Vref_t vref;
    Mcp4728PD_t power;
    Mcp4728Gain_t gain;
    uint16_t value;
} Mcp4728ChannelConfig_t;

/**
 * @brief Set an output channel on an MCP4728 DAC
 * 
 * @param dev I2C device for the DAC
 * @param channel Output channel of the MCP4728 to set
 * @param conf Channel parameters to apply to the output channel
 * 
 * @return int32_t TS_STATUS_OK on success, else error
 */
int32_t mcp4728_channel_set(i2c_t dev, uint8_t channel, Mcp4728ChannelConfig_t conf);