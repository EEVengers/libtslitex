/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Trim DAC Register Configuration for the MCP4728
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#include "mcp4728.h"

#include "ts_common.h"
#include "i2c.h"
#include "util.h"

#define MCP4728_NUM_CH          (4)

#define MCP4728_ADDR_LEN        (1)
#define MCP4728_REG_LEN         (2)
#define MCP4728_CMD(cmd, ch)    (((cmd) << 3) | (((ch) & 0x3) << 1))

#define MCP4728_WR_CMD          (0x0B)

#define MCP4728_DATA_VAL_LOWER(val) ((val) & 0xFF)
#define MCP4728_DATA_VAL_UPPER(val) (((val) >> 8) & 0x0F)
#define MCP4728_DATA_GX(gx)         (((gx) << 4) & 0x01)
#define MCP4728_DATA_PD(pd)         (((pd) << 5) & 0x60)
#define MCP4728_DATA_VREF(vr)       (((vr) << 7) & 0x80)

int32_t mcp4728_channel_set(i2c_t dev, uint8_t channel, Mcp4728ChannelConfig_t conf)
{
    uint32_t cmd;
    uint8_t data[MCP4728_REG_LEN] = {0};

    if(channel >= MCP4728_NUM_CH)
    {
        LOG_ERROR("Invalid Channel ID %d", channel);
        return TS_STATUS_ERROR;
    }

    cmd = MCP4728_CMD(MCP4728_WR_CMD, channel);

    data[0] = MCP4728_DATA_VAL_UPPER(conf.value) |
                MCP4728_DATA_GX(conf.gain) |
                MCP4728_DATA_PD(conf.power) |
                MCP4728_DATA_VREF(conf.vref);
    data[1] = MCP4728_DATA_VAL_LOWER(conf.value);

    LOG_DEBUG("Set Channel %d cmd %02x data[] %02x %02x", channel, cmd, data[0], data[1]);

    if(i2c_write(dev, cmd, data, MCP4728_REG_LEN, MCP4728_ADDR_LEN))
    {
        return TS_STATUS_OK;
    }
    else
    {
        LOG_ERROR("Failed to write AFE DAC channel %d", channel);
        return TS_STATUS_ERROR;
    }
}