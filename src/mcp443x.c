/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Digital Pot Configuration for the MCP443X
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#include "mcp443x.h"
#include "i2c.h"
#include "ts_common.h"
#include "util.h"

#define MCP4432_CMD_LEN         (1)
#define MCP4432_WRITE_CMD(x)    (((x) << 4) & 0xF0)
#define MCP4432_READ_CMD(x)     ((((x) << 4) & 0xF0) | 0x0C)

const uint8_t channelMap[MCP4432_NUM_CH] = 
{
    0x00,   //Wiper 0
    0x01,   //Wiper 1
    0x06,   //Wiper 2
    0x07    //Wiper 3
};

int32_t mcp443x_set_wiper(i2c_t dev, uint8_t channel, uint8_t val)
{
    if(channel > MCP4432_NUM_CH)
    {
        //Error
        return TS_STATUS_ERROR;
    }
    uint8_t devAddr = channelMap[channel];
    LOG_DEBUG("MCP4432 Set Wiper %d: [%02x %02x]", channel, MCP4432_WRITE_CMD(devAddr), val);
    if(!i2c_write(dev, (uint32_t)MCP4432_WRITE_CMD(devAddr),
                    &val, 1, MCP4432_CMD_LEN))
    {
        LOG_DEBUG("\t%02X : %02X (NACK!)", (uint32_t)MCP4432_WRITE_CMD(devAddr), val);
        return TS_STATUS_ERROR;
    }

    uint8_t readback[2] = {0};
    if(!i2c_read(dev, MCP4432_READ_CMD(devAddr), readback, 2, true, MCP4432_CMD_LEN))
    {
        LOG_ERROR("Failed to read back MCP4432 Data for Wiper %d, CMD %x", channel, MCP4432_READ_CMD(devAddr));
    }
    else
    {
        LOG_DEBUG("MCP4432 Wiper %d Read: %02x %02x", channel, readback[0], readback[1]);
    }

    return TS_STATUS_OK;
}
