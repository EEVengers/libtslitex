/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Control the ZL30260/ZL30250 in the Thunderscope LiteX design
 *
 * Copyright (c) 2024 John Simons <jammsimons@gmail.com>
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stddef.h>
#include <time.h>

#include "ts_common.h"
#include "mcp_clkgen.h"
#include "i2c.h"
#include "liblitepcie.h"
#include "util.h"

#define ZL302XX_ADDR_LEN        (3)
#define ZL302XX_WRITE_REG(reg)  (0x020000 | ((reg) & 0xFFFF))
#define ZL302XX_READ_REG(reg)   (0x030000 | ((reg) & 0xFFFF))

int32_t mcp_clkgen_config(i2c_t device, const mcp_clkgen_conf_t* confData, uint32_t confLen)
{
    if(NULL == confData)
    {
        //Error
        return TS_STATUS_ERROR;
    }
    LOG_DEBUG("Write Regs:");

    for(uint32_t i = 0; i < confLen; i++)
    {
        switch(confData[i].action)
        {
        case MCP_CLKGEN_DELAY:
        {
            LOG_DEBUG("\tDelay %d us", confData[i].delay_us);
            NS_DELAY(confData[i].delay_us * 1000);
            break;
        }
        case MCP_CLKGEN_WRITE_REG:
        {
            LOG_DEBUG("\t%06X : %02X ", (uint32_t)ZL302XX_WRITE_REG(confData[i].addr), confData[i].value);
            if(!i2c_write(device, (uint32_t)ZL302XX_WRITE_REG(confData[i].addr),
                            &confData[i].value, 1, ZL302XX_ADDR_LEN))
            {
                LOG_DEBUG("(NACK!)");
                return TS_STATUS_ERROR;
            }
            LOG_DEBUG("(ack)");
            break;
        }
        default:
            //Error
            break;    
        }
    }
    return TS_STATUS_OK;
}

void mcp_clkgen_regdump(i2c_t device, const mcp_clkgen_conf_t* confData, uint32_t confLen)
{
    
    if(NULL == confData)
    {
        //Error
        return;
    }

    printf("Confirming Regs:\r\n");
    for(uint32_t i = 0; i < confLen; i++)
    {
        switch(confData[i].action)
        {
        case MCP_CLKGEN_DELAY:
        {
            //skip
            break;
        }
        case MCP_CLKGEN_WRITE_REG:
        {
            uint8_t data[1] = {0};
            
            if(!i2c_read(device, (uint32_t)ZL302XX_READ_REG(confData[i].addr),
                            data, 1, true, ZL302XX_ADDR_LEN))
            {
                LOG_ERROR("MCP CLKGEN REG DUMP Failed to read reg %d", confData[i].addr);
                return;
            }
            
            if(data[1] == confData[i].value)
            {
                printf("\t%06X : %02X\r\n", (uint32_t)ZL302XX_READ_REG(confData[i].addr), data[1]);
            }
            else
            {
                printf("\t%06X : %02X (expected %02X)\r\n", (uint32_t)ZL302XX_READ_REG(confData[i].addr), data[1], confData[i].value);
            }

            break;
        }
        default:
            //Error
            break;    
        }
    }
}