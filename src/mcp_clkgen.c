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

#include "ts_common.h"
#include "mcp_clkgen.h"
#include "i2c.h"
#include "liblitepcie.h"

#define ZL302XX_ADDR_LEN    (2)

int32_t mcp_clkgen_config(i2c_t device, uint32_t* confData, uint32_t confLen)
{
    if(NULL == confData)
    {
        //Error
        return TS_STATUS_ERROR;
    }

    for(uint32_t i = 0; i < confLen; i++)
    {
        //Two bytes of address
        uint32_t addr = (confData[i] >> 8) & 0xFFFF;
        //One byte of data
        uint8_t reg = confData[i] & 0xFF;

        if(!i2c_write(device, addr, &reg, 1, ZL302XX_ADDR_LEN))
        {
            return TS_STATUS_ERROR;
        }
    }
    return TS_STATUS_OK;
}
