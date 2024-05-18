/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Data Control and Management functions for the
 * Thunderscope LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#include <stdlib.h>

#include "thunderscope.h"
#include "ts_common.h"

#include "i2c.h"
#include "spi.h"
#include "litepcie.h"

typedef struct ts_inst_s
{
    file_t ctrl;
    struct litepcie_dma_ctrl dma;
    tsChannelConfig_t conf;
    //TBD - Other Instance Data
} ts_inst_t;


int32_t thunderscopeListDevices(uint32_t devIndex, char* nameBuffer, uint32_t bufLen)
{
    int32_t retVal = TS_STATUS_ERROR;
    
    // Find device path by index

    //If index valid
    if(devIndex)
    {
        retVal = TS_STATUS_OK;
    }

    return retVal;
}

tsHandle_t thunderscopeOpen(char* name)
{
    tsHandle_t ts = NULL;
    
    if(!name)
    {
        return ts;
    } 

    return ts;
}


int32_t thunderscopeClose(tsHandle_t ts)
{
    if(ts == NULL)
    {
        return TS_STATUS_ERROR;
    }

    ts_inst_t* pInst = (ts_inst_t*)ts;
    
    litepcie_dma_cleanup(&pInst->dma);
    litepcie_close(pInst->ctrl);

    return TS_STATUS_OK;
}

int32_t thunderscopeChannelConfigGet(tsHandle_t ts, uint32_t channel, tsChannelParam_t* conf)
{
    //TODO
    return TS_STATUS_ERROR;
}

int32_t thunderscopeChannelConfigSet(tsHandle_t ts, uint32_t channel, tsChannelParam_t* conf)
{
    //TODO
    return TS_STATUS_ERROR;
}

int32_t thunderscopeDataEnable(tsHandle_t ts, uint8_t enable)
{
    //TODO
    return TS_STATUS_ERROR;
}

int32_t thunderscopeRead(tsHandle_t ts, uint8_t* buffer, uint32_t len)
{
    //TODO
    return TS_STATUS_ERROR;
}