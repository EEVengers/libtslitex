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
#include "ts_channel.h"
#include "samples.h"

#include "litepcie.h"


#ifdef _WIN32
#define FILE_FLAGS  (FILE_ATTRIBUTE_NORMAL)
#else
#define FILE_FLAGS  (O_RDWR)
#endif

typedef struct ts_inst_s
{
    file_t ctrl;
    tsDeviceInfo_t identity;
    tsChannelHdl_t pChannel;
    sampleStream_t samples;
    //TBD - Other Instance Data
} ts_inst_t;


int32_t thunderscopeListDevices(uint32_t devIndex, tsDeviceInfo_t *info)
{
    int32_t retVal = TS_STATUS_ERROR;
    char testPath[TS_IDENT_STR_LEN];
    
    // Find device path by index
    snprintf(testPath, LITEPCIE_CTRL_NAME() "%d", devIndex, TS_IDENT_STR_LEN);
    file_t testDev = litepcie_open((const char*)testPath, FILE_FLAGS);

    //If index valid
    if(testDev != INVALID_HANDLE_VALUE)
    {
        info->deviceID = devIndex;
        //Copy device identifier
        for (uint32_t i = 0; i < TS_IDENT_STR_LEN; i++)
        {
            info->identity[i] = (char)litepcie_readl(testDev, CSR_IDENTIFIER_MEM_BASE + 4 * i);
        }
        //TODO Implement Serial Number
        strncpy(info->devicePath, testPath, TS_IDENT_STR_LEN);
        litepcie_close(testDev);
        retVal = TS_STATUS_OK;
    }

    return retVal;
}

tsHandle_t thunderscopeOpen(uint32_t devIdx)
{
    ts_inst_t* pInst = calloc(sizeof(ts_inst_t), 1);

    if(pInst)
    {
        pInst->ctrl = litepcie_open(LITEPCIE_CTRL_NAME(devIdx), FILE_FLAGS);
    }
    else
    {
        return NULL;
    }

    if(pInst->ctrl == INVALID_HANDLE_VALUE)
    {
        free(pInst);
        return NULL;
    }

    if(TS_STATUS_OK != ts_channel_init(&pInst->pChannel, pInst->ctrl))
    {
        free(pInst);
        return NULL;
    }

    if(TS_STATUS_OK != samples_init(&pInst->samples, devIdx, 0))
    {
        ts_channel_destroy(pInst->pChannel);
        free(pInst);
        return NULL;
    }

    return (tsHandle_t)pInst;
}

int32_t thunderscopeClose(tsHandle_t ts)
{
    if(ts == NULL)
    {
        return TS_STATUS_ERROR;
    }

    ts_inst_t* pInst = (ts_inst_t*)ts;
    
    samples_teardown(&pInst->samples);
    ts_channel_destroy(pInst->pChannel);
    litepcie_close(pInst->ctrl);
    free(pInst);

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

int32_t thunderscopeStatusGet(tsHandle_t ts, tsScopeState_t* conf)
{
    if(!conf)
    {
        return TS_STATUS_ERROR;
    }

    //TODO
    conf->adc_sample_rate = 1000000000;
    conf->adc_sample_bits = 8;
    conf->adc_sample_resolution = 256;
    conf->adc_lost_buffer_count = 0;
    conf->flags = 0;

    return TS_STATUS_OK;
}

int32_t thunderscopeSampleModeSet(tsHandle_t ts, uint32_t rate, uint32_t resolution)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;

    if(pInst)
    {
        return ts_channel_sample_rate_set(&pInst->pChannel, rate, resolution);
    }

    return TS_STATUS_ERROR;
}

int32_t thunderscopeCalibrationSet(tsHandle_t ts, uint32_t channel, uint32_t cal)
{
    //TODO
    return TS_STATUS_ERROR;
}

int32_t thunderscopeDataEnable(tsHandle_t ts, uint8_t enable)
{
    int32_t status = TS_STATUS_OK;
    ts_inst_t* pInst = (ts_inst_t*)ts;
    status = ts_channel_run(pInst->pChannel, enable);
    if(status = TS_STATUS_OK)
    {
        return status;
    }

    return samples_enable_set(&pInst->samples, enable);
}

int32_t thunderscopeRead(tsHandle_t ts, uint8_t* buffer, uint32_t len)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    return samples_get_buffers(&pInst->samples, buffer, len);
}