/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Access the Sample Data stream from the Thunderscope
 * LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */



#if defined(_WIN32)
#include <Windows.h>
#define ssize_t int64_t
#else
#include <unistd.h>
#include <sys/mman.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include "samples.h"
#include "ts_common.h"

#include "liblitepcie.h"


#if defined(_WIN32)
#define TS_DMA_NAME         "\\DMA%u%u"
#define TS_DMA_NAME_LEN     (16)
#define TS_DMA_OS_FLAGS     (FILE_ATTRIBUTE_NORMAL | \
                             FILE_FLAG_NO_BUFFERING)
#else
#define INVALID_HANDLE_VALUE (-1)
#define TS_DMA_NAME         "/dev/litepcie%u"
#define TS_DMA_NAME_LEN     (24)
#define TS_DMA_OS_FLAGS     (O_RDWR | O_CLOEXEC)
#endif


int32_t samples_init(sampleStream_t* inst, uint8_t devIdx, uint8_t channel)
{
    int32_t retVal = TS_STATUS_ERROR;
    char devName[TS_DMA_NAME_LEN] = {0};
    
    if(inst)
    {
        inst->dma_buffer_count = 0;
        inst->driver_buffer_count = 0;
        inst->active = 0;

        snprintf(devName, TS_DMA_NAME_LEN, TS_DMA_NAME, channel, devIdx);

        inst->dma = litepcie_open(devName, TS_DMA_OS_FLAGS);
        
        if((INVALID_HANDLE_VALUE != inst->dma) &&
            litepcie_request_dma(inst->dma, 0, 1))
        {
            retVal = TS_STATUS_OK;
        }
        //else, DMA Unavailable
    }

    return retVal;
}

int32_t samples_enable_set(sampleStream_t* inst, uint8_t en)
{
    if(NULL == inst)
    {
        return TS_STATUS_ERROR;
    }

    inst->active = en;
    
    //Enable Trigger
    litepcie_writel(inst->dma, CSR_ADC_TRIGGER_CONTROL_ADDR, en);

    //Start/Stop DMA
    litepcie_dma_writer(inst->dma, en,
                        &inst->dma_buffer_count,
                        &inst->driver_buffer_count);

    return TS_STATUS_OK;
}

int32_t samples_get_buffers(sampleStream_t* inst, uint8_t* sampleBuffer, uint32_t bufferLen)
{
    int32_t retVal = TS_STATUS_OK;
    if(inst->active == 0)
    {
        retVal = TS_STATUS_ERROR;
    }
    else if(bufferLen % TS_SAMPLE_BUFFER_SIZE)
    {
        retVal = TS_INVALID_PARAM;
    }
    else
    {
#if defined(_WIN32)
        uint32_t len = 0;
        
        if (!ReadFile(inst->dma, sampleBuffer, bufferLen, &len, NULL))
        {
            fprintf(stderr, "Read failed: %d\n", GetLastError());
            fprintf(stderr, "Read args: 0x%p - 0x%lx - 0x%x\n", sampleBuffer, bufferLen, len);
            samples_teardown(inst);
            abort();
        }
        retVal = (int32_t)len;
#else
        retVal = (int32_t)read(inst->dma, sampleBuffer, bufferLen);
#endif
    }

    return retVal;
}

int32_t samples_teardown(sampleStream_t* inst)
{
    int32_t retVal = TS_STATUS_OK;
    if(NULL == inst)
    {
        retVal = TS_STATUS_ERROR;
    }
    else
    {
        inst->active = 0;
        litepcie_dma_writer(inst->dma, 0, 
                            &inst->dma_buffer_count,
                            &inst->driver_buffer_count);

        litepcie_close(inst->dma);
    }

    return retVal;
}