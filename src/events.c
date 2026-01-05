/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Event Subsystem for the Thunderscope LiteX design
 *
 * Copyright (C) 2025 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#include "events.h"
#include "csr.h"
#include "util.h"


#define EVENT_SOURCE_SW         (0)
#define EVENT_SOURCE_EXT_SYNC   (1)

#define EVENT_OUTPUT_EXT_SYNC   (16)

#define EXT_SYNC_DISABLED       (0x00)
#define EXT_SYNC_INPUT          (0x01)
#define EXT_SYNC_OUTPUT         (0x02)

#define EVENT_FIFO_FLUSH        (1UL << 31)


// Defaults
#define EXT_SYNC_PULSE_WIDTH_DEFAULT    (50)
#define EVENT_SOURCE_ENABLE_DEFAULT     (1UL << EVENT_SOURCE_SW)


static const tsEvent_t g_EventNone = {
    .ID = TS_EVT_NONE,
    .event_sample = 0
};


int32_t events_initialize(file_t handle)
{
    int32_t status = TS_STATUS_OK;
    uint32_t evt_data;

    // Default Sync pin to Disabled
    status = events_set_ext_sync(handle, TS_SYNC_DISABLED);

    // Enable Default Event Sources and disable all Outputs
    evt_data = EVENT_SOURCE_ENABLE_DEFAULT;
    // Flush the Event queue
    evt_data |= EVENT_FIFO_FLUSH;
    LOG_DEBUG("Set Event Control 0x%X", evt_data);
    litepcie_writel(handle, CSR_EVENTS_ENGINE_CONTROL_ADDR, evt_data);

    // Default Ext Pulse Width
    litepcie_writel(handle, CSR_EVENTS_EXT_SYNC_PULSE_LEN_ADDR, EXT_SYNC_PULSE_WIDTH_DEFAULT);

    return status;
}

int32_t events_flush(file_t handle)
{
    if(handle == INVALID_HANDLE_VALUE)
    {
        LOG_ERROR("Invalid Event Handle");
        return TS_STATUS_ERROR;
    }
    uint32_t event_cfg = litepcie_readl(handle, CSR_EVENTS_ENGINE_CONTROL_ADDR);
    event_cfg |= EVENT_FIFO_FLUSH;
    LOG_DEBUG("Flush Events Queue");
    litepcie_writel(handle, CSR_EVENTS_ENGINE_CONTROL_ADDR, event_cfg);

    return TS_STATUS_OK;
}

bool events_available(file_t handle)
{
    if(handle != INVALID_HANDLE_VALUE && 
        litepcie_readl(handle, CSR_EVENTS_ENGINE_EVENT_ADDR) & (1UL << CSR_EVENTS_ENGINE_EVENT_PENDING_OFFSET))
    {
        return true;
    }
    else
    {
        return false;
    }
}

int32_t events_get_next(file_t handle, tsEvent_t *pEvent)
{
    if(handle == INVALID_HANDLE_VALUE)
    {
        LOG_ERROR("Invalid Event Handle");
        return TS_STATUS_ERROR;
    }
    else if(pEvent == NULL)
    {
        LOG_ERROR("Invalid Event pointer");
        return TS_STATUS_ERROR;
    }

    uint32_t evt_data = 0;
    
    // Read Sample, then Type
    pEvent->event_sample = (uint64_t)litepcie_readl(handle, CSR_EVENTS_ENGINE_FIFO_READMARKER_ADDR) << 32;
    pEvent->event_sample += (uint64_t)litepcie_readl(handle, CSR_EVENTS_ENGINE_FIFO_READMARKER_ADDR + 4);

    evt_data = litepcie_readl(handle, CSR_EVENTS_ENGINE_FIFO_READSOURCE_ADDR);
    if((evt_data & 0xF) == EVENT_SOURCE_SW)
    {
        pEvent->ID = TS_EVT_HOST_SW;
    }
    else if((evt_data & 0xF) == EVENT_SOURCE_EXT_SYNC)
    {
        pEvent->ID = TS_EVT_EXT_SYNC;
    }
    else
    {
        LOG_ERROR("Unknown Event Source: %x", evt_data);
        return TS_STATUS_ERROR;
    }

    return TS_STATUS_OK;
}

int32_t events_set_periodic(file_t handle, uint32_t period_us)
{
    if(handle == INVALID_HANDLE_VALUE)
    {
        LOG_ERROR("Invalid Event Handle");
        return TS_STATUS_ERROR;
    }

    if(period_us == 0)
    {
        //Disable periodic event generation
        litepcie_writel(handle, CSR_EVENTS_GENERATOR_CONTROL_ADDR, 0);
    }
    else
    {
        litepcie_writel(handle, CSR_EVENTS_GENERATOR_TIMEOUT_ADDR, period_us);
        litepcie_writel(handle, CSR_EVENTS_GENERATOR_CONTROL_ADDR, (1 << CSR_EVENTS_GENERATOR_CONTROL_PERIODIC_OFFSET));
    }

    return TS_STATUS_OK;
}

int32_t events_set_immediate(file_t handle)
{
    if(handle == INVALID_HANDLE_VALUE)
    {
        LOG_ERROR("Invalid Event Handle");
        return TS_STATUS_ERROR;
    }

    litepcie_writel(handle, CSR_EVENTS_GENERATOR_CONTROL_ADDR, (1 << CSR_EVENTS_GENERATOR_CONTROL_IMMEDIATE_OFFSET));
    return TS_STATUS_OK;
}

uint32_t events_get_source_status(file_t handle)
{
    return litepcie_readl(handle, CSR_EVENTS_ENGINE_STATUS_ADDR) & 0xFFFF;
}

int32_t events_set_ext_sync(file_t handle, tsSyncMode_t mode)
{
    if(handle == INVALID_HANDLE_VALUE)
    {
        LOG_ERROR("Invalid Event Handle");
        return TS_STATUS_ERROR;
    }
    
    int32_t status = TS_STATUS_ERROR;
    uint32_t evt_ctrl = litepcie_readl(handle, CSR_EVENTS_ENGINE_CONTROL_ADDR);
    evt_ctrl &= ~(EVENT_FIFO_FLUSH);

    switch (mode)
    {
    case TS_SYNC_DISABLED:
    {
        //Disable Ext Output
        evt_ctrl &= ~(1UL << EVENT_OUTPUT_EXT_SYNC);
        //Disable Ext Input
        evt_ctrl &= ~(1UL << EVENT_SOURCE_EXT_SYNC);
        LOG_DEBUG("Set Event Control 0x%X", evt_ctrl);
        litepcie_writel(handle, CSR_EVENTS_ENGINE_CONTROL_ADDR, evt_ctrl);
        litepcie_writel(handle, CSR_EVENTS_EXT_SYNC_CONTROL_ADDR, EXT_SYNC_DISABLED);
        status = TS_STATUS_OK;
        break;
    }
    case TS_SYNC_IN:
    {
        //Disable Ext Output
        evt_ctrl &= ~(1UL << EVENT_OUTPUT_EXT_SYNC);
        litepcie_writel(handle, CSR_EVENTS_ENGINE_CONTROL_ADDR, evt_ctrl);

        //Set SYNC IN
        litepcie_writel(handle, CSR_EVENTS_EXT_SYNC_CONTROL_ADDR, EXT_SYNC_INPUT);
        
        //Enable Ext Input
        evt_ctrl |= (1UL << EVENT_SOURCE_EXT_SYNC);
        LOG_DEBUG("Set Event Control 0x%X", evt_ctrl);
        litepcie_writel(handle, CSR_EVENTS_ENGINE_CONTROL_ADDR, evt_ctrl);
        status = TS_STATUS_OK;
        break;
    }
    case TS_SYNC_OUT:
    {
        //Disable Ext Input
        evt_ctrl &= ~(1UL << EVENT_SOURCE_EXT_SYNC);
        litepcie_writel(handle, CSR_EVENTS_ENGINE_CONTROL_ADDR, evt_ctrl);
        
        //Set SYNC OUT
        litepcie_writel(handle, CSR_EVENTS_EXT_SYNC_CONTROL_ADDR, EXT_SYNC_OUTPUT);
        
        //Enable Ext Output
        evt_ctrl |= (1UL << EVENT_OUTPUT_EXT_SYNC);
        LOG_DEBUG("Set Event Control 0x%X", evt_ctrl);
        litepcie_writel(handle, CSR_EVENTS_ENGINE_CONTROL_ADDR, evt_ctrl);

        status = TS_STATUS_OK;
        break;
    }
    default:
    {
        LOG_ERROR("Invalid Sync Mode: %x", mode);
        break;
    }
    }

    return status;
}
