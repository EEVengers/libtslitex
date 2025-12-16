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
#include <string.h>
#include <fcntl.h>

#include "thunderscope.h"
#include "ts_common.h"
#include "ts_calibration.h"
#include "ts_channel.h"
#include "samples.h"
#include "gpio.h"
#include "ts_fw_manager.h"
#include "ts_data.h"
#include "util.h"

#include "litepcie.h"


#if defined(_WIN32)
#define FILE_FLAGS  (FILE_ATTRIBUTE_NORMAL)
#else
#define FILE_FLAGS  (O_RDWR)
#endif

#define DEFAULT_DATA_INTR_RATE   (100)

typedef struct ts_inst_s
{
    file_t ctrl;
    tsDeviceInfo_t identity;
    tsChannelHdl_t pChannel;
    sampleStream_t samples;
    gpio_t status_leds;
    const led_signals_t *signals;
    bool initialized;
    ts_fw_manager_t fw;
    uint32_t interrupt_rate;
    uint8_t bytes_per_sample;
    //TBD - Other Instance Data
} ts_inst_t;


int32_t thunderscopeListDevices(uint32_t devIndex, tsDeviceInfo_t *info)
{
    int32_t retVal = TS_STATUS_ERROR;
    char testPath[TS_IDENT_STR_LEN];
    ts_fw_manager_t fwmngr;
    
    // Find device path by index
    snprintf(testPath, TS_IDENT_STR_LEN, LITEPCIE_CTRL_NAME(%d), devIndex);
    file_t testDev = litepcie_open((const char*)testPath, FILE_FLAGS);

    //If index valid
    if(testDev != INVALID_HANDLE_VALUE)
    {
        info->device_id = devIndex;
        info->hw_id = litepcie_readl(testDev, CSR_DEV_STATUS_HW_ID_ADDR);
        info->gw_id = litepcie_readl(testDev, CSR_DEV_STATUS_GW_REV_ADDR);
        info->litex = litepcie_readl(testDev, CSR_DEV_STATUS_LITEX_REL_ADDR);
        //Copy device identifier
        for (uint32_t i = 0; i < TS_IDENT_STR_LEN; i++)
        {
            info->identity[i] = (char)litepcie_readl(testDev, CSR_IDENTIFIER_MEM_BASE + 4 * i);
        }
        strncpy(info->device_path, testPath, TS_IDENT_STR_LEN);
        // Get Factory Build Info
        ts_fw_manager_init(testDev, &fwmngr);
        ts_data_factory_id_get(&fwmngr, info);

        litepcie_close(testDev);
        retVal = TS_STATUS_OK;
    }

    return retVal;
}

tsHandle_t thunderscopeOpen(uint32_t devIdx, bool skip_init)
{
    ts_inst_t* pInst = calloc(sizeof(ts_inst_t), 1);
    char devName[TS_IDENT_STR_LEN] = {0};

    if(pInst)
    {
        pInst->initialized = false;
        snprintf(devName, TS_IDENT_STR_LEN, LITEPCIE_CTRL_NAME(%d), devIdx);
        pInst->ctrl = litepcie_open(devName, FILE_FLAGS);
    }
    else
    {
        LOG_ERROR("Litepcie Failed to allocate device");
        return NULL;
    }

    if(pInst->ctrl == INVALID_HANDLE_VALUE)
    {
        LOG_ERROR("litepcie_open returned Invalid File Handle for device %d (%s)", devIdx, devName);
        free(pInst);
        return NULL;
    }
    
    //Get Device Info
    pInst->identity.device_id = devIdx;
    pInst->identity.hw_id = litepcie_readl(pInst->ctrl, CSR_DEV_STATUS_HW_ID_ADDR);
    pInst->identity.gw_id = litepcie_readl(pInst->ctrl, CSR_DEV_STATUS_GW_REV_ADDR);
    pInst->identity.litex = litepcie_readl(pInst->ctrl, CSR_DEV_STATUS_LITEX_REL_ADDR);
    //Copy device identifier
    for (uint32_t i = 0; i < TS_IDENT_STR_LEN; i++)
    {
        pInst->identity.identity[i] = (char)litepcie_readl(pInst->ctrl, CSR_IDENTIFIER_MEM_BASE + 4 * i);
    }
    
    
    strncpy(pInst->identity.device_path, devName, TS_IDENT_STR_LEN);

    if(!skip_init)
    {
        pInst->interrupt_rate = DEFAULT_DATA_INTR_RATE;
        if(TS_STATUS_OK != ts_channel_init(&pInst->pChannel, pInst->ctrl))
        {
            LOG_ERROR("Failed to initialize channels");
            litepcie_close(pInst->ctrl);
            free(pInst);
            return NULL;
        }

        if(TS_STATUS_OK != samples_init(&pInst->samples, devIdx, 0))
        {
            LOG_ERROR("Failed to initialize samples");
            ts_channel_destroy(pInst->pChannel);
            litepcie_close(pInst->ctrl);
            free(pInst);
            return NULL;
        }
        pInst->bytes_per_sample = 1;
        pInst->initialized = true;
    }

    if(TS_STATUS_OK != ts_fw_manager_init(pInst->ctrl, &pInst->fw))
    {
        LOG_ERROR("Failed to initialize spiflash");
        samples_teardown(&pInst->samples);
        ts_channel_destroy(pInst->pChannel);
        litepcie_close(pInst->ctrl);
        free(pInst);
        return NULL;
    }

    // Get Factory Build Info
    ts_data_factory_id_get(&pInst->fw, &pInst->identity);

    pInst->status_leds.fd = pInst->ctrl;
    pInst->status_leds.reg = TS_STATUS_LED_ADDR;
    pInst->status_leds.bit_mask = TS_STATUS_LED_MASK;
    if(pInst->identity.hw_id & TS_HW_ID_VALID_MASK)
    {
        pInst->signals = &ts_dev_leds;
    }
    else
    {
        pInst->signals = &ts_beta_leds;
    }
    gpio_group_set(pInst->status_leds, pInst->signals->ready);

    LOG_DEBUG("Opened TS idx %d with handle %p", devIdx, pInst);
    return (tsHandle_t)pInst;
}

int32_t thunderscopeClose(tsHandle_t ts)
{
    if(ts == NULL)
    {
        return TS_STATUS_ERROR;
    }

    ts_inst_t* pInst = (ts_inst_t*)ts;
    
    if(pInst->initialized)
    {
        samples_teardown(&pInst->samples);
        ts_channel_destroy(pInst->pChannel);
    }
    
    gpio_group_set(pInst->status_leds, pInst->signals->disabled);
    litepcie_close(pInst->ctrl);
    free(pInst);

    return TS_STATUS_OK;
}

int32_t thunderscopeChannelConfigGet(tsHandle_t ts, uint32_t channel, tsChannelParam_t* conf)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;

    if(pInst && pInst->initialized)
    {
        return ts_channel_params_get(pInst->pChannel, channel, conf);
    }

    return TS_STATUS_ERROR;
}

int32_t thunderscopeChannelConfigSet(tsHandle_t ts, uint32_t channel, tsChannelParam_t* conf)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;

    if(pInst && pInst->initialized)
    {
        return ts_channel_params_set(pInst->pChannel, channel, conf);
    }

    return TS_STATUS_ERROR;
}

int32_t thunderscopeRefClockSet(tsHandle_t ts, tsRefClockMode_t mode, uint32_t refclk_freq)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;

    if(pInst && pInst->initialized)
    {
        return ts_channel_ext_clock_config(pInst->pChannel, mode, refclk_freq);
    }

    return TS_STATUS_ERROR;
}

int32_t thunderscopeStatusGet(tsHandle_t ts, tsScopeState_t* state)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    if(!state || !pInst || !pInst->initialized)
    {
        return TS_STATUS_ERROR;
    }

    *state = ts_channel_scope_status(pInst->pChannel);
    samples_update_status(&pInst->samples);
    state->adc_lost_buffer_count = pInst->samples.dropped_buffer_count;
    return TS_STATUS_OK;
}

int32_t thunderscopeSampleModeSet(tsHandle_t ts, uint32_t rate, tsSampleFormat_t mode)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;

    if(pInst && pInst->initialized)
    {
        int32_t status = ts_channel_sample_rate_set(pInst->pChannel, rate, mode);
        if(status == TS_STATUS_OK)
        {
            //Target 100Hz interrupt rate
            pInst->samples.interrupt_rate  = 1 + ((((mode == TS_8_BIT) ? rate : 2*rate) / 
                                                (DMA_BUFFER_SIZE)) / pInst->interrupt_rate);

            LOG_DEBUG("DMA Interrupt Rate is 1/%d MB", pInst->samples.interrupt_rate);
        }
        return status;
    }

    return TS_STATUS_ERROR;
}

int32_t thunderscopeSampleInterruptRate(tsHandle_t ts, uint32_t interrupt_rate)
{
     ts_inst_t* pInst = (ts_inst_t*)ts;

    if(pInst && pInst->initialized)
    {
        if(interrupt_rate == 0)
        {
            interrupt_rate = DEFAULT_DATA_INTR_RATE;
        }
        pInst->interrupt_rate = interrupt_rate;
        
        LOG_DEBUG("Targeting Interrupt frequency of %d Hz", pInst->interrupt_rate);
        
        return TS_STATUS_OK;
    }

    return TS_STATUS_ERROR;
}

int32_t thunderscopeChanCalibrationSet(tsHandle_t ts, uint32_t channel, tsChannelCalibration_t *cal)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;

    if(pInst && pInst->initialized)
    {
        return ts_channel_calibration_set(pInst->pChannel, channel, cal);
    }
    
    return TS_STATUS_ERROR;
}

int32_t thunderscopeChanCalibrationGet(tsHandle_t ts, uint32_t channel, tsChannelCalibration_t *cal)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;

    if(pInst && pInst->initialized)
    {
        return ts_channel_calibration_get(pInst->pChannel, channel, cal);
    }
    
    return TS_STATUS_ERROR;
}

int32_t thunderscopeAdcCalibrationSet(tsHandle_t ts, tsAdcCalibration_t *cal)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;

    if(pInst && pInst->initialized)
    {
        return ts_channel_adc_calibration_set(pInst->pChannel, cal);
    }
    
    return TS_STATUS_ERROR;
}

int32_t thunderscopeAdcCalibrationGet(tsHandle_t ts, tsAdcCalibration_t *cal)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;

    if(pInst && pInst->initialized)
    {
        return ts_channel_adc_calibration_get(pInst->pChannel, cal);
    }
    
    return TS_STATUS_ERROR;
}

int32_t thunderscopeCalibrationManualCtrl(tsHandle_t ts, uint32_t channel, tsChannelCtrl_t *ctrl)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;

    if(pInst && pInst->initialized)
    {
        return ts_channel_calibration_manual(pInst->pChannel, channel, *ctrl);
    }
    
    return TS_STATUS_ERROR;
}

int32_t thunderscopeCalibrationAdcTest(tsHandle_t ts, tsCalAdcTest_t test_mode, uint32_t test_pattern)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;

    if(pInst && pInst->initialized)
    {
        return ts_channel_set_adc_test(pInst->pChannel, (hmcad15xxTestMode_t)test_mode,
                    (uint16_t)(test_pattern & 0xFFFF), (uint16_t)(test_pattern >> 16));
    }
    
    return TS_STATUS_ERROR;
}

int32_t thunderscopeDataEnable(tsHandle_t ts, uint8_t enable)
{
    int32_t status = TS_STATUS_OK;
    ts_inst_t* pInst = (ts_inst_t*)ts;

    if(!pInst || !pInst->initialized)
    {
        return TS_STATUS_ERROR;
    }

    status = ts_channel_run(pInst->pChannel, enable);
    if(status != TS_STATUS_OK)
    {
        gpio_group_set(pInst->status_leds, pInst->signals->error);
        return status;
    }

    if(enable)
    {
        gpio_group_set(pInst->status_leds, pInst->signals->active);
        //Get current sample increment
        pInst->bytes_per_sample = ts_channel_scope_status(pInst->pChannel).adc_sample_bits / 8;
        uint8_t active_channels = 0;
        for(uint8_t ch=0; ch < TS_NUM_CHANNELS; ch++)
        {
            tsChannelParam_t params;
            ts_channel_params_get(pInst->pChannel, ch, &params);
            if(params.active)
            {
                active_channels++;
            }
        }
        if(active_channels == 2)
        {
            pInst->bytes_per_sample *= 2;
        }
        else if(active_channels > 2)
        {
            pInst->bytes_per_sample *= 4;
        }
    }
    else
    {
        gpio_group_set(pInst->status_leds, pInst->signals->ready);
    }
    return samples_enable_set(&pInst->samples, enable);
}

int32_t thunderscopeRead(tsHandle_t ts, uint8_t* buffer, uint32_t len)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    if(pInst && pInst->initialized)
    {
        return samples_get_buffers(&pInst->samples, buffer, len);
    }
    else
    {
        return TS_STATUS_ERROR;
    }
}

int32_t thunderscopeReadCount(tsHandle_t ts, uint8_t* buffer, uint32_t len, uint64_t* count)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    int32_t sample_result = TS_STATUS_ERROR;
    if(pInst && pInst->initialized)
    {
        sample_result = samples_get_buffers(&pInst->samples, buffer, len);
        if(sample_result > 0)
        {
            samples_update_status(&pInst->samples);
            *count = ((pInst->samples.driver_buffer_count * DMA_BUFFER_SIZE) - sample_result) / pInst->bytes_per_sample;
        }
        else
        {
            *count = 0;
        }
    }

    return sample_result;
}

int32_t thunderscopeExtSyncConfig(tsHandle_t ts, tsSyncMode_t mode)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    int32_t status = TS_STATUS_ERROR;
    if(pInst)
    {
        //TODO
        status = TS_STATUS_OK;
    }

    return status;
}

int32_t thunderscopeEventSyncAssert(tsHandle_t ts)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    int32_t status = TS_STATUS_ERROR;
    if(pInst)
    {
        //TODO
        status = TS_STATUS_OK;
    }

    return status;
}

int32_t thunderscopeEventGet(tsHandle_t ts, tsEvent_t* evt)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    int32_t status = TS_STATUS_ERROR;
    if(pInst && evt)
    {
        //TODO
        evt->ID = TS_EVT_NONE;
        evt->event_sample = 0;
        status = TS_STATUS_OK;

        //Debug
        static uint64_t lastBuffer = 0;
        if(pInst->samples.driver_buffer_count > (lastBuffer + 950))
        {
            evt->ID = TS_EVT_EXT_SYNC;
            evt->event_sample = ((pInst->samples.driver_buffer_count * DMA_BUFFER_SIZE)) / pInst->bytes_per_sample + 200;
            lastBuffer = pInst->samples.driver_buffer_count;
        }
    }

    return status;
}

int32_t thunderscopeFwUpdate(tsHandle_t ts, const char* bitstream, uint32_t len)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    int32_t status = TS_STATUS_ERROR;
    if(pInst)
    {
        status = ts_fw_manager_user_fw_update(&pInst->fw, bitstream, len);
        if(status == TS_STATUS_OK)
        {
            LOG_DEBUG("Bitstream Update Complete");
        }
        else
        {
            LOG_ERROR("Bitstream Update Failed: %d", status);
        }
    }

    return status;
}

int32_t thunderscopeUserDataRead(tsHandle_t ts, char* buffer, uint32_t offset, uint32_t readLen)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    int32_t status = TS_STATUS_ERROR;
    if(pInst)
    {
        status = ts_fw_manager_user_data_read(&pInst->fw, buffer, offset, readLen);
    }

    return status;
}

int32_t thunderscopeUserDataWrite(tsHandle_t ts, const char* buffer, uint32_t offset, uint32_t writeLen)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    int32_t status = TS_STATUS_ERROR;
    if(pInst)
    {
        status = ts_fw_manager_user_data_write(&pInst->fw, buffer, offset, writeLen);
    }

    return status;
}

int32_t thunderscopeGetFwProgress(tsHandle_t ts, uint32_t* progress)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    if(pInst)
    {
        return ts_fw_manager_get_progress(&pInst->fw, progress);
    }
    else
    {
        return TS_STATUS_ERROR;
    }
}

#if defined(FACTORY_PROVISIONING_API)
int32_t thunderscopeFactoryProvisionPrepare(tsHandle_t ts, uint64_t dna)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    if(pInst)
    {
        return ts_fw_manager_factory_data_erase(&pInst->fw, dna);
    }
    else
    {
        return TS_STATUS_ERROR;
    }
}

int32_t thunderscopeFactoryProvisionAppendTLV(tsHandle_t ts, const uint32_t tag, uint32_t length, const char* content)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    if(pInst)
    {
        return ts_fw_manager_factory_data_append(&pInst->fw, tag, length, content);
    }
    else
    {
        return TS_STATUS_ERROR;
    }
}
#else
int32_t thunderscopeFactoryProvisionPrepare(tsHandle_t ts, uint64_t dna)
{
    LOG_ERROR("Factory API Not Enabled");
    return TS_STATUS_ERROR;
}

int32_t thunderscopeFactoryProvisionAppendTLV(tsHandle_t ts, const uint32_t tag, uint32_t length, const char* content)
{
    LOG_ERROR("Factory API Not Enabled");
    return TS_STATUS_ERROR;
}
#endif

int32_t thunderscopeFactoryProvisionVerify(tsHandle_t ts)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    if(pInst)
    {
        return ts_fw_manager_factory_data_verify(&pInst->fw);
    }
    else
    {
        return TS_STATUS_ERROR;
    }
}

int32_t thunderscopeFactoryReadItem(tsHandle_t ts, const uint32_t tag, char* content_buffer, uint32_t item_max_len)
{
    ts_inst_t* pInst = (ts_inst_t*)ts;
    if(pInst)
    {
        return ts_fw_manager_factory_data_retreive(&pInst->fw, tag, content_buffer, item_max_len);
    }
    else
    {
        return TS_STATUS_ERROR;
    }
}
