/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Provide methods to fetch and store content in flash
 *
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 */

#include <string.h>
#include <json.h>

#include "ts_data.h"
#include "ts_fw_manager.h"
#include "util.h"

static inline bool json_obj_type_is_number(const struct json_object* obj)
{
    return ((json_object_get_type(obj) == json_type_double) || (json_object_get_type(obj) == json_type_int));
}

static bool ts_parse_channel_bitmap(struct json_object *channel_list_obj, tsChannelsActive_t *chan, uint32_t *count)
{
    struct json_object *ch_obj;
    size_t ch_idx = 0;
    int32_t ch_num = 0;
    uint32_t ch_bitmap = 0;

    *count = 0;

    if (json_object_get_type(channel_list_obj) != json_type_array)
    {
        return false;
    }

    ch_obj = json_object_array_get_idx(channel_list_obj, ch_idx);
    while (ch_obj != NULL)
    {
        if (json_object_get_type(ch_obj) == json_type_int)
        {
            ch_num = json_object_get_int(ch_obj);
            if (ch_num < 0 || ch_num >= TS_NUM_CHANNELS)
            {
                LOG_ERROR("Calibration invalid channel number %d", ch_num);
                return false;
            }
            ch_bitmap |= (1 << ch_num);
            (*count)++;
        }
        else
        {
            LOG_ERROR("Calibration failed to parse channel list - %s", json_object_get_string(ch_obj));
            return false;
        }

        if (*count > TS_NUM_CHANNELS)
        {
            LOG_ERROR("Calibration error, channel list too long");
            return false;
        }

        ch_idx++;
        ch_obj = json_object_array_get_idx(channel_list_obj, ch_idx);
    }

    *chan = (tsChannelsActive_t)ch_bitmap;
    return true;
}

static bool ts_parse_afe_cal(tsChannelCalibration_t *afe_cal, struct json_object *afe_obj)
{
    bool status = true;
    struct json_object *item;
    struct json_object *arr_obj;
    struct json_object *path_obj;
    bool high_gain = false;
    uint32_t ladder_idx = 0;
    size_t path_idx = 0;
    tsAfePathCalibration_t* path_cal;

    // Parse Attenuator Scale
    if (json_object_object_get_ex(afe_obj, "attenuatorScale", &item) && json_obj_type_is_number(item))
    {
        afe_cal->attenuatorScale = json_object_get_double(item);
    }
    else
    {
        LOG_ERROR("Calibration error parsing Attenuator Scale");
        status = false;
    }

    // Parse Path Cal
    if (json_object_object_get_ex(afe_obj, "path", &arr_obj) && json_object_get_type(arr_obj) == json_type_array)
    {
        path_obj = json_object_array_get_idx(arr_obj, path_idx);
        while (path_obj != NULL)
        {
            // Get Gain and Ladder Index
            if (json_object_object_get_ex(path_obj, "pgaPreampGain", &item) && json_object_get_type(item) == json_type_string)
            {
                high_gain = (0 == strncmp("high", json_object_get_string(item), 4)) ? true : false;
            }
            else
            {
                LOG_ERROR("Calibration Error parsing PGA Gain");
                status = false;
            
                path_idx++;
                path_obj = json_object_array_get_idx(arr_obj, path_idx);
                continue;
            }
            
            if (json_object_object_get_ex(path_obj, "pgaLadder", &item) && json_object_get_type(item) == json_type_int)
            {
                ladder_idx = json_object_get_int(item);

                if ((ladder_idx < 0) || (ladder_idx >= TS_CAL_NUM_PATHS))
                {
                    LOG_ERROR("Calibration Error invalid PGA Ladder %d", ladder_idx);
                    status = false;
                    path_idx++;
                    path_obj = json_object_array_get_idx(arr_obj, path_idx);
                    continue;    
                }
            }
            else
            {
                LOG_ERROR("Calibration Error parsing PGA Ladder");
                status = false;
                path_idx++;
                path_obj = json_object_array_get_idx(arr_obj, path_idx);
                continue;
            }

            path_cal = high_gain ? &afe_cal->highPgaPathCal[ladder_idx] : &afe_cal->lowPgaPathCal[ladder_idx];

            // Parse Path variables
            if (json_object_object_get_ex(path_obj, "trimDPot", &item) && json_object_get_type(item) == json_type_int)
                path_cal->trimDPot = json_object_get_int(item);
            if (json_object_object_get_ex(path_obj, "trimDacScale", &item) && json_obj_type_is_number(item))
                path_cal->trimOffsetDacScale = json_object_get_double(item);
            if (json_object_object_get_ex(path_obj, "trimDacZeroM", &item) && json_obj_type_is_number(item))
                path_cal->trimOffsetDacZeroM = json_object_get_double(item);
            if (json_object_object_get_ex(path_obj, "trimDacZeroC", &item) && json_obj_type_is_number(item))
                path_cal->trimOffsetDacZeroC = json_object_get_double(item);
            if (json_object_object_get_ex(path_obj, "bufferInputVpp", &item) && json_obj_type_is_number(item))
                path_cal->bufferInputVpp = json_object_get_double(item);
            
            path_idx++;
            path_obj = json_object_array_get_idx(arr_obj, path_idx);
        }
    }
    else
    {
        LOG_ERROR("Calibration error parsing Channel Path");
        status = false;
    }
    return status;
}

static bool ts_parse_adc_cal(tsAdcCalibration_t *adc_cal, struct json_object *adc)
{
    bool status = true;
    struct json_object *item;
    struct json_object *cal_obj;
    struct json_object *rate_obj;
    struct json_object *scale_obj;
    struct json_object *gain_obj;
    struct json_object *arr_obj;
    size_t load_idx = 0;
    size_t rate_idx = 0;
    int32_t int_value;
    tsChannelsActive_t channels;

    // Parse Load Scale
    if (json_object_object_get_ex(adc, "loadScale", &cal_obj) && json_object_get_type(cal_obj) == json_type_array)
    {
        scale_obj = json_object_array_get_idx(cal_obj, load_idx);
        while (scale_obj != NULL)
        {
            if (load_idx >= TS_CAL_NUM_LOADS)
            {
                LOG_ERROR("Too many entries in loadScale");
                status = false;
                break;
            }

            uint32_t count;
            if (json_object_object_get_ex(scale_obj, "channel", &item) && json_object_get_type(item) == json_type_array &&
                ts_parse_channel_bitmap(item, &channels, &count))
            {
                adc_cal->loadCal[load_idx].channels = channels;

                if(json_object_object_get_ex(scale_obj, "rateScale", &item) && json_object_get_type(item) == json_type_array)
                {
                    rate_idx = 0;
                    rate_obj = json_object_array_get_idx(item, rate_idx);
                    while (rate_obj != NULL)
                    {
                        struct json_object *val_obj;

                        if (rate_idx >= TS_CAL_NUM_RATES)
                        {
                            LOG_ERROR("Too many entries in rateScale");
                            status = false;
                            break;
                        }

                        if (json_object_object_get_ex(rate_obj, "rate", &val_obj) && json_object_get_type(val_obj) == json_type_int)
                            adc_cal->loadCal[load_idx].conf[rate_idx].rate = json_object_get_int(val_obj);
    
                        if (json_object_object_get_ex(rate_obj, "scale", &arr_obj) &&
                                json_object_get_type(arr_obj) == json_type_array)
                        {
                            for (size_t ch=0; ch < count; ch++)
                            {
                                val_obj = json_object_array_get_idx(arr_obj, ch);
                                if (val_obj != NULL && json_obj_type_is_number(val_obj))
                                    adc_cal->loadCal[load_idx].conf[rate_idx].scale[ch] = json_object_get_double(val_obj);
                            }
                        }

                        rate_idx++;
                        rate_obj = json_object_array_get_idx(item, rate_idx);
                    }
                }
            }
            else
            {
                LOG_ERROR("Unknown channel list in LoadScale[%d]", load_idx);
                status = false;
            }

            load_idx++;
            scale_obj = json_object_array_get_idx(cal_obj, load_idx);
        }
    }

    // Parse Branch Gain
    if (json_object_object_get_ex(adc, "branchGain", &cal_obj) && json_object_get_type(cal_obj) == json_type_array)
    {
        load_idx = 0;
        gain_obj = json_object_array_get_idx(cal_obj, load_idx);
        while (gain_obj != NULL)
        {
            if (load_idx >= TS_CAL_NUM_LOADS)
            {
                LOG_ERROR("Too many entries in loadScale");
                status = false;
                break;
            }

            uint32_t count;
            if (json_object_object_get_ex(gain_obj, "channel", &item) && json_object_get_type(item) == json_type_array &&
                ts_parse_channel_bitmap(item, &channels, &count))
            {
                adc_cal->branchFineGain[load_idx].channels = channels;

                if(json_object_object_get_ex(gain_obj, "rateGain", &item) && json_object_get_type(item) == json_type_array)
                {
                    rate_idx = 0;
                    rate_obj = json_object_array_get_idx(item, rate_idx);
                    while (rate_obj != NULL)
                    {
                        struct json_object *val_obj;
                        
                        if (rate_idx >= TS_CAL_NUM_RATES)
                        {
                            LOG_ERROR("Too many entries in rateGain");
                            status = false;
                            break;
                        }

                        if (json_object_object_get_ex(rate_obj, "rate", &val_obj) && json_object_get_type(val_obj) == json_type_int)
                            adc_cal->branchFineGain[load_idx].conf[rate_idx].rate = json_object_get_int(val_obj);
    
                        if (json_object_object_get_ex(rate_obj, "gain", &arr_obj) &&
                                json_object_get_type(arr_obj) == json_type_array && json_object_array_length(arr_obj) == 8)
                        {
                            for (size_t branch = 0; branch < 8; branch++)
                            {
                                val_obj = json_object_array_get_idx(arr_obj, branch);
                                if (val_obj != NULL && json_obj_type_is_number(val_obj))
                                    adc_cal->branchFineGain[load_idx].conf[rate_idx].gain[branch] = (int8_t)json_object_get_int(val_obj);
                            }
                        }
                        else
                        {
                            LOG_ERROR("Invalid Branch Gain config");
                        }

                        rate_idx++;
                        rate_obj = json_object_array_get_idx(item, rate_idx);
                    }
                }
            }
            else
            {
                LOG_ERROR("Unknown channel list in LoadScale[%d]", load_idx);
                status = false;
            }

            load_idx++;
            gain_obj = json_object_array_get_idx(cal_obj, load_idx);
        }
    }

    return status;
}

int32_t ts_data_parse_factory_cal(uint8_t* cal_buffer, tsScopeCalibration_t *fcal)
{
    struct json_object *json_data;
    struct json_object *item;
    int32_t cal_version = -1;
    size_t ch_idx = 0;
    int32_t ret = TS_STATUS_ERROR;

    json_data = json_tokener_parse(cal_buffer);
    if (!json_data)
    {
        LOG_ERROR("Failed to parse Factory Calibration JSON");
        free(cal_buffer);
        return TS_STATUS_ERROR;
    }

    // Initialize calibration data
    memset(fcal, 0, sizeof(tsScopeCalibration_t));

    // Check the version
    if(json_object_object_get_ex(json_data,"version", &item) && json_object_get_type(item) == json_type_int)
    {
        cal_version = json_object_get_int(item);
    }
    else
    {
        LOG_ERROR("Calibration Version key not found or invalid type");
    }

    if (cal_version == 1)
    {
        LOG_DEBUG("Parsing Factory Calibration version 1");
        if (json_object_object_get_ex(json_data, "serial", &item) && json_object_get_type(item) == json_type_string)
        {
            LOG_DEBUG("\tSerial: %s", json_object_get_string(item));
        }
        if (json_object_object_get_ex(json_data, "timestamp", &item) && json_object_get_type(item) == json_type_string)
        {
            LOG_DEBUG("\tTimestamp: %s", json_object_get_string(item));
        }

        if (json_object_object_get_ex(json_data, "frontend", &item) && json_object_get_type(item) == json_type_array)
        {
            // Parse the AFE calibration for each channel
            struct json_object *ch_obj = json_object_array_get_idx(item, ch_idx);
            while (ch_obj != NULL)
            {
                struct json_object *id_obj;
                int32_t chan;
                if(json_object_object_get_ex(ch_obj, "channel", &id_obj) &&
                    json_object_get_type(id_obj) == json_type_int)
                {
                    // Get which channel this element if for, order may not match index
                    chan = json_object_get_int(id_obj);
                    if (chan >= 0 && chan < TS_NUM_CHANNELS)
                    {
                        if (!ts_parse_afe_cal(&fcal->afeCal[chan], ch_obj))
                        {
                            LOG_ERROR("Problem while reading calibration data for Channel %d AFE", chan);
                        }
                    }
                    else
                    {
                        LOG_ERROR("Bad AFE calibration channel %d", chan);
                    }
                }
                else
                {
                    LOG_ERROR("Cannot retrieve AFE channel ID: %d", ch_idx);
                }

                ch_idx++;
                ch_obj = json_object_array_get_idx(item, ch_idx);
            }
        }

        if (json_object_object_get_ex(json_data, "adc", &item) && json_object_get_type(item) == json_type_object)
        {
            if (!ts_parse_adc_cal(&fcal->adcCal, item))
            {
                LOG_ERROR("Problem while reading calibration data for ADC");
            }
        }
        ret = TS_STATUS_OK;
    }
    else
    {
        LOG_ERROR("Unsupported Calibration Version: %d", cal_version);
    }

    // Cleanup
    json_object_put(json_data);

    return ret;
}

int32_t ts_data_factory_cal_get(ts_fw_manager_t* mngr, tsScopeCalibration_t *fcal)
{
    uint32_t cal_len = 0;
    uint8_t* cal_buffer = NULL;
    int32_t ret = TS_STATUS_ERROR;

    cal_len = ts_fw_manager_factory_data_get_length(mngr, TAG_FCAL);

    if(cal_len <= 0)
    {
        LOG_ERROR("Could not read Factory Calibration data - %d", cal_len);
        return TS_STATUS_ERROR;
    }

    cal_buffer = malloc(cal_len + 1);
    if (cal_buffer == NULL)
    {
        LOG_ERROR("Failed to allocate local buffer for cal data");
        return TS_STATUS_ERROR;
    }

    if(TS_STATUS_ERROR == ts_fw_manager_factory_data_retreive(mngr, TAG_FCAL, cal_buffer, cal_len))
    {
        LOG_ERROR("Failed to read Factory Calibration data");
        free(cal_buffer);
        return TS_STATUS_ERROR;
    }

    cal_buffer[cal_len] = '\0';

    ret = ts_data_parse_factory_cal(cal_buffer, fcal);

    free(cal_buffer);

    return ret;
}

int32_t ts_data_factory_id_get(ts_fw_manager_t* mngr, tsDeviceInfo_t* infos)
{
    struct json_object *fid;
    struct json_object *item;
    uint32_t hwid_version = 0;
    uint32_t hwid_len = 0;
    uint8_t* hwid_buffer = NULL;

    // Check HWID tag in factory partition
    hwid_len = ts_fw_manager_factory_data_get_length(mngr, TAG_HWID);

    if(hwid_len <= 0)
    {
        LOG_ERROR("Could not read mfg data");
        return TS_STATUS_ERROR;
    }

    // Read HWID to buffer
    hwid_buffer = malloc(hwid_len + 1);
    if(TS_STATUS_ERROR == ts_fw_manager_factory_data_retreive(mngr, TAG_HWID, hwid_buffer, hwid_len))
    {
        LOG_ERROR("Failed to read HWID");
        free(hwid_buffer);
        return TS_STATUS_ERROR;
    }

    // Parse json
    hwid_buffer[hwid_len] = '\0';
    fid = json_tokener_parse(hwid_buffer);
    
    // Store HWID version
    if(json_object_object_get_ex(fid,"version", &item))
    {
        if(json_object_get_type(item) == json_type_int)
        {
            hwid_version = json_object_get_int(item);
        }
        else
        {
            LOG_ERROR("HWID Version bad value");
        }
    }
    else
    {
        LOG_ERROR("HWID Version key not found");
    }

    // Fill struct with info
    if(json_object_object_get_ex(fid,"serial", &item))
    {
        if(json_object_get_type(item) == json_type_string)
        {
            strncpy(infos->serial_number, json_object_get_string(item), TS_IDENT_STR_LEN);
        }
        else
        {
            LOG_ERROR("Serial Number bad value");
        }
    }
    else
    {
        LOG_ERROR("Serial Number key not found");
    }

    if(json_object_object_get_ex(fid,"boardRevision", &item))
    {
        if(json_object_get_type(item) == json_type_int)
        {
            infos->board_rev = json_object_get_int(item);
        }
        else
        {
            LOG_ERROR("Board Revision bad value");
        }
    }
    else
    {
        LOG_ERROR("Board Revision key not found");
    }

    if(json_object_object_get_ex(fid,"buildConfiguration", &item))
    {
        if(json_object_get_type(item) == json_type_string)
        {
            strncpy(infos->build_config, json_object_get_string(item), TS_IDENT_STR_LEN);
        }
        else
        {
            LOG_ERROR("Build Config bad value");
        }
    }
    else
    {
        LOG_ERROR("Build Config key not found");
    }

    // Build Date
    if(json_object_object_get_ex(fid,"buildDate", &item))
    {
        if(json_object_get_type(item) == json_type_string)
        {
            strncpy(infos->build_date, json_object_get_string(item), TS_IDENT_STR_LEN);
        }
        else
        {
            LOG_ERROR("Build Date bad value");
        }
    }
    else
    {
        LOG_ERROR("Build Date key not found");
    }

    // Signature
    if(json_object_object_get_ex(fid,"manufacturingSignature", &item))
    {
        if(json_object_get_type(item) == json_type_string)
        {
            strncpy(infos->mfg_signature, json_object_get_string(item), TS_IDENT_STR_LEN);
        }
        else
        {
            LOG_ERROR("Mfg Signature bad value");
        }
    }
    else
    {
        LOG_ERROR("Mfg Signature key not found");
    }

    free(hwid_buffer);
    return TS_STATUS_OK;
}
