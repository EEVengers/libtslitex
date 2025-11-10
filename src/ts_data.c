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


int32_t ts_data_factory_cal_get(ts_fw_manager_t* mngr, tsScopeCalibration_t *fcal)
{
    // struct json_object * fcal;

    return TS_STATUS_OK;
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
    if(json_object_object_get_ex(fid,"Serial Number", &item))
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

    if(json_object_object_get_ex(fid,"Board Revision", &item))
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

    if(json_object_object_get_ex(fid,"Build Config", &item))
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
    if(json_object_object_get_ex(fid,"Build Date", &item))
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
    if(json_object_object_get_ex(fid,"Mfg Signature", &item))
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
