/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Provide methods to fetch and store content in flash
 *
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 */

#include <string.h>

#include "ts_fw_manager.h"
#include "platform.h"
#include "spiflash.h"
#include "liblitepcie.h"
#include "util.h"

// From AMD UG470
#define IDCODE_7A200T   (0x03636093)
#define IDCODE_7A100T   (0x03631093)
#define IDCODE_7A50T    (0x0362C093)
#define IDCODE_7A35T    (0x0362D093)
#define IDCODE_MASK     (0x0FFFFFFF)

#define ICAP_REG_IDCODE (0x0C)

static const struct {
    uint32_t code;
    char name[16];
} g_id_map[] = {
    {IDCODE_7A35T, "7a35tcsg325"},
    {IDCODE_7A50T, "7a50tcsg325"},
    {IDCODE_7A100T, "7a100tfgg484"},
    {IDCODE_7A200T, "7a200tfbg484"},
    {0, ""}
};

static void ts_fw_progress_update(void* ctx, uint32_t work_done, uint32_t work_total);

static uint32_t ts_fw_get_idcode(file_t fd)
{
    uint32_t code;

    litepcie_writel(fd, CSR_ICAP_ADDR_ADDR, ICAP_REG_IDCODE);
    litepcie_writel(fd, CSR_ICAP_READ_ADDR, 1);

    LOG_DEBUG("Read ID Code from ICAP");
    while(litepcie_readl(fd, CSR_ICAP_DONE_ADDR) == 0)
    {
        LOG_DEBUG(".");
        ;;
    }
    code = litepcie_readl(fd, CSR_ICAP_DATA_ADDR);
    LOG_DEBUG("Read IDCODE 0x%08X", code);
    return code & IDCODE_MASK;
}

static const char* ts_fw_parse_bit_header(const char* header, const char** part, uint32_t* bin_len)
{
    const char* position = header;
    uint16_t key_len = 0;

    //First Field ('0FF0...')
    key_len = (uint16_t)(position[0] << 8) + (position[1]);
    if(key_len != 9)
    {
        LOG_ERROR("Invalid bitstream FIELD 1 (%d)", key_len);
        return NULL;
    }
    position += (2+key_len);

    //Second Field ('a')
    key_len = (uint16_t)(position[0] << 8) + (position[1]);
    if(key_len != 1 && position[2] != 'a')
    {
        LOG_ERROR("Invalid bitstream FIELD 2 (%d)", key_len);
        return NULL;
    }
    position += (2+key_len);

    //Third Field (Design Name)
    key_len = (uint16_t)(position[0] << 8) + (position[1]);
    LOG_DEBUG("Parsing Bitstream for project: %s", &position[2]);
    position += (2+key_len);

    //Fourth Field ('b' + Part Name)
    if(position[0] != 'b')
    {
        LOG_ERROR("Invalid bitstream FIELD 4 (%c)", position[0]);
        return NULL;
    }
    key_len = (uint16_t)(position[1] << 8) + (position[2]);
    *part = &position[3];
    LOG_DEBUG("Part: %s", &position[3]);
    position += (3+key_len);

    //Fifth Field ('c' + Build Date)
    key_len = (uint16_t)(position[1] << 8) + (position[2]);
    if((position[0] != 'c') || (key_len != 11))
    {
        LOG_ERROR("Invalid bitstream FIELD 5 (%c)", position[0]);
        return NULL;
    }
    LOG_DEBUG("Build Date: %s", &position[3]);
    position += (3+key_len);

    
    //Sixth Field ('d' + Build Time)
    key_len = (uint16_t)(position[1] << 8) + (position[2]);
    if((position[0] != 'd') || (key_len != 9))
    {
        LOG_ERROR("Invalid bitstream FIELD 6 (%c)", position[0]);
        return NULL;
    }
    LOG_DEBUG("Build Time: %s", &position[3]);
    position += (3+key_len);

    
    //Seventh Field ('e' + Bitstream)
    if(position[0] != 'e')
    {
        LOG_ERROR("Invalid bitstream FIELD 6 (%c)", position[0]);
        return NULL;
    }
    *bin_len = (uint32_t)(((uint32_t)position[1] << 24) + ((uint32_t)position[2] << 16) + ((uint32_t)position[3] << 8) + (uint32_t)position[4]);
    LOG_DEBUG("Bitstream Length: %u", *bin_len);
    position += 5;

    return position;
}

int32_t ts_fw_manager_init(file_t fd, ts_fw_manager_t* mngr)
{
    if(mngr == NULL)
    {
        LOG_ERROR("Invalid manager handle");
        return TS_STATUS_ERROR;
    }
    //Initialize SPI Flash
    if(TS_STATUS_OK == spiflash_init(fd, &mngr->flash_dev))
    {
        mngr->flash_dev.op_progress = ts_fw_progress_update;
        mngr->flash_dev.op_progress_ctx = mngr;
    }
    else
    {
        LOG_ERROR("Failed to initialize SPI Flash");
        return TS_STATUS_ERROR;
    }

    //Set partition Table
    if( mngr->flash_dev.mfg_code == TS_FLASH_256M_MFG)
    {
        mngr->partition_table = &ts_256Mb_layout;
    }
    else if( mngr->flash_dev.mfg_code == TS_FLASH_64M_MFG)
    {
        mngr->partition_table = &ts_64Mb_layout;
    }

    // Lock Factory Partitions
    //TODO

    return TS_STATUS_OK;
}

int32_t ts_fw_manager_user_fw_update(ts_fw_manager_t* mngr, const char* file_stream, uint32_t len)
{
    // Verify Bitstream and Strip Header Info
    uint32_t bin_length = 0;
    const char* part_name = NULL;
    atomic_store(&mngr->fw_progress, 0);
    const char* bin_start = ts_fw_parse_bit_header(file_stream, &part_name, &bin_length);
    if(bin_length > len)
    {
        LOG_ERROR("INVALID length in bitstream header (%u > %u)", bin_length, len);
        return TS_STATUS_ERROR;
    }
    LOG_DEBUG("Bitstream Length: %u", bin_length);
    
    mngr->fw_progress_max = bin_length*2;
    
    // Verify FPGA Part 
    // Get IDCODE from TS
    uint32_t idcode;
    bool id_valid = false;
    idcode = ts_fw_get_idcode(mngr->flash_dev.fd);

    // Compare IDCODE with associated Part# from bitstream
    uint32_t idx = 0;
    while(g_id_map[idx].code != 0)
    {
        if(g_id_map[idx].code == idcode)
        {
            if(0 == strncmp(g_id_map[idx].name, part_name, strlen(g_id_map[idx].name)))
            {
                id_valid = true;
            }
            else
            {
                LOG_ERROR("Invalid Update bitstream for device %s", part_name);
            }
            break;
        }
        idx++;
    }

    if(id_valid == false)
    {
        LOG_ERROR("Bitstream IDCODE Validation Failed");
        return TS_INVALID_PARAM;
    }

    // Verify File Length Good
    uint32_t user_partition_len = mngr->partition_table->user_bitstream_end - mngr->partition_table->user_bitstream_start;
    if(bin_length > user_partition_len)
    {
        LOG_ERROR("Bad bitstream length.  Supplied length %u is longer than the allowed %u", bin_length, user_partition_len);
        return TS_INVALID_PARAM;
    }

    // Erase User Flash Partition
    if(TS_STATUS_OK != spiflash_erase(&mngr->flash_dev, mngr->partition_table->user_bitstream_start, bin_length))
    {
        LOG_ERROR("Failed to erase user bitstream partition");
        return TS_STATUS_ERROR;
    }

    atomic_store(&mngr->fw_progress, bin_length);

    // Program New Bitstream
    // TODO: AMD Recommends programming the SYNC word last, so program the bitstream from end to start
    // Ref: AMD AR 58090 [https://adaptivesupport.amd.com/s/article/58090]
    // Ref: AMD AR 58249 [https://adaptivesupport.amd.com/s/article/58249]
    if(bin_length != spiflash_write(&mngr->flash_dev, mngr->partition_table->user_bitstream_start, (const uint8_t*)bin_start, bin_length))
    {
        LOG_ERROR("Failed to write user bitstream partition");
        return TS_STATUS_ERROR;
    }

    atomic_store(&mngr->fw_progress, mngr->fw_progress_max);

    return TS_STATUS_OK;
}

int32_t ts_fw_manager_get_progress(ts_fw_manager_t* mngr, uint32_t* progress)
{
    uint32_t current_progress = 0;
    
    // Get Progress of Flash Write
    if(mngr == NULL || progress == NULL)
    {
        LOG_ERROR("Invalid manager handle");
        return TS_STATUS_ERROR;
    }
    
    if(mngr->fw_progress_max == 0)
    {
        LOG_ERROR("Firmware Update not in progress");
    }
    else if (current_progress < mngr->fw_progress_max)
    {
        current_progress = (mngr->fw_progress * 100) / mngr->fw_progress_max;
    }
    else
    {
        current_progress = 100;
    }
    
    *progress = current_progress;
    return TS_STATUS_OK;
}

int32_t ts_fw_manager_user_cal_get(ts_fw_manager_t* mngr, const char* file_stream, uint32_t max_len)
{
    // Read File from SPI Flash

    return TS_STATUS_OK;
}

int32_t ts_fw_manager_user_cal_update(ts_fw_manager_t* mngr, const char* file_stream, uint32_t len)
{
    // Verify File Good

    // Erase User Flash Partition

    // Program New User Data

    return TS_STATUS_OK;
}

int32_t ts_fw_manager_factory_cal_get(ts_fw_manager_t* mngr, const char* file_stream, uint32_t len)
{
    // Read File from SPI Flash

    return TS_STATUS_OK;
}

static void ts_fw_progress_update(void* ctx, uint32_t work_done, uint32_t work_total)
{
    ts_fw_manager_t* mngr = (ts_fw_manager_t*)ctx;
    if(mngr != NULL)
    {
        atomic_fetch_add(&mngr->fw_progress, work_done);
    }
}