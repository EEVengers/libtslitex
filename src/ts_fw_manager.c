/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Provide methods to fetch and store content in flash
 *
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 */

#include <string.h>

#if defined(_WIN32)
#include <winsock2.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <arpa/inet.h>
#endif

#include <zlib.h>

#include "ts_fw_manager.h"
#include "platform.h"
#include "spiflash.h"
#include "liblitepcie.h"
#include "util.h"

#define FLASH_PAGE_LEN   (SPI_FLASH_ERASE_SIZE)

// From AMD UG470
#define IDCODE_7A200T   (0x03636093)
#define IDCODE_7A100T   (0x03631093)
#define IDCODE_7A50T    (0x0362C093)
#define IDCODE_7A35T    (0x0362D093)
#define IDCODE_MASK     (0x0FFFFFFF)

#define ICAP_REG_IDCODE (0x0C)

#define INVALID_TAG     (0xFFFFFFFF)

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
    const uint8_t* position = header;
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

int32_t ts_fw_manager_user_data_read(ts_fw_manager_t* mngr, char* buffer, uint32_t offset, uint32_t max_len)
{
    if(mngr == NULL)
    {
        LOG_ERROR("Invalid manager handle");
        return TS_STATUS_ERROR;
    }

    uint32_t max_offset = mngr->partition_table->user_config_end - mngr->partition_table->user_config_start;
    uint32_t readLen = max_len;
    atomic_store(&mngr->fw_progress, 0);
    mngr->fw_progress_max = max_len;
    
    if(buffer == NULL)
    {
        LOG_ERROR("User Data Read Invalid Buffer Address");
        return TS_STATUS_ERROR;
    }
    if (offset > max_offset)
    {
        LOG_ERROR("User Data Read Invalid Offset (%d is beyond range of %d)", offset, max_offset);
        return TS_STATUS_ERROR;
    }
    if(max_len == 0)
    {
        LOG_ERROR("User Data Read Invalid Len (%d)", max_len);
        return TS_STATUS_ERROR;
    }

    if((offset + max_len) > (max_offset))
    {
        readLen = max_offset - offset;
    }
    
    // Read File from SPI Flash
    if(readLen != spiflash_read(&mngr->flash_dev, (mngr->partition_table->user_config_start + offset), buffer, readLen))
    {
        LOG_ERROR("Failed to read user data partition (%d)", readLen);
        return TS_STATUS_ERROR;
    }

    return readLen;
}

int32_t ts_fw_manager_user_data_write(ts_fw_manager_t* mngr, const char* buffer, uint32_t offset, uint32_t len)
{
    if(mngr == NULL)
    {
        LOG_ERROR("Invalid manager handle");
        return TS_STATUS_ERROR;
    }

    int32_t status;
    uint8_t start_buffer[FLASH_PAGE_LEN];
    uint8_t end_buffer[FLASH_PAGE_LEN];
    uint32_t max_offset = mngr->partition_table->user_config_end - mngr->partition_table->user_config_start;
    uint32_t start_addr = mngr->partition_table->user_config_start + (offset & ~(FLASH_PAGE_LEN - 1));
    uint32_t op_len = ((len + (offset % FLASH_PAGE_LEN) //Add prepend page len
                        + (FLASH_PAGE_LEN - 1)) / FLASH_PAGE_LEN) //Round up to total number of pages
                        * FLASH_PAGE_LEN; // Multiply to get number of bytes

    
    atomic_store(&mngr->fw_progress, 0);
    mngr->fw_progress_max = op_len*2;


    // Verify Parameters Good
    if(buffer == NULL)
    {
        LOG_ERROR("User Data Write Invalid Buffer Address");
        return TS_STATUS_ERROR;
    }
    if (offset >= max_offset)
    {
        LOG_ERROR("User Data Write Invalid Offset (%d is beyond range of %d)", offset, max_offset);
        return TS_STATUS_ERROR;
    }
    if(len == 0)
    {
        LOG_ERROR("User Data Write Invalid Len (%d)", len);
        return TS_STATUS_ERROR;
    }
    else if((len + offset) > max_offset)
    {
        LOG_ERROR("User Data Write Invalid Offset 0x%06X Len %d exceeds end of User Data Region (0x%06X)", offset, len, max_offset);
        return TS_STATUS_ERROR;
    }

    //Save initial page if offset not 4k-aligned
    if((offset % FLASH_PAGE_LEN) != 0)
    {
        LOG_DEBUG("Read %d bytes at address %08X", (FLASH_PAGE_LEN), (start_addr + op_len - FLASH_PAGE_LEN));
        status = spiflash_read(&mngr->flash_dev, start_addr, start_buffer, FLASH_PAGE_LEN);
        if(status != FLASH_PAGE_LEN)
        {
            LOG_ERROR("Failed to read SPI Flash page 0x%06X (%d)", start_addr, status);
            return status;
        }
    }

    //Save final page
    LOG_DEBUG("Read %d bytes at address %08X", (FLASH_PAGE_LEN), (start_addr + op_len - FLASH_PAGE_LEN));
    status = spiflash_read(&mngr->flash_dev, (start_addr + op_len - FLASH_PAGE_LEN), end_buffer, FLASH_PAGE_LEN);
    if(status != FLASH_PAGE_LEN)
    {
        LOG_ERROR("Failed to read SPI Flash page 0x%06X (%d)", (start_addr + op_len - FLASH_PAGE_LEN), status);
        return status;
    }
    
    // Erase User Flash Partition
    LOG_DEBUG("Erase %d bytes at address %08X", (op_len), (start_addr));
    if(TS_STATUS_OK != spiflash_erase(&mngr->flash_dev, start_addr, op_len))
    {
        LOG_ERROR("Failed to erase user data partition (0x%06X 0x%06X)", start_addr, op_len);
        return TS_STATUS_ERROR;
    }

    atomic_store(&mngr->fw_progress, op_len);

    // Program New User Data
    if((offset % FLASH_PAGE_LEN) != 0)
    {
        //Copy beginning of user data over saved first page
        uint32_t start_page_len = ((offset % FLASH_PAGE_LEN) + len) > FLASH_PAGE_LEN ? 
                                    (FLASH_PAGE_LEN - (offset % FLASH_PAGE_LEN)) :
                                    len;
        memcpy(&start_buffer[offset % FLASH_PAGE_LEN], buffer, start_page_len);
        LOG_DEBUG("Copy %d bytes at offset %08X to start buffer", start_page_len, (offset % FLASH_PAGE_LEN));
        
        LOG_DEBUG("Write %d bytes at address %08X", FLASH_PAGE_LEN, (start_addr));
        if(FLASH_PAGE_LEN != spiflash_write(&mngr->flash_dev, start_addr, start_buffer, FLASH_PAGE_LEN))
        {
            LOG_ERROR("Failed to erase user bitstream partition");
            return TS_STATUS_ERROR;
        }
        
        start_addr += FLASH_PAGE_LEN;
        op_len -= FLASH_PAGE_LEN;
    }

    if(op_len > FLASH_PAGE_LEN)
    {
        if(start_addr + op_len >
             mngr->partition_table->user_config_end)
        {
            LOG_ERROR("Failed to write %d bytes at 0x%06X", op_len, start_addr);
            return TS_STATUS_ERROR;
        }

        LOG_DEBUG("Write %d bytes at address %08X", (op_len - FLASH_PAGE_LEN), (start_addr));
        if(TS_STATUS_ERROR == spiflash_write(&mngr->flash_dev, (start_addr), (uint8_t*)buffer,
                                                (op_len - FLASH_PAGE_LEN)))
        {
            LOG_ERROR("Failed to write user bitstream partition");
            return TS_STATUS_ERROR;
        }
        start_addr += op_len - FLASH_PAGE_LEN;
        op_len = FLASH_PAGE_LEN;
    }

    if(op_len != 0)
    {
        //Copy end of user data over saved final page
        uint32_t remainder = (len+offset) % FLASH_PAGE_LEN;
        if(remainder == 0)
        {
            remainder = FLASH_PAGE_LEN;
        }
        memcpy(end_buffer, &buffer[len - remainder], remainder);
        LOG_DEBUG("Copy %d bytes at src offset %08X to final buffer", (remainder), (len - remainder));

        if(start_addr + FLASH_PAGE_LEN > mngr->partition_table->user_config_end)
        {
            LOG_ERROR("Failed to write final page at 0x%06X", start_addr + FLASH_PAGE_LEN);
            return TS_STATUS_ERROR;
        }

        LOG_DEBUG("Write %d bytes at address %08X", (FLASH_PAGE_LEN), (start_addr));
        if(FLASH_PAGE_LEN != spiflash_write(&mngr->flash_dev, start_addr, end_buffer, FLASH_PAGE_LEN))
        {
            LOG_ERROR("Failed to erase user bitstream partition");
            return TS_STATUS_ERROR;
        }
    }

    atomic_store(&mngr->fw_progress, mngr->fw_progress_max);

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

int32_t ts_fw_manager_factory_data_erase(ts_fw_manager_t* mngr, uint64_t dna)
{
    uint64_t dna_actual= (uint64_t)litepcie_readl(mngr->flash_dev.fd, CSR_DNA_ID_ADDR) << 32;
    dna_actual |= litepcie_readl(mngr->flash_dev.fd, CSR_DNA_ID_ADDR + 4);

    if(dna == dna_actual)
    {
        return spiflash_erase(&mngr->flash_dev, mngr->partition_table->factory_config_start,
            (mngr->partition_table->factory_config_end - mngr->partition_table->factory_config_start));
    }
    return TS_STATUS_ERROR;
}

int32_t ts_fw_manager_factory_data_append(ts_fw_manager_t* mngr, uint32_t tag, uint32_t length, const uint8_t *content)
{
    int32_t retVal =  TS_STATUS_ERROR;
    uint32_t offset = 0;
    uint32_t current_tag = INVALID_TAG;
    uint32_t next_len = 0;
    uint32_t tlv_crc = crc32(0, Z_NULL, 0);

    // Append the new TLV
    while (offset < mngr->partition_table->factory_config_end)
    {
        retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start+offset),
                                (uint8_t*)&current_tag, sizeof(current_tag));
        if(retVal < 0)
        {
            LOG_ERROR("Failed to read tag at offset 0x%x", offset);
            break;
        }

        if(tag == htonl(current_tag))
        {
            LOG_ERROR("ERROR: Duplicate TAG %08X", tag);
            retVal = TS_STATUS_ERROR;
            break;
        }

        // Check if empty
        if(current_tag == INVALID_TAG)
        {
            //Test for length
            if((mngr->partition_table->factory_bitstream_start + offset + sizeof(uint32_t)*3 + length) > 
                mngr->partition_table->factory_config_end)
            {
                LOG_ERROR("Not enough space in Factory Partition to store object of %dB",length);
                retVal = TS_STATUS_ERROR;
                break;
            }

            //Write Tag, and Length
            uint64_t tl = ((uint64_t)htonl(length) << 32) + htonl(tag);
            retVal = spiflash_write(&mngr->flash_dev, (mngr->partition_table->factory_config_start+offset), (uint8_t*)&tl, sizeof(uint64_t));
            if(retVal < 0)
            {
                LOG_ERROR("Failed to write tag %08x at offset 0x%x", tag, offset);
                break;
            }

            offset += sizeof(uint64_t);

            //Write Value
            retVal = spiflash_write(&mngr->flash_dev, (mngr->partition_table->factory_config_start + offset), content, length);
            if(retVal < 0)
            {
                LOG_ERROR("Failed to write value for tag %08x at offset 0x%x", tag, offset);
                break;
            }
            offset += length;

            //Write CRC
            tlv_crc = crc32(tlv_crc, content, length);
            LOG_DEBUG("Completing TLV with CRC %08X", tlv_crc);
            tlv_crc = htonl(tlv_crc);
            retVal = spiflash_write(&mngr->flash_dev, (mngr->partition_table->factory_config_start + offset), (uint8_t*)&tlv_crc, sizeof(uint32_t));
            if(retVal < 0)
            {
                LOG_ERROR("Failed to write CRC32 for tag %08X at offset 0x%x", tag, offset);
                break;
            }
            retVal = TS_STATUS_OK;
            break;
        }
        
        //Read Len and increment for next tag
        offset += sizeof(uint32_t);
        retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start + offset), (uint8_t*)&next_len, sizeof(uint32_t));
        if(retVal < 0)
        {
            LOG_ERROR("Failed to read tag at offset 0x%x", offset);
            break;
        }

        // Increment size of Length, Value, and CRC
        offset += sizeof(uint32_t)*2 + ntohl(next_len);
    }
    return retVal;
}

int32_t ts_fw_manager_factory_data_verify(ts_fw_manager_t* mngr)
{
    int32_t retVal =  TS_STATUS_OK;
    uint32_t offset = 0;
    uint32_t tag = INVALID_TAG;
    uint32_t next_len = 0;
    uint8_t val_buffer[4096];
    uint32_t stored_crc;
    uint32_t new_crc = crc32(0, Z_NULL, 0);
    uint32_t element_count = 0;

    // Read and Verify each TLV CRC32
    while (offset < mngr->partition_table->factory_config_end)
    {
        //Read the TAG
        retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start+offset),
                                (uint8_t*)&tag, sizeof(tag));
        if(retVal < 0)
        {
            LOG_ERROR("Failed to read tag at offset 0x%x", offset);
            break;
        }

        offset += sizeof(tag);
        
        // Check if empty, no more tags to verify
        if(tag == INVALID_TAG)
        {
            LOG_DEBUG("Factory data partition verified");
            retVal = TS_STATUS_OK;
            break;
        }

        //Read the Length
        retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start+offset),
                                (uint8_t*)&next_len, sizeof(uint32_t));
        if(retVal < 0)
        {
            LOG_ERROR("Failed to write tag %08x at offset 0x%x", tag, offset);
            break;
        }
        offset += sizeof(uint32_t);

        if((mngr->partition_table->factory_config_start + offset + ntohl(next_len) + sizeof(uint32_t)) >
            mngr->partition_table->factory_config_end)
        {
            LOG_ERROR("Tag %08X Exceeds Length (%d)", tag, next_len);
            retVal = TS_STATUS_ERROR;
            break;
        }
        
        //Read the CRC
        retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start+offset + ntohl(next_len)),
                                (uint8_t*)&stored_crc, sizeof(uint32_t));
        if(retVal < 0)
        {
            LOG_ERROR("Failed to write tag %08x at offset 0x%x", tag, offset);
            break;
        }

        //Read Value and Compute CRC
        while(next_len > 0)
        {
            uint32_t segment_len = next_len > sizeof(val_buffer) ? sizeof(val_buffer) : next_len;
            next_len -= segment_len;
            retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start + offset),
                                    val_buffer, segment_len);
            if(retVal < 0)
            {
                LOG_ERROR("Failed to write value for tag %08x at offset 0x%x", tag, offset);
                break;
            }
            offset += segment_len;
            
            //Calculate the CRC
            new_crc = crc32(new_crc, val_buffer, segment_len);
        }

        if(ntohl(stored_crc) != new_crc)
        {
            LOG_ERROR();
            retVal = TS_STATUS_ERROR;
            break;
        }

        // Increment size of CRC
        offset += sizeof(uint32_t);
        element_count++;
    }
    return retVal;
}

int32_t ts_fw_manager_factory_data_get_length(ts_fw_manager_t* mngr, uint32_t tag)
{
    int32_t retVal =  TS_STATUS_OK;
    uint32_t offset = 0;
    uint32_t read_tag = INVALID_TAG;
    uint32_t next_len = 0;

    // Search for a specific Tag and copy it to the provided Content buffer
    while (offset < mngr->partition_table->factory_config_end)
    {
        //Read the TAG
        retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start+offset), (uint8_t*)&read_tag, sizeof(read_tag));
        if(retVal < 0)
        {
            LOG_ERROR("Failed to read tag at offset 0x%x", offset);
            break;
        }

        offset += sizeof(read_tag);
        
        // Check if empty, no more tags to try
        if(read_tag == INVALID_TAG)
        {
            LOG_DEBUG("Tag 0x%08X not found in Factory data partition", tag);
            retVal = 0;
            break;
        }

        //Read the Length
        retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start+offset), (uint8_t*)&next_len, sizeof(uint32_t));
        if(retVal < 0)
        {
            LOG_ERROR("Failed to write tag %08x at offset 0x%x", tag, offset);
            break;
        }
        next_len = ntohl(next_len);
        offset += sizeof(uint32_t);

        if((mngr->partition_table->factory_config_start + offset + next_len + sizeof(uint32_t)) >
            mngr->partition_table->factory_config_end)
        {
            LOG_ERROR("Tag %08X Bad Length (%d)", tag, next_len);
            retVal = TS_STATUS_ERROR;
            break;
        }

        //Tag matches, return length
        if(ntohl(read_tag) == tag)
        {
            retVal = next_len;
            break;
        }
        else
        {
            offset += (sizeof(uint32_t) + next_len);
        }
    }
    return retVal;
}

int32_t ts_fw_manager_factory_data_retreive(ts_fw_manager_t* mngr, uint32_t tag, uint8_t* content, uint32_t max_len)
{
    int32_t retVal =  TS_STATUS_OK;
    uint32_t offset = 0;
    uint32_t check_offset = 0;
    uint32_t read_tag = INVALID_TAG;
    uint32_t next_len = 0;
    uint32_t val_len = 0;
    uint8_t val_buffer[4096];
    uint32_t stored_crc;
    uint32_t new_crc = crc32(0, Z_NULL, 0);

    // Search for a specific Tag and copy it to the provided Content buffer
    while (offset < mngr->partition_table->factory_config_end)
    {
        //Read the TAG
        retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start+offset), (uint8_t*)&read_tag, sizeof(read_tag));
        if(retVal < 0)
        {
            LOG_ERROR("Failed to read tag at offset 0x%x", offset);
            break;
        }

        offset += sizeof(read_tag);
        
        // Check if empty, no more tags to try
        if(tag == INVALID_TAG)
        {
            LOG_DEBUG("Tag 0x%08X not found in Factory data partition", tag);
            retVal = TS_STATUS_OK;
            break;
        }

        //Read the Length
        retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start+offset), (uint8_t*)&next_len, sizeof(uint32_t));
        if(retVal < 0)
        {
            LOG_ERROR("Failed to read tag %08x length at offset 0x%x", ntohl(read_tag), offset);
            break;
        }
        next_len = ntohl(next_len);
        offset += sizeof(uint32_t);

        if((mngr->partition_table->factory_config_start + offset + next_len + sizeof(uint32_t)) >
            mngr->partition_table->factory_config_end)
        {
            LOG_ERROR("Tag %08X Bad Length (%d)", ntohl(read_tag), next_len);
            retVal = TS_STATUS_ERROR;
            break;
        }

        //Find next if tag doesn't match
        if(ntohl(read_tag) != tag)
        {
            offset +=  (sizeof(uint32_t) + next_len);
            continue;
        }
        
        //Read the CRC
        retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start+offset + next_len), (uint8_t*)&stored_crc, sizeof(uint32_t));
        if(retVal < 0)
        {
            LOG_ERROR("Failed to write tag %08x at offset 0x%x", tag, offset);
            break;
        }

        //Read Value and Compute CRC
        val_len = next_len;
        check_offset = offset;
        while(next_len > 0)
        {
            uint32_t segment_len = next_len > sizeof(val_buffer) ? sizeof(val_buffer) : next_len;
            next_len -= segment_len;
            retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start + check_offset), val_buffer, segment_len);
            if(retVal < 0)
            {
                LOG_ERROR("Failed to read value for tag %08x at offset 0x%x : %d", tag, offset, retVal);
                break;
            }
            check_offset += segment_len;
            
            //Calculate the CRC
            new_crc = crc32(new_crc, val_buffer, segment_len);
        }

        if(ntohl(stored_crc) != new_crc)
        {
            LOG_ERROR("Failed to read Tag %08X, bad CRC (expected %08X, calculated %08X)", tag,
                        ntohl(stored_crc), new_crc);
            retVal = TS_STATUS_ERROR;
            break;
        }
        
        //CRC Passes, read to user buffer
        new_crc = crc32(0, Z_NULL, 0);
        retVal = spiflash_read(&mngr->flash_dev, (mngr->partition_table->factory_config_start + offset), content, val_len);
        if(retVal < 0)
        {
            LOG_ERROR("Failed to read value for tag %08x at offset 0x%x : %d", tag, offset, retVal);
            break;
        }
        
        //Calculate the CRC one last time
        new_crc = crc32(new_crc, content, val_len);

        if(ntohl(stored_crc) == new_crc)
        {
            retVal = val_len;
        }
        else
        {
            LOG_ERROR("Failed CRC check for tag %08X (expected 0x%08X, calculated 0x%08X)",
                        tag, ntohl(stored_crc), new_crc);
            retVal = TS_STATUS_ERROR;
        }
        break;
    }
    return retVal;
}