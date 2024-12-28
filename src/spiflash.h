/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * A SPI Flash driver for the LiteX LiteSPI Core in the 
 * Thunderscope LiteX design
 *
 * Copyright (c) 2024 / Nate Meyer / nate.devel@gmail.com
 *
 */

#ifndef _SPIFLASH_H_
#define _SPIFLASH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ts_common.h"
#include "liblitepcie.h"

typedef struct spiflash_ops_s {
    uint8_t read;
    uint8_t program;
    uint8_t erase_sector;
    uint8_t cmd_addr_len;
} spiflash_ops_t;

typedef struct spiflash_dev_s {
    file_t fd;
    uint8_t mfg_code;
    uint16_t part_id;
    spiflash_ops_t ops;
} spiflash_dev_t;

/**
 * @brief Initialize the SPI Flash module
 * 
 * @param fd File handle for the driver
 * @param dev Pointer to the spiflash device instance
 * 
 * @return TS_STATUS_OK on success.
 */
int32_t spiflash_init(file_t fd, spiflash_dev_t* dev);

/**
 * @brief Read a data buffer from the SPIFlash
 * 
 * @param dev Pointer to the spiflash instance
 * @param addr Location in SPI Flash to read
 * @param pData Pointer to the buffer to store read data in
 * @param len Length of the read
 * 
 * @return Length of the data read or a negative error code.
 */
int32_t spiflash_read(spiflash_dev_t* dev, uint32_t addr, uint8_t* pData, uint32_t len);

/**
 * @brief Read a data buffer from the SPIFlash OTP Region
 * 
 * @param dev Pointer to the spiflash instance
 * @param addr Location in SPI Flash OTP Region to read
 * @param pData Pointer to the buffer to store read data in
 * @param len Length of the read
 * 
 * @return Length of the data read or a negative error code.
 */
int32_t spiflash_OTP_read(spiflash_dev_t* dev, uint32_t addr, uint8_t* pData, uint32_t len);

/**
 * @brief Erase a 64KB sector in SPI Flash.
 *
 * @param Pointer to the spiflash instance
 * @param addr Address to erase.  Should point to the start of a sector.
 * @param len Length of the area to be erased.  This will erase 64K sectors up to and including the
 *              sector containing the end of this range.
 * 
 * @return TS_STATUS_OK on success.
 */
int32_t spiflash_erase(spiflash_dev_t* dev, uint32_t addr, uint32_t len);

/**
 * @brief Write a data buffer to SPI Flash.  All data written will be verified against the input buffer.
 * 
 * @param dev Pointer to the spiflash instance
 * @param addr Location in SPI Flash to write
 * @param pData Pointer to the buffer of data to be written
 * @param len Length of the write buffer
 * 
 * @return Length of the data written or a negative error code.
 */
int32_t spiflash_write(spiflash_dev_t* dev, uint32_t addr, const uint8_t *pData, uint32_t len);


#ifdef __cplusplus
}
#endif

#endif