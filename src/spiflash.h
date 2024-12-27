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
    spiflash_ops_t ops;
} spiflash_dev_t;

void spiflash_init(file_t fd);

int32_t spiflash_read(file_t fd, uint32_t addr, uint8_t* pData, uint32_t len);

bool spiflash_idle(file_t fd);

int32_t spiflash_erase(file_t fd, uint32_t addr, uint32_t len);

int32_t spiflash_write(file_t fd, uint32_t addr, uint8_t *pData, uint32_t len);


#ifdef __cplusplus
}
#endif
#endif