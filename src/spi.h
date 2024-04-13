/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * A SPI driver for the LiteX SoC SPI Core in the 
 * Thunderscope LiteX design
 *
 * Copyright (c) 2020-2021 Florent Kermarrec <florent@enjoy-digital.fr>
 * Copyright (c) 2022 Franck Jullien <franck.jullien@collshade.fr>
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 * Copyright (c) 2024 John Simons <jammsimons@gmail.com>
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 *
 */
#ifndef _SPI_H_
#define _SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "liblitepcie.h"


typedef struct spi_bus_s
{
    file_t fd;
    uint32_t spi_base;
    uint32_t num_cs;
    uint32_t cs_mask;
}spi_bus_t;

typedef struct spi_dev_s
{
    spi_bus_t* bus;
    uint32_t cs;
}spi_dev_t;

int32_t spi_bus_init(spi_bus_t* bus, file_t fd, uint32_t spi_base, uint32_t num_cs);
int32_t spi_dev_init(spi_dev_t* dev, spi_bus_t* bus, uint32_t cs_index);

void spi_write(spi_dev_t dev, uint8_t reg, uint8_t* data, uint8_t len);
bool spi_is_busy(spi_dev_t dev);
int32_t spi_busy_wait(spi_dev_t dev);

#ifdef __cplusplus
}
#endif
#endif