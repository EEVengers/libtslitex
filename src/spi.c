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
#include "spi.h"
#include "liblitepcie.h"

#include <time.h>

#define SPI_CONTROL(base)       ((base) + 0x00)
#define SPI_STATUS(base)        ((base) + 0x04)
#define SPI_MOSI(base)          ((base) + 0x08)
#define SPI_MISO(base)          ((base) + 0x0C)
#define SPI_CS(base)            ((base) + 0x10)
#define SPI_LOOPBACK(base)      ((base) + 0x14)

#define SPI_CTRL_START      (1 << 0)
#define SPI_CTRL_LENGTH(x)  ((8*x) << 8)
#define SPI_STATUS_DONE     (1 << 0)
#define SPI_TIMEOUT_US      100000 /* in us */

#define SPI_CS_SEL(n)           (1 << (n))
#define SPI_CS_SEL_MASK(n)      ((1 << (n)) - 1)
#define SPI_CS_MODE_NORMAL      (0)
#define SPI_CS_MODE_MANUAL      (1 << 16)
#define SPI_CS_MODE_MASK        ((uint32_t)(1 << 16))

static bool spi_check_timeout(const struct timespec * start, uint32_t timeout);


int32_t spi_bus_init(spi_bus_t* bus, file_t fd, uint32_t spi_base, uint32_t num_cs)
{
    if(bus == NULL)
    {
        return -1;
    }

    bus->fd = fd;
    bus->spi_base = spi_base;

    bus->num_cs = num_cs;
    bus->cs_mask = SPI_CS_SEL_MASK(num_cs);

    return 0;
}

int32_t spi_dev_init(spi_dev_t* dev, spi_bus_t* bus, uint32_t cs_index)
{
    if(bus == NULL || dev == NULL)
    {
        return -1;
    }

    dev->bus = bus;
    dev->cs = cs_index;

    return 0;
}

void spi_write(spi_dev_t dev, uint8_t reg, uint8_t* data, uint8_t len) {

    uintptr_t addr = dev.bus->spi_base;

    // Set Chip Select.
    addr = SPI_CS(dev.bus->spi_base);
    litepcie_writel(dev.bus->fd, addr, SPI_CS_SEL(dev.cs) | SPI_CS_MODE_NORMAL);

    // Prepare MOSI data.
    uint32_t mosi_data = (reg << 16) + (data[0] << 8) + data[1]; 

    addr = SPI_MOSI(dev.bus->spi_base);
    litepcie_writel(dev.bus->fd, addr, mosi_data);

    // Start SPI Xfer.
    addr = SPI_CONTROL(dev.bus->spi_base);
    litepcie_writel(dev.bus->fd, addr, SPI_CTRL_LENGTH(len + 1) | SPI_CTRL_START);
}

bool spi_is_busy(spi_dev_t dev)
{
    return litepcie_readl(dev.bus->fd, SPI_STATUS(dev.bus->spi_base)) != SPI_STATUS_DONE;
}

// Wait SPI Xfer to be done.
int32_t spi_busy_wait(spi_dev_t dev)
{
    int32_t busyError = -1;
    struct timespec timeStart;
    timespec_get(&timeStart, TIME_UTC);

    do {
        if (litepcie_readl(dev.bus->fd, SPI_STATUS(dev.bus->spi_base)) == SPI_STATUS_DONE)
        {
            break;
        }
    } while (!spi_check_timeout(&timeStart, SPI_TIMEOUT_US));
    
    return busyError;
}


static bool spi_check_timeout(const struct timespec * start, uint32_t timeout)
{
    struct timespec timeNow;
    timespec_get(&timeNow, TIME_UTC);

    uint32_t delta_us = ((timeNow.tv_sec - start->tv_sec) * 1000000)
                            + ((timeNow.tv_nsec - start->tv_nsec) / 1000);
    return (delta_us > timeout);
}