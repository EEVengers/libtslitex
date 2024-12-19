/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * An I2C driver for the LiteX SoC Bitbang I2C Core in
 * the Thunderscope LiteX design
 * 
 * Copyright (c) 2020-2021 Florent Kermarrec <florent@enjoy-digital.fr>
 * Copyright (c) 2022 Franck Jullien <franck.jullien@collshade.fr>
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 * Copyright (c) 2024 John Simons <jammsimons@gmail.com>
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 */
#ifndef _I2C_H_
#define _I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "liblitepcie.h"

#ifndef I2C_FREQ_HZ
#define I2C_FREQ_HZ  400000
#endif

#define I2C_ADDR_WR(addr) ((addr) << 1)
#define I2C_ADDR_RD(addr) (((addr) << 1) | 1u)


typedef struct i2c_s
{
    file_t fd;
    uint8_t devAddr;
} i2c_t;


int32_t i2c_init(i2c_t* device, file_t fd, uint8_t addr);

/*
 * Read slave memory over I2C starting at given address
 *
 * First writes the memory starting address, then reads the data:
 *   START WR(slaveaddr) WR(addr) STOP START WR(slaveaddr) RD(data) RD(data) ... STOP
 * Some chips require that after transmiting the address, there will be no STOP in between:
 *   START WR(slaveaddr) WR(addr) START WR(slaveaddr) RD(data) RD(data) ... STOP
 */
bool i2c_read(i2c_t device, uint32_t addr, uint8_t* data, uint32_t len, bool send_stop, uint32_t addr_size);

/*
 * Write slave memory over I2C starting at given address
 *
 * First writes the memory starting address, then writes the data:
 *   START WR(slaveaddr) WR(addr) WR(data) WR(data) ... STOP
 */
bool i2c_write(i2c_t device, uint32_t addr, const uint8_t* data, uint32_t len, uint32_t addr_size);


void i2c_reset(i2c_t device);

bool i2c_poll(i2c_t device);


#ifdef __cplusplus
}
#endif

#endif