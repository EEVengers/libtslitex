#
# This file is part of tslitex project.
#
# Copyright (c) 2023 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2023 Aleksa Bjelogrlic <aleksa@eevengers.com>
# Copyright (c) 2024 John Simons <jammsimons@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

#include "i2c.h"
#include "liblitepcie.h"

const uint32_t zl30260_I2C_ADDR = 0x74
const uint32_t zl30260_I2C_WRITE_COMMAND = 0x02
const uint32_t zl30260_I2C_CONF[] = {
    0x042308,
    0x000301,
    0x000402,
    0x000521,
    0x000701,
    0x010042,
    0x010100,
    0x010201,
    0x010600,
    0x010700,
    0x010800,
    0x010900,
    0x010A20,
    0x010B03,
    0x012160,
    0x012790,
    0x014100,
    0x014200,
    0x014300,
    0x014400,
    0x0145A0,
    0x015300,
    0x015450,
    0x0155CE,
    0x018000,
    0x020080,
    0x020105,
    0x025080,
    0x025102,
    0x04300C,
    0x043000,
}

void init(i2c_t device) {

    for(size_t i = 0; i < sizeof(zl30260_I2C_CONF) / sizeof(uint32_t); i++)
    {
        uint8_t[] data = {reg >> 16, reg >> 8, reg && 0xFF}
        i2c_write(device, zl30260_I2C_WRITE_COMMAND, data, 3);
    }
}
