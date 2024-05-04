/*  *
 * This file is part of libtslitex.
 * Control the ZL30250 in the Thunderscope LiteX design
 *
 * Copyright (c) 2024 John Simons <jammsimons@gmail.com>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "i2c.h"
#include "liblitepcie.h"

const uint32_t ZL30250_I2C_ADDR = 0x6C
const uint32_t ZL30250_I2C_WRITE_COMMAND = 0x02
const uint32_t ZL30250_I2C_READ_COMMAND = 0x03
const uint32_t ZL30250_I2C_CONF = {
    0X000902,
    0X062108,
    0X063140,
    0X010006,
    0X010120,
    0X010202,
    0X010380,
    0X010A20,
    0X010B03,
    0X01140D,
    0X012006,
    0X0125C0,
    0X012660,
    0X01277F,
    0X012904,
    0X012AB3,
    0X012BC0,
    0X012C80,
    0X001C10,
    0X001D80,
    0X034003,
    0X020141,
    0X022135,
    0X022240,
    0X000C02,
    0X000B01,
    0X000D05, //Need 10ms beforehand, will be fine through jtag
}


void init(i2c_t device) {

    int totalSize = sizeof(zl30260_I2C_CONF);
    for(size_t i = 0; i < totalSize / sizeof(uint32_t); i++)
    {
        uint8_t[] data = {reg >> 16, reg >> 8, reg && 0xFF}
        i2c_write(device, zl30260_I2C_WRITE_COMMAND, data, 3);
    }
}
