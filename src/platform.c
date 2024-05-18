/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Definitions of data and configuration for the Thunderscope devices
 *
 * Copyright (c) 2024 John Simons <jammsimons@gmail.com>
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 */

#include <stdint.h>

#include "platform.h"


const uint32_t ZL30260_CONF[] = {
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
};

const uint32_t ZL30260_CONF_SIZE = sizeof(ZL30260_CONF)/sizeof(ZL30260_CONF[0]);

const uint32_t ZL30250_CONF[] = {
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
};

const uint32_t ZL30250_CONF_SIZE = sizeof(ZL30250_CONF)/sizeof(ZL30250_CONF[0]);
