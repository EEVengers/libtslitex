/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * A simple GPIO driver for controlling banks of pins
 * in the Thunderscope LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _GPIO_H_
#define _GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "liblitepcie.h"

typedef struct gpio_s
{
    file_t fd;
    uint32_t reg;
    uint32_t bit_mask;
} gpio_t;

uint32_t gpio_get(gpio_t gpio);
void gpio_set(gpio_t gpio);
void gpio_clear(gpio_t gpio);
void gpio_group_set(gpio_t gpio, uint32_t set);

#ifdef __cplusplus
}
#endif

#endif