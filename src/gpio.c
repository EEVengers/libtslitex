/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * A simple GPIO driver for controlling banks of pins
 * in the Thunderscope LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#include "gpio.h"
#include "liblitepcie.h"

uint32_t gpio_get(gpio_t gpio)
{
    uint32_t result = litepcie_readl(gpio.fd, gpio.reg);
    return (result & gpio.bit_mask);
}

void gpio_set(gpio_t gpio)
{
    uint32_t value = litepcie_readl(gpio.fd, gpio.reg);
    litepcie_writel(gpio.fd, gpio.reg, (value | gpio.bit_mask));
}

void gpio_clear(gpio_t gpio)
{
    uint32_t value = litepcie_readl(gpio.fd, gpio.reg);
    litepcie_writel(gpio.fd, gpio.reg, (value & ~gpio.bit_mask));
}

void gpio_group_set(gpio_t gpio, uint32_t set)
{
    uint32_t value = litepcie_readl(gpio.fd, gpio.reg);
    litepcie_writel(gpio.fd, gpio.reg, (value | (set & gpio.bit_mask)));
}
