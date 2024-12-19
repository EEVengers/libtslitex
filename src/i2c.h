/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * An I2C driver for the LiteI2C Core in
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

typedef struct i2c_s
{
    file_t fd;
    uint8_t devAddr;
} i2c_t;

typedef enum i2c_rate_e
{
    I2C_100KHz = 0,
    I2C_400KHz = 1,
    I2C_1MHz = 2,
} i2c_rate_t;

/**
 * @brief Initialize an I2C device struct
 */
int32_t i2c_init(i2c_t* device, file_t fd, uint8_t addr);

/**
 * @brief Set the I2C clock rate
 * 
 * @param device I2C device struct
 * @param rate Clock rate to set
 */
void i2c_rate_set(i2c_t device, i2c_rate_t rate);

/**
 * @brief Read from an I2C device
 * 
 * @param device I2C device struct
 * @param addr Device Command/Register to write for preparing a read
 * @param data Pointer to a byte-array to store data read
 * @param len Length of the data array
 * @param addr_size Number of bytes in the addr parameter
 * 
 * @return True if the read completed successfully.
 */
bool i2c_read(i2c_t device, uint32_t addr, uint8_t* data, uint32_t len, uint32_t addr_size);

/**
 * @brief Write to an I2C device
 * 
 * @param device I2C device struct
 * @param addr Device Command/Register to write
 * @param data Pointer to a byte-array of data to be written
 * @param len Length of the data array
 * @param addr_size Number of bytes in the addr parameter
 *
 * @return True if the write was ACK'd by the device
 */
bool i2c_write(i2c_t device, uint32_t addr, const uint8_t* data, uint32_t len, uint32_t addr_size);

/**
 * @brief Release the I2C lines
 * 
 * @param device I2C device struct
 * 
 */
void i2c_reset(i2c_t device);
 
/**
 * @brief Poll I2C slave at given address, return true if it sends an ACK back
 * 
 * @param device I2C device struct
 * 
 * @return True if device is present
 */
bool i2c_poll(i2c_t device);


#ifdef __cplusplus
}
#endif

#endif