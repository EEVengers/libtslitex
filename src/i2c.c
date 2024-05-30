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

#include "i2c.h"
#include "util.h"

#define I2C_PERIOD	(NANOSECOND / I2C_FREQ_HZ)
#define I2C_DELAY(n)    NS_DELAY(I2C_PERIOD * (n) / 4)

#define I2C_SCL     (0x01)
#define I2C_SDAOE   (0x02)
#define I2C_SDAOUT  (0x04)
#define I2C_SDAIN   (0x01)


static inline void i2c_oe_scl_sda(i2c_t device, bool oe, bool scl, bool sda)
{
    litepcie_writel(device.fd, CSR_I2C_W_ADDR, ((oe & 1) << CSR_I2C_W_OE_OFFSET) |
        ((scl & 1) << CSR_I2C_W_SCL_OFFSET) |
        ((sda & 1) << CSR_I2C_W_SDA_OFFSET));

}

// START condition: 1-to-0 transition of SDA when SCL is 1
static void i2c_start(i2c_t device)
{
    i2c_oe_scl_sda(device, 1, 1, 1);
    I2C_DELAY(1);
    i2c_oe_scl_sda(device, 1, 1, 0);
    I2C_DELAY(1);
    i2c_oe_scl_sda(device, 1, 0, 0);
    I2C_DELAY(1);
}

// STOP condition: 0-to-1 transition of SDA when SCL is 1
static void i2c_stop(i2c_t device)
{
    i2c_oe_scl_sda(device, 1, 0, 0);
    I2C_DELAY(1);
    i2c_oe_scl_sda(device, 1, 1, 0);
    I2C_DELAY(1);
    i2c_oe_scl_sda(device, 1, 1, 1);
    I2C_DELAY(1);
    i2c_oe_scl_sda(device, 0, 1, 1);
}

// Call when in the middle of SCL low, advances one clk period
static void i2c_transmit_bit(i2c_t device, bool bit)
{
    i2c_oe_scl_sda(device, 1, 0, bit);
    I2C_DELAY(1);
    i2c_oe_scl_sda(device, 1, 1, bit);
    I2C_DELAY(2);
    i2c_oe_scl_sda(device, 1, 0, bit);
    I2C_DELAY(1);
}

// Call when in the middle of SCL low, advances one clk period
static int i2c_receive_bit(i2c_t device)
{
    int value;
    i2c_oe_scl_sda(device, 0, 0, 0);
    I2C_DELAY(1);
    i2c_oe_scl_sda(device, 0, 1, 0);
    I2C_DELAY(1);
    // read in the middle of SCL high
    value = litepcie_readl(device.fd, CSR_I2C_R_ADDR) & 1;
    I2C_DELAY(1);
    i2c_oe_scl_sda(device, 0, 0, 0);
    I2C_DELAY(1);
    return value;
}

// Send data byte and return 1 if slave sends ACK
static bool i2c_transmit_byte(i2c_t device, uint8_t data)
{
    int i;
    int ack;

    // SCL should have already been low for 1/4 cycle
    // Keep SDA low to avoid short spikes from the pull-ups
    i2c_oe_scl_sda(device, 1, 0, 0);
    for (i = 0; i < 8; ++i) {
        // MSB first
        i2c_transmit_bit(device, (data & (1 << 7)) != 0);
        data <<= 1;
    }
    i2c_oe_scl_sda(device, 0, 0, 0); // release line
    ack = i2c_receive_bit(device);

    // 0 from slave means ack
    return ack == 0;
}

// Read data byte and send ACK if ack=1
static unsigned char i2c_receive_byte(i2c_t device, bool ack)
{
    int i;
    uint8_t data = 0;

    for (i = 0; i < 8; ++i) {
        data <<= 1;
        data |= i2c_receive_bit(device);
    }
    i2c_transmit_bit(device, !ack);
    i2c_oe_scl_sda(device, 0, 0, 0); // release line

    return data;
}

// Reset line state
void i2c_reset(i2c_t dev)
{
    int i;
    i2c_oe_scl_sda(dev, 1, 1, 1);
    I2C_DELAY(8);
    for (i = 0; i < 9; ++i) {
        i2c_oe_scl_sda(dev, 1, 0, 1);
        I2C_DELAY(2);
        i2c_oe_scl_sda(dev, 1, 1, 1);
        I2C_DELAY(2);
    }
    i2c_oe_scl_sda(dev, 0, 0, 1);
    I2C_DELAY(1);
    i2c_stop(dev);
    i2c_oe_scl_sda(dev, 0, 1, 1);
    I2C_DELAY(8);
}

/*
    * Read slave memory over I2C starting at given address
    *
    * First writes the memory starting address, then reads the data:
    *   START WR(slaveaddr) WR(addr) STOP START WR(slaveaddr) RD(data) RD(data) ... STOP
    * Some chips require that after transmiting the address, there will be no STOP in between:
    *   START WR(slaveaddr) WR(addr) START WR(slaveaddr) RD(data) RD(data) ... STOP
    */
bool i2c_read(i2c_t device, uint32_t addr, uint8_t* data, uint32_t len, bool send_stop, uint32_t addr_size)
{
    int32_t i, j;

    if ((addr_size < 1) || (addr_size > 4)) {
        return false;
    }

    i2c_start(device);

    if (!i2c_transmit_byte(device, I2C_ADDR_WR(device.devAddr))) {
        i2c_stop(device);
        LOG_ERROR("I2C NACK writing slave RD addr 0x%02X", device.devAddr);
        return false;
    }
    for (j = addr_size - 1; j >= 0; j--) {
        if (!i2c_transmit_byte(device, (uint8_t)(0xff & (addr >> (8 * j))))) {
            i2c_stop(device);
            LOG_ERROR("I2C NACK writing RD register 0x%X", addr);
            return false;
        }
    }

    if (send_stop) {
        i2c_stop(device);
    }
    i2c_start(device);

    if (!i2c_transmit_byte(device, I2C_ADDR_RD(device.devAddr))) {
        i2c_stop(device);
        LOG_ERROR("I2C NACK reading slave RD addr 0x%02X", device.devAddr);
        return false;
    }
    for (i = 0; (uint32_t)i < len; ++i) {
        data[i] = i2c_receive_byte(device, ((uint32_t)i != (len - 1)));
    }

    i2c_stop(device);

    return true;
}

/*
    * Write slave memory over I2C starting at given address
    *
    * First writes the memory starting address, then writes the data:
    *   START WR(slaveaddr) WR(addr) WR(data) WR(data) ... STOP
    */
bool i2c_write(i2c_t device, uint32_t addr, const uint8_t* data, uint32_t len, uint32_t addr_size)
{
    int32_t i, j;

    if ((addr_size < 1) || (addr_size > 4)) {
        return false;
    }

    i2c_start(device);

    if (!i2c_transmit_byte(device, I2C_ADDR_WR(device.devAddr))) {
        i2c_stop(device);
        LOG_ERROR("I2C NACK writing slave WR addr 0x%02X", device.devAddr);
        return false;
    }
    for (j = addr_size - 1; j >= 0; j--) {
        if (!i2c_transmit_byte(device, (unsigned char)(0xff & (addr >> (8 * j))))) {
            i2c_stop(device);
            LOG_ERROR("I2C NACK writing WR register 0x%X", addr);
            return false;
        }
    }
    for (i = 0; (uint32_t)i < len; ++i) {
        if (!i2c_transmit_byte(device, data[i])) {
            i2c_stop(device);
            LOG_ERROR("I2C NACK writing %d byte 0x%X", i, data[i]);
            return false;
        }
    }

    i2c_stop(device);

    return true;
}

/*
    * Poll I2C slave at given address, return true if it sends an ACK back
    */
bool i2c_poll(i2c_t device)
{
    bool result;

    i2c_start(device);
    result = i2c_transmit_byte(device, I2C_ADDR_WR(device.devAddr));
    if (!result) {
        i2c_start(device);
        result |= i2c_transmit_byte(device, I2C_ADDR_RD(device.devAddr));
        if (result)
            i2c_receive_byte(device, false);
    }
    i2c_stop(device);

    return result;
}
