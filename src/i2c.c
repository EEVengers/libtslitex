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

#define I2C_PHY_SPEED_MODE_REG      (0x0000)
#define I2C_MASTER_ACTIVE_REG       (0x0004)
#define I2C_MASTER_SETTINGS_REG     (0x0008)
#define I2C_MASTER_ADDR_REG         (0x000C)
#define I2C_MASTER_RXTX_REG         (0x0010)
#define I2C_MASTER_STATUS_REG       (0x0014)

#define I2C_MASTER_SETTINGS_LEN_TX_OFFSET       (0)
#define I2C_MASTER_SETTINGS_LEN_RX_OFFSET       (8)
#define I2C_MASTER_SETTINGS_RECOVER_OFFSET      (16)
#define I2C_MASTER_STATUS_TX_READY_OFFSET       (0)
#define I2C_MASTER_STATUS_RX_READY_OFFSET       (1)
#define I2C_MASTER_STATUS_NACK_OFFSET           (8)
#define I2C_MASTER_STATUS_TX_UNFINISHED_OFFSET  (16)
#define I2C_MASTER_STATUS_RX_UNFINISHED_OFFSET  (17)

#define I2C_WAIT_TIMEOUT    (10000)

//I2C Transfer Settings
#define I2C_TX_LEN(x)   (((x) & 0x7) << I2C_MASTER_SETTINGS_LEN_TX_OFFSET)
#define I2C_RX_LEN(x)   (((x) & 0x7) << I2C_MASTER_SETTINGS_LEN_RX_OFFSET)
#define I2C_RECOVER(x)  (((x) & 0x1) << I2C_MASTER_SETTINGS_RECOVER_OFFSET)

//I2C Status
#define I2C_TX_READY(x)     (((x) >> I2C_MASTER_STATUS_TX_READY_OFFSET) & 0x1)
#define I2C_RX_READY(x)     (((x) >> I2C_MASTER_STATUS_RX_READY_OFFSET) & 0x1)
#define I2C_NACK(x)         (((x) >> I2C_MASTER_STATUS_NACK_OFFSET) & 0x1)
#define I2C_TX_PEND(x)      (((x) >> I2C_MASTER_STATUS_TX_UNFINISHED_OFFSET) & 0x1)
#define I2C_RX_PEND(x)      (((x) >> I2C_MASTER_STATUS_RX_UNFINISHED_OFFSET) & 0x1)

static inline void i2c_reg_write(i2c_t dev, uint32_t reg, uint32_t val)
{
    litepcie_writel(dev.fd, (dev.peripheral_baseaddr + reg), val);
}

static inline uint32_t i2c_reg_read(i2c_t dev, uint32_t reg)
{
    return litepcie_readl(dev.fd, (dev.peripheral_baseaddr + reg));
}

/**
 * @brief Enable the I2C core
 */
static inline void i2c_activate(i2c_t device, bool active)
{
    if(active)
    {
        i2c_reg_write(device, I2C_MASTER_ACTIVE_REG, 1);
    }
    else
    {
        i2c_reg_write(device, I2C_MASTER_ACTIVE_REG, 0);
    }
}

/**
 * @brief Set the I2C Device address for a transaction
 */
static inline void i2c_set_addr(i2c_t device)
{
    i2c_reg_write(device, I2C_MASTER_ADDR_REG, device.devAddr);
}

/**
 * @brief Poll for TX Ready status
 */
static inline void i2c_wait_tx_ready(i2c_t device)
{
    uint32_t timeout = I2C_WAIT_TIMEOUT;
    while(timeout && (I2C_TX_READY(i2c_reg_read(device, I2C_MASTER_STATUS_REG)) == 0))
    {
        NS_DELAY(1000);
        timeout--;
    }
}

/**
 * @brief Poll for RX Ready status
 */
static inline void i2c_wait_rx_ready(i2c_t device)
{
    uint32_t timeout = I2C_WAIT_TIMEOUT;
    while(timeout && (I2C_RX_READY(i2c_reg_read(device, I2C_MASTER_STATUS_REG)) == 0))
    {
        NS_DELAY(1000);
        timeout--;
    }
}


/**
 * @brief Send data up to 4 bytes
 * 
 * @return True if device sends ACK
 */
static bool i2c_tx(i2c_t device, uint32_t data, uint8_t len)
{
    bool ack;
    uint32_t i2cStatus;

    //Write Settings
    i2c_reg_write(device, I2C_MASTER_SETTINGS_REG, I2C_TX_LEN(len));

    //Wait for TX ready
    i2c_wait_tx_ready(device);

    //Write TX word
    i2c_reg_write(device, I2C_MASTER_RXTX_REG, data);

    //Wait for RX ready
    i2c_wait_rx_ready(device);

    //Get ACK
    i2cStatus = i2c_reg_read(device, I2C_MASTER_STATUS_REG);
    ack = I2C_NACK(i2cStatus) ? false : true;

    //Read the RX word
    i2cStatus = i2c_reg_read(device, I2C_MASTER_RXTX_REG);

    return ack;
}

/**
 * @brief Receive a word from the I2C core
 * 
 * @return True if device sends ACK
 */
static bool i2c_rx(i2c_t device, uint8_t len, uint32_t* pData)
{
    int i;
    uint32_t status = 0;
    bool ack;

    //Write Settings
    i2c_reg_write(device, I2C_MASTER_SETTINGS_REG, I2C_RX_LEN(len));

    //Wait for TX ready
    i2c_wait_tx_ready(device);

    //Write TX word
    i2c_reg_write(device, I2C_MASTER_RXTX_REG, 0);

    //Wait for RX Ready
    i2c_wait_rx_ready(device);

    //Get ACK
    status = i2c_reg_read(device, I2C_MASTER_STATUS_REG);
    ack = I2C_NACK(status) ? false : true;

    //Read Data word
    *pData = i2c_reg_read(device, I2C_MASTER_RXTX_REG);

    return ack;
}

int32_t i2c_init(i2c_t* device, file_t fd, uint8_t addr)
{
    if(device == NULL)
    {
        return -1;
    }

    device->fd = fd;
    device->devAddr = addr;

    return 0;
}

void i2c_rate_set(i2c_t device, i2c_rate_t rate)
{
    //Activate false
    i2c_activate(device, false);

    //Set Clock Rate
    i2c_reg_write(device, I2C_PHY_SPEED_MODE_REG, rate);
}

void i2c_reset(i2c_t device)
{
    i2c_activate(device, true);
    
    i2c_reg_write(device, I2C_MASTER_SETTINGS_REG, I2C_RECOVER(1));
    
    i2c_wait_tx_ready(device);

    i2c_reg_write(device, I2C_MASTER_RXTX_REG, 0);

    i2c_wait_rx_ready(device);

    i2c_reg_read(device, I2C_MASTER_RXTX_REG);

    i2c_activate(device, false);
}

bool i2c_read(i2c_t device, uint32_t addr, uint8_t* data, uint32_t len, uint32_t addr_size)
{
    int32_t i, j;

    if (addr_size > 4) {
        return false;
    }

    i2c_activate(device, true);

    //Set Address
    i2c_set_addr(device);

    //Write address
    if(addr_size > 0)
    {
        if(!i2c_tx(device, addr, addr_size))
        {
            LOG_ERROR("I2C NACK writing slave %02x address %x", device.devAddr, addr);
            i2c_activate(device, false);
            return false;
        }
    }

    // Read Data
    for(i=0; i < len; i += 4)
    {
        uint32_t data_word;
        uint8_t rx_size, rx_bytes;
        rx_bytes = rx_size = (len - i);
        if(rx_size > 5)
        {
            rx_size = 5;
            rx_bytes = 4;
        }

        if(!i2c_rx(device, rx_size, &data_word))
        {
            LOG_ERROR("I2C Read NACK for slave address 0x%02X", device.devAddr);
            i2c_activate(device, false);
            return false;
        }

        for(j=0; j < rx_bytes; j++)
        {
            data[i + j] = (data_word >> (8*(rx_bytes - 1 - j))) & 0xFF;
        }
    }

    i2c_activate(device, false);

    return true;
}

bool i2c_write(i2c_t device, uint32_t addr, const uint8_t* data, uint32_t len, uint32_t addr_size)
{
    uint32_t tx_word = 0;
    uint32_t tx_pend = 0;
    uint32_t data_idx = 0;
    uint32_t tx_size, tx_bytes=0;


    if (addr_size > 4) {
        return false;
    }

    i2c_activate(device, true);

    //Set Address
    i2c_set_addr(device);

    //Load address
    if(addr_size > 0)
    {
        tx_word = addr;
        tx_pend = addr_size;
    }

    // Write Data
    while(tx_bytes < (len + addr_size))
    {
        tx_size = (len + addr_size) - tx_bytes;
        if(tx_size > 4)
        {
            tx_size = 5;
        }

        while(tx_pend < tx_size && tx_pend < 4)
        {
            tx_word <<= 8;
            tx_word |= ((uint32_t)(data[data_idx++]) & 0xFF);
            tx_pend++;
        }

        if( !i2c_tx(device, tx_word, (uint8_t)tx_size) )
        {
            LOG_ERROR("I2C NACK writing data to slave %02X", device.devAddr);
            i2c_activate(device, false);
            return false;
        }

        tx_bytes += tx_pend;
    }

    i2c_activate(device, false);

    return true;
}

bool i2c_poll(i2c_t device)
{
    bool result;
    i2c_activate(device, true);

    //Set Address
    i2c_set_addr(device);

    //Empty Write
    result = i2c_tx(device, 0, 0);

    if (!result) {
        //Empty Read
        uint32_t data = 0;
        result = i2c_rx(device, 0, &data);
    }

    i2c_activate(device, false);

    return result;
}
