/* SPDX-License-Identifier: BSD-2-Clause
*
* This file is part of libtslitex.
* A SPI Flash driver for the LiteX LiteSPI Core in the 
* Thunderscope LiteX design
*
* Copyright (c) 2020 / Antmicro / <www.antmicro.com>
* Copyright (c) 2024 / Nate Meyer / nate.devel@gmail.com
*
*/

#include "spiflash.h"
#include "util.h"

#include <csr.h>
#include <liblitepcie.h>
#include <inttypes.h>
#include <string.h>

#define SPI_FLASH_WINDOW_SIZE 0x10000

#define SPI_FLASH_CLK_DIV_DEFAULT   (3)
#define SPI_FLASH_CLK_DIV_MAX       (10)

#define SPI_FLASH_JEDEC_READ_ID_CMD             (0x9F)
#define SPI_FLASH_JEDEC_READ_STATUS_REG_1_CMD   (0x05)
#define SPI_FLASH_WRITE_ENABLE_CMD              (0x06)
#define SPI_FLASH_WRITE_DISABLE_CMD             (0x04)

/**
 * LiteSPI Register Maps
 */
/** pre-2025 liteSPI **/
#ifndef CSR_SPIFLASH_CORE_BASE
#define CSR_SPIFLASH_CORE_BASE CSR_SPIFLASH_BASE
#endif

/** post 2025 liteSPI **/
#ifndef CSR_SPIFLASH_BASE
#define CSR_SPIFLASH_BASE CSR_SPIFLASH_CORE_BASE
#endif
#define SPIFLASH_PHYCONFIG_LEN_OFFSET 0
#define SPIFLASH_PHYCONFIG_LEN_SIZE 8
#define SPIFLASH_PHYCONFIG_WIDTH_OFFSET 8
#define SPIFLASH_PHYCONFIG_WIDTH_SIZE 4
#define SPIFLASH_PHYCONFIG_MASK_OFFSET 16
#define SPIFLASH_PHYCONFIG_MASK_SIZE 8

#define SPIFLASH_STATUS_TX_READY_OFFSET 0
#define SPIFLASH_STATUS_RX_READY_OFFSET 1

#ifndef min
#define min(x, y) (((x) < (y)) ? (x) : (y))
#endif

typedef struct spiflash_regs_s {
    uint32_t PHY_CLK_DIV;
    uint32_t MASTER_CS;
    uint32_t MASTER_RXTX;
    uint32_t MASTER_STATUS;
    uint32_t MASTER_PHYCONFIG;
    uint32_t DUMMY_BITS;
} spiflash_regs_t;

static const spiflash_regs_t spiflash_regs[] = {
    {
        .PHY_CLK_DIV = (CSR_SPIFLASH_CORE_BASE + 0x800L),
        .MASTER_CS = (CSR_SPIFLASH_CORE_BASE +  0x08L),
        .MASTER_RXTX = (CSR_SPIFLASH_CORE_BASE +  0x10L),
        .MASTER_STATUS =  (CSR_SPIFLASH_CORE_BASE +  0x14L),
        .MASTER_PHYCONFIG = (CSR_SPIFLASH_CORE_BASE +  0x0cL),
        .DUMMY_BITS = (CSR_SPIFLASH_CORE_BASE + 0x00L)
    },
    {
        .PHY_CLK_DIV = (CSR_SPIFLASH_BASE + 0x00L),
        .MASTER_CS = (CSR_SPIFLASH_BASE + 0x0cL),
        .MASTER_RXTX = (CSR_SPIFLASH_BASE + 0x14L),
        .MASTER_STATUS = (CSR_SPIFLASH_BASE + 0x18L),
        .MASTER_PHYCONFIG = (CSR_SPIFLASH_BASE + 0x10L),
        .DUMMY_BITS = (CSR_SPIFLASH_BASE + 0x04L)
    }
};

static const spiflash_ops_t s25fl256s_ops = {
    .read = 0x6C, //4QOR
    .program = 0x12, //4PP
    .erase_sector = 0xDC, //4SE
    .cmd_addr_len = 4
};

static const spiflash_ops_t mx25u6432f_ops = {
    .read = 0x6B, //QREAD
    .program = 0x02, //PP
    .erase_sector = 0xD8, //BE64
    .cmd_addr_len = 3
};

static uint32_t get_flash_data(spiflash_dev_t* dev, uint32_t flash_addr, uint8_t* pData, uint32_t data_len)
{
    const uint32_t flash_base = 0x10000;
    uint32_t index = 0;
    uint32_t flash_data = 0;
    uint32_t flash_window = litepcie_readl(dev->fd, CSR_FLASH_ADAPTER_WINDOW0_ADDR);
    while(index < data_len)
    {
        if(flash_window != ((flash_addr+index) >> 16))
        {
            flash_window = (flash_addr+index) >> 16;
            litepcie_writel(dev->fd, CSR_FLASH_ADAPTER_WINDOW0_ADDR, flash_window);
        }

        flash_data = litepcie_readl(dev->fd, (flash_base + ((flash_addr+index) & 0xFFFF)));
        *(uint32_t*)&pData[index] = flash_data;
        index += 4;
    }

    return index;
}


static void spiflash_dummy_bits_setup(spiflash_dev_t* dev, unsigned int dummy_bits)
{
    litepcie_writel(dev->fd, spiflash_regs[dev->ip_rev].DUMMY_BITS, dummy_bits);
    LOG_DEBUG("Dummy bits set to: %" PRIx32 , litepcie_readl(dev->fd, spiflash_regs[dev->ip_rev].DUMMY_BITS));

}

static void spiflash_len_mask_width_write(spiflash_dev_t* dev, uint32_t len, uint32_t width, uint32_t mask)
{
    uint32_t tmp = len & ((1 <<  SPIFLASH_PHYCONFIG_LEN_SIZE) - 1);
    uint32_t word = tmp << SPIFLASH_PHYCONFIG_LEN_OFFSET;
    tmp = width & ((1 << SPIFLASH_PHYCONFIG_WIDTH_SIZE) - 1);
    word |= tmp << SPIFLASH_PHYCONFIG_WIDTH_OFFSET;
    tmp = mask & ((1 <<  SPIFLASH_PHYCONFIG_MASK_SIZE) - 1);
    word |= tmp << SPIFLASH_PHYCONFIG_MASK_OFFSET;
    litepcie_writel(dev->fd, spiflash_regs[dev->ip_rev].MASTER_PHYCONFIG, word);
}

static bool spiflash_tx_ready(spiflash_dev_t* dev)
{
    return (litepcie_readl(dev->fd, spiflash_regs[dev->ip_rev].MASTER_STATUS) >> SPIFLASH_STATUS_TX_READY_OFFSET) & 1;
}

static bool spiflash_rx_ready(spiflash_dev_t* dev)
{
    return (litepcie_readl(dev->fd, spiflash_regs[dev->ip_rev].MASTER_STATUS) >> SPIFLASH_STATUS_RX_READY_OFFSET) & 1;
}

static void spiflash_master_write(spiflash_dev_t* dev, uint32_t val, size_t len, size_t width, uint32_t mask)
{
    /* Be sure to empty RX queue before doing Xfer. */
    while (spiflash_rx_ready(dev))
        litepcie_readl(dev->fd, spiflash_regs[dev->ip_rev].MASTER_RXTX);

    /* Configure Master */
    spiflash_len_mask_width_write(dev, 8*len, width, mask);

    /* Set CS. */
    litepcie_writel(dev->fd, spiflash_regs[dev->ip_rev].MASTER_CS, 1UL);

    /* Do Xfer. */
    litepcie_writel(dev->fd, spiflash_regs[dev->ip_rev].MASTER_RXTX, val);

    while (!spiflash_rx_ready(dev));

    /* Clear CS. */
    litepcie_writel(dev->fd, spiflash_regs[dev->ip_rev].MASTER_CS, 0UL);
}

static uint8_t w_buf[SPI_FLASH_PROG_SIZE + 5];
static uint8_t r_buf[SPI_FLASH_PROG_SIZE + 5];

static void transfer_cmd(spiflash_dev_t* dev, const uint8_t *bs, uint8_t *resp, int len)
{
    uint32_t xfer_word = 0;
    uint32_t xfer_num_bytes = 0;

    litepcie_writel(dev->fd, spiflash_regs[dev->ip_rev].MASTER_CS, 1UL);

    /* Flush RX */
    while (spiflash_rx_ready(dev))
    {
        litepcie_readl(dev->fd, spiflash_regs[dev->ip_rev].MASTER_RXTX);
    }
    
    /* wait for tx ready */
    while (!spiflash_tx_ready(dev))
    {
        litepcie_readl(dev->fd, spiflash_regs[dev->ip_rev].MASTER_RXTX);
    }


    for (int i=0; i < len; i+=4)
    {
        xfer_num_bytes = min((len - i), 4);
        xfer_word = (uint32_t)bs[i];
        if(xfer_num_bytes > 1)
            xfer_word = (xfer_word << 8) | (uint32_t)bs[i+1];
        if(xfer_num_bytes > 2)
            xfer_word = (xfer_word << 8) | (uint32_t)bs[i+2];
        if(xfer_num_bytes > 3)
            xfer_word = (xfer_word << 8) | (uint32_t)bs[i+3];
            
        spiflash_len_mask_width_write(dev, (8*xfer_num_bytes), 1, 1);

        litepcie_writel(dev->fd, spiflash_regs[dev->ip_rev].MASTER_RXTX, (uint32_t)xfer_word);

        /* wait for rx ready */
        while (!spiflash_rx_ready(dev))
        {
            NS_DELAY(250);
        }

        xfer_word = litepcie_readl(dev->fd, spiflash_regs[dev->ip_rev].MASTER_RXTX);
        resp[i] = (uint8_t)((xfer_word >> (8*(xfer_num_bytes-1))) & 0xFF);
        if(xfer_num_bytes > 1)
            resp[i+1] = (uint8_t)((xfer_word >> (8*(xfer_num_bytes-2))) & 0xFF);
        if(xfer_num_bytes > 2)
            resp[i+2] = (uint8_t)((xfer_word >> (8*(xfer_num_bytes-3))) & 0xFF);
        if(xfer_num_bytes > 3)
            resp[i+3] = (uint8_t)((xfer_word) & 0xFF);
    }

    litepcie_writel(dev->fd, spiflash_regs[dev->ip_rev].MASTER_CS, 0UL);
}

static uint32_t spiflash_read_id_register(spiflash_dev_t* dev)
{
    uint8_t buf[4];
    w_buf[0] = SPI_FLASH_JEDEC_READ_ID_CMD;
    w_buf[1] = 0x00;
    w_buf[2] = 0x00;
    w_buf[3] = 0x00;
    transfer_cmd(dev, w_buf, buf, 4);

    LOG_DEBUG("[ID: %02x %02x %02x %02x]", buf[0], buf[1], buf[2], buf[3]);

    uint32_t flash_id = (buf[1] << 16) | (buf[2] << 8) | buf[3];
    return flash_id;
}

static uint32_t spiflash_read_status_register(spiflash_dev_t* dev)
{
    uint8_t buf[2];
    w_buf[0] = SPI_FLASH_JEDEC_READ_STATUS_REG_1_CMD;
    w_buf[1] = 0x00;
    transfer_cmd(dev, w_buf, buf, 2);

    LOG_DEBUG("[SR: %02x %02x]", buf[0], buf[1]);

    return buf[1];
}

static void spiflash_write_enable(spiflash_dev_t* dev)
{
    uint8_t buf[1];
    w_buf[0] = SPI_FLASH_WRITE_ENABLE_CMD;
    transfer_cmd(dev, w_buf, buf, 1);
}

static void spiflash_write_disable(spiflash_dev_t* dev)
{
    uint8_t buf[1];
    w_buf[0] = SPI_FLASH_WRITE_DISABLE_CMD;
    transfer_cmd(dev, w_buf, buf, 1);
}

static void page_program(spiflash_dev_t* dev, uint32_t addr, uint8_t *data, int len)
{
    int cmd_idx = 0;
    w_buf[cmd_idx++] = dev->ops.program;
    if(dev->ops.cmd_addr_len == 4)
    {
        w_buf[cmd_idx++] = addr>>24;
    }
    w_buf[cmd_idx++] = addr>>16;
    w_buf[cmd_idx++] = addr>>8;
    w_buf[cmd_idx++] = addr>>0;
    memcpy((void *)(&w_buf[cmd_idx]), (void *)data, len);
    transfer_cmd(dev, w_buf, r_buf, len+cmd_idx);
}

static void spiflash_sector_erase(spiflash_dev_t* dev, uint32_t addr)
{
    int cmd_idx = 0;
    w_buf[cmd_idx++] = dev->ops.erase_sector;
    if(dev->ops.cmd_addr_len == 4)
    {
        w_buf[cmd_idx++] = addr>>24;
    }
    w_buf[cmd_idx++] = addr>>16;
    w_buf[cmd_idx++] = addr>>8;
    w_buf[cmd_idx++] = addr>>0;
    transfer_cmd(dev, w_buf, r_buf, cmd_idx);
}

int32_t spiflash_erase(spiflash_dev_t* dev, uint32_t addr, uint32_t len)
{
    uint32_t i = 0;
    uint32_t j = 0;
    //Check Address is aligned to the erase sector
    if((addr % SPI_FLASH_ERASE_SIZE) != 0)
    {
        LOG_ERROR("Error: Flash Erase address must be 64K-aligned (0x%08X)", addr);
        return TS_STATUS_ERROR;
    }

    for (i=0; i<len; i+=SPI_FLASH_ERASE_SIZE) {
        LOG_DEBUG("Erase SPI Flash @0x%08lx", ((uint32_t)addr+i));
        spiflash_write_enable(dev);
        spiflash_sector_erase(dev, addr+i);

        while (spiflash_read_status_register(dev) & 1) {
            NS_DELAY(10000000);
        }

        /* check if region was really erased */
        for (j = 0; j < SPI_FLASH_ERASE_SIZE; j+=4) {
            uint32_t flash_word;
            get_flash_data(dev, (addr+i+j), (uint8_t*)&flash_word, sizeof(uint32_t));
            if (flash_word != 0xffffffff) {
                LOG_ERROR("Error: location 0x%08x not erased (0x%08x)", addr+i+j, flash_word);
                return TS_STATUS_ERROR;
            }
        }

        //Report progress
        if(dev->op_progress != NULL)
        {
            dev->op_progress(dev->op_progress_ctx, SPI_FLASH_ERASE_SIZE, len);
        }
    }
    spiflash_write_disable(dev);

    return TS_STATUS_OK;
}

int32_t spiflash_write(spiflash_dev_t* dev, uint32_t addr, const uint8_t *pData, uint32_t len)
{
    int res = 0;
    uint32_t w_len = min(len, SPI_FLASH_PROG_SIZE);
    uint32_t offset = 0;
    uint32_t j = 0;

    LOG_DEBUG("Write SPI Flash @0x%08lx", ((uint32_t)addr));

    while(w_len) {
        spiflash_write_enable(dev);
        page_program(dev, addr+offset, (uint8_t*)pData+offset, w_len);

        while(spiflash_read_status_register(dev) & 1) {
            NS_DELAY(10000);
        }

        spiflash_read(dev, addr+offset, (uint8_t*)r_buf, w_len);
        for (j = 0; j < w_len; j++) {
            if (r_buf[j] != pData[offset+j]) {
                LOG_ERROR("Error: verify failed at 0x%08x (0x%02x should be 0x%02x)", (uint32_t)(addr+offset+j), r_buf[j], pData[offset+j]);
                spiflash_write_disable(dev);
                return TS_STATUS_ERROR;
            }
        }

        //Report progress
        if(dev->op_progress != NULL)
        {
            dev->op_progress(dev->op_progress_ctx, w_len, len);
        }

        offset += w_len;
        w_len = min(len-offset, SPI_FLASH_PROG_SIZE);
        res = offset;
    }
    spiflash_write_disable(dev);

    return res;
}

int32_t spiflash_init(file_t fd, spiflash_dev_t* dev)
{
    uint32_t flash_id = 0;
    uint32_t divisor = SPI_FLASH_CLK_DIV_DEFAULT;
    uint32_t ip_version = 0;

    dev->fd = fd;

    // Get IP Version
    ip_version = litepcie_readl(fd, CSR_DEV_STATUS_LITEX_REL_ADDR);
    if(ip_version >= LITEX_VERSION(2025, 4)) // 2025.4+
    {
        dev->ip_rev = 1;
    }
    else
    {
        dev->ip_rev = 0;
    }

    spiflash_dummy_bits_setup(dev, 8);

    while(divisor <= SPI_FLASH_CLK_DIV_MAX)
    {
        litepcie_writel(dev->fd, spiflash_regs[dev->ip_rev].PHY_CLK_DIV, divisor);
        flash_id = spiflash_read_id_register(dev); //First ID read returns garbage?
        flash_id = spiflash_read_id_register(dev);
        
        if((flash_id == 0x010219) ||
            (flash_id == 0xC22017) ||
            (flash_id == 0xC22537) ||
            (flash_id == 0xC22B27))
        {
            LOG_DEBUG("Using SPIFLASH Divisor %ld", divisor);
            break;
        }
        else
        {
            divisor++;
            continue;
        }
    }

    dev->mfg_code = (flash_id >> 16) & 0xFF;
    dev->part_id = (flash_id & 0xFFFF);

    switch(flash_id)
    {
        case 0x010219:
        {
            dev->ops = s25fl256s_ops;
            break;
        }
        case 0xC22017:
        case 0xC22537:
        case 0xC22B27: // Test SPI device MX25S6433F
        {
            dev->ops = mx25u6432f_ops;
            break;
        }
        default:
        {
            LOG_ERROR("Unknown SPI Flash Device %08X", flash_id);
            break;
        }
    }

//     /* Quad / QPI Configuration. */
// #ifdef SPIFLASH_MODULE_QUAD_CAPABLE
//     printf("Enabling Quad mode...\n");
//     spiflash_master_write(fd, 0x00000006, 1, 1, 0x1);
//     spiflash_master_write(fd, 0x00014307, 3, 1, 0x1);

// #ifdef SPIFLASH_MODULE_QPI_CAPABLE
//     printf("Switching to QPI mode...\n");
//     spiflash_master_write(fd, 0x00000035, 1, 1, 0x1);
// #endif

// #endif

    return TS_STATUS_OK;
}

int32_t spiflash_read(spiflash_dev_t* dev, uint32_t addr, uint8_t* pData, uint32_t len)
{
    uint32_t read_len = 0;
    uint32_t remaining = len;
    uint32_t tempAddr = (addr/4)*4; //Round address down
    uint8_t flashBytes[4];
    //Validate Address and Lengths
    if(len == 0)
    {
        LOG_ERROR("Failed to read with length 0");
        return TS_STATUS_ERROR;
    }
    
    if(addr % 4 != 0)
    {
        read_len = get_flash_data(dev, tempAddr, flashBytes, 4);
        if( 4 != read_len )
        {
            LOG_ERROR("Failed to read with length 0");
            return TS_STATUS_ERROR;
        }
        read_len = (4 - (addr%4));
        memcpy(pData, &flashBytes[addr%4], read_len);
        addr += read_len;
        pData += read_len;
        remaining -= read_len;
    }

    read_len = get_flash_data(dev, addr, pData, (remaining/4)*4);
    addr += read_len;
    pData += read_len;
    remaining -= read_len;

    if(remaining >= 4)
    {
        LOG_ERROR("SPIFLASH READ FAILED (%d - %d)", read_len, remaining);
    }
    else if(0 != remaining)
    {
        //Should only be 1-3 bytes left
        if( 4 != get_flash_data(dev, addr, flashBytes, 4))
        {
            LOG_ERROR("Failed to read end of data (%d / %d)", read_len, len);
            return TS_STATUS_ERROR;
        }
        memcpy(pData, flashBytes, remaining);
    }

    return (int32_t)len;
}
