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

#define SPI_FLASH_PROG_SIZE	256
#define SPI_FLASH_ERASE_SIZE (64*1024)

#define SPI_FLASH_WINDOW_SIZE 0x10000

#define SPI_FLASH_CLK_DIV_DEFAULT   (1)
#define SPI_FLASH_CLK_DIV_MAX       (10)

#define SPI_FLASH_JEDEC_READ_ID_CMD             (0x9F)
#define SPI_FLASH_JEDEC_READ_STATUS_REG_1_CMD   (0x05)
#define SPI_FLASH_WRITE_ENABLE_CMD              (0x06)
#define SPI_FLASH_WRITE_DISABLE_CMD             (0x04)

#ifndef min
#define min(x, y) (((x) < (y)) ? (x) : (y))
#endif

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

static uint32_t get_flash_data(file_t fd, uint32_t flash_addr, uint8_t* pData, uint32_t data_len)
{
    const uint32_t flash_base = 0x10000;
    uint32_t index = 0;
    uint32_t flash_data = 0;
    uint32_t flash_window = litepcie_readl(fd, CSR_FLASH_ADAPTER_WINDOW0_ADDR);
    while(index < data_len)
    {
        if(flash_window != ((flash_addr+index) >> 16))
        {
            flash_window = (flash_addr+index) >> 16;
            litepcie_writel(fd, CSR_FLASH_ADAPTER_WINDOW0_ADDR, flash_window);
        }

        flash_data = litepcie_readl(fd, (flash_base + ((flash_addr+index) & 0xFFFF)));
        *(uint32_t*)&pData[index] = flash_data;
        index += 4;
    }

    return index;
}

#ifdef CSR_SPIFLASH_CORE_MMAP_DUMMY_BITS_ADDR
static void spiflash_dummy_bits_setup(file_t fd, unsigned int dummy_bits)
{
    litepcie_writel(fd, CSR_SPIFLASH_CORE_MMAP_DUMMY_BITS_ADDR, dummy_bits);
    LOG_DEBUG("Dummy bits set to: %" PRIx32 , litepcie_readl(fd, CSR_SPIFLASH_CORE_MMAP_DUMMY_BITS_ADDR));

}
#endif

static void spiflash_len_mask_width_write(file_t fd, uint32_t len, uint32_t width, uint32_t mask)
{
    uint32_t tmp = len & ((1 <<  CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_LEN_SIZE) - 1);
    uint32_t word = tmp << CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_LEN_OFFSET;
    tmp = width & ((1 << CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_WIDTH_SIZE) - 1);
    word |= tmp << CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_WIDTH_OFFSET;
    tmp = mask & ((1 <<  CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_MASK_SIZE) - 1);
    word |= tmp << CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_MASK_OFFSET;
    litepcie_writel(fd, CSR_SPIFLASH_CORE_MASTER_PHYCONFIG_ADDR, word);
}

static bool spiflash_tx_ready(file_t fd)
{
    return (litepcie_readl(fd, CSR_SPIFLASH_CORE_MASTER_STATUS_ADDR) >> CSR_SPIFLASH_CORE_MASTER_STATUS_TX_READY_OFFSET) & 1;
}

static bool spiflash_rx_ready(file_t fd)
{
    return (litepcie_readl(fd, CSR_SPIFLASH_CORE_MASTER_STATUS_ADDR) >> CSR_SPIFLASH_CORE_MASTER_STATUS_RX_READY_OFFSET) & 1;
}

static void spiflash_master_write(file_t fd, uint32_t val, size_t len, size_t width, uint32_t mask)
{
    /* Be sure to empty RX queue before doing Xfer. */
    while (spiflash_rx_ready(fd))
        litepcie_readl(fd, CSR_SPIFLASH_CORE_MASTER_RXTX_ADDR);

    /* Configure Master */
    spiflash_len_mask_width_write(fd, 8*len, width, mask);

    /* Set CS. */
    litepcie_writel(fd, CSR_SPIFLASH_CORE_MASTER_CS_ADDR, 1UL);

    /* Do Xfer. */
    litepcie_writel(fd, CSR_SPIFLASH_CORE_MASTER_RXTX_ADDR, val);

    while (!spiflash_rx_ready(fd));

    /* Clear CS. */
    // spiflash_core_master_cs_write(0);
    litepcie_writel(fd, CSR_SPIFLASH_CORE_MASTER_CS_ADDR, 0UL);
}

static uint8_t w_buf[SPI_FLASH_PROG_SIZE + 5];
static uint8_t r_buf[SPI_FLASH_PROG_SIZE + 5];

static void transfer_cmd(file_t fd, const uint8_t *bs, uint8_t *resp, int len)
{
    uint32_t xfer_word = 0;
    uint32_t xfer_num_bytes = 0;

    litepcie_writel(fd, CSR_SPIFLASH_CORE_MASTER_CS_ADDR, 1UL);

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

        
        spiflash_len_mask_width_write(fd, (8*xfer_num_bytes), 1, 1);

        /* wait for tx ready */
        while (!spiflash_tx_ready(fd));
        
        litepcie_writel(fd, CSR_SPIFLASH_CORE_MASTER_RXTX_ADDR, (uint32_t)xfer_word);

        /* wait for rx ready */
        while (!spiflash_rx_ready(fd))
        {
            NS_DELAY(250);
        }
        xfer_word = litepcie_readl(fd, CSR_SPIFLASH_CORE_MASTER_RXTX_ADDR);
        resp[i] = (uint8_t)((xfer_word >> (8*(xfer_num_bytes-1))) & 0xFF);
        if(xfer_num_bytes > 1)
            resp[i+1] = (uint8_t)((xfer_word >> (8*(xfer_num_bytes-2))) & 0xFF);
        if(xfer_num_bytes > 2)
            resp[i+2] = (uint8_t)((xfer_word >> (8*(xfer_num_bytes-3))) & 0xFF);
        if(xfer_num_bytes > 3)
            resp[i+3] = (uint8_t)((xfer_word) & 0xFF);
    }

    litepcie_writel(fd, CSR_SPIFLASH_CORE_MASTER_CS_ADDR, 0UL);
}

static uint32_t spiflash_read_id_register(file_t fd)
{
    uint8_t buf[4];
    w_buf[0] = SPI_FLASH_JEDEC_READ_ID_CMD;
    w_buf[1] = 0x00;
    w_buf[2] = 0x00;
    w_buf[3] = 0x00;
    transfer_cmd(fd, w_buf, buf, 4);

    LOG_DEBUG("[ID: %02x %02x %02x %02x]", buf[0], buf[1], buf[2], buf[3]);

    uint32_t flash_id = (buf[1] << 16) | (buf[2] << 8) | buf[3];
    return flash_id;
}

static uint32_t spiflash_read_status_register(file_t fd)
{
    uint8_t buf[2];
    w_buf[0] = SPI_FLASH_JEDEC_READ_STATUS_REG_1_CMD;
    w_buf[1] = 0x00;
    transfer_cmd(fd, w_buf, buf, 2);

    LOG_DEBUG("[SR: %02x %02x]", buf[0], buf[1]);

    return buf[1];
}

static void spiflash_write_enable(file_t fd)
{
    uint8_t buf[1];
    w_buf[0] = SPI_FLASH_WRITE_ENABLE_CMD;
    transfer_cmd(fd, w_buf, buf, 1);
}

static void spiflash_write_disable(file_t fd)
{
    uint8_t buf[1];
    w_buf[0] = SPI_FLASH_WRITE_DISABLE_CMD;
    transfer_cmd(fd, w_buf, buf, 1);
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
    transfer_cmd(dev->fd, w_buf, r_buf, len+cmd_idx);
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
    transfer_cmd(dev->fd, w_buf, r_buf, cmd_idx);
}

int32_t spiflash_erase(spiflash_dev_t* dev, uint32_t addr, uint32_t len)
{
    uint32_t i = 0;
    uint32_t j = 0;
    //Check Address is aligned to the erase sector
    if(addr & 0xffff)
    {
        LOG_ERROR("Error: Flash Erase address must be 64K-aligned (0x%08X)", addr);
        return TS_STATUS_ERROR;
    }

    for (i=0; i<len; i+=SPI_FLASH_ERASE_SIZE) {
        LOG_DEBUG("Erase SPI Flash @0x%08lx", ((uint32_t)addr+i));
        spiflash_write_enable(dev->fd);
        spiflash_sector_erase(dev, addr+i);

        while (spiflash_read_status_register(dev->fd) & 1) {
            NS_DELAY(250000000);
        }

        /* check if region was really erased */
        for (j = 0; j < SPI_FLASH_ERASE_SIZE; j+=4) {
            uint32_t flash_word;
            get_flash_data(dev->fd, (addr+i+j), (uint8_t*)&flash_word, sizeof(uint32_t));
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
    spiflash_write_disable(dev->fd);

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
        spiflash_write_enable(dev->fd);
        page_program(dev, addr+offset, (uint8_t*)pData+offset, w_len);

        while(spiflash_read_status_register(dev->fd) & 1) {
            NS_DELAY(10000);
        }

        get_flash_data(dev->fd, addr+offset, (uint8_t*)r_buf, w_len);
        for (j = 0; j < w_len; j++) {
            if (r_buf[j] != pData[offset+j]) {
                LOG_ERROR("Error: verify failed at 0x%08x (0x%02x should be 0x%02x)", (uint32_t)(addr+offset+j), r_buf[j], pData[offset+j]);
                spiflash_write_disable(dev->fd);
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
    spiflash_write_disable(dev->fd);

    return res;
}

int32_t spiflash_init(file_t fd, spiflash_dev_t* dev)
{
    uint32_t flash_id = 0;
    uint32_t divisor = SPI_FLASH_CLK_DIV_DEFAULT;

    dev->fd = fd;

#ifdef CSR_SPIFLASH_CORE_MMAP_DUMMY_BITS_ADDR
    spiflash_dummy_bits_setup(fd, 8);
#endif

    while(divisor <= SPI_FLASH_CLK_DIV_MAX)
    {
        litepcie_writel(fd, CSR_SPIFLASH_PHY_CLK_DIVISOR_ADDR, divisor);
        flash_id = spiflash_read_id_register(dev->fd); //First ID read returns garbage?
        flash_id = spiflash_read_id_register(dev->fd);
        
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
    //Validate Address and Lengths
    if((len % 4) || (addr % 4))
    {
        //Only supports word-reads currently
        return TS_STATUS_ERROR;
    }
    int32_t read_len = get_flash_data(dev->fd, addr, pData, len);

    return len;
}
