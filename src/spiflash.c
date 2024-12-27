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

#define SPI_FLASH_PROG_SIZE	256
/* erase page size in bytes, check flash datasheet */
#define SPI_FLASH_ERASE_SIZE (64*1024)

#define SPI_FLASH_WINDOW_SIZE 0x10000

#ifndef min
#define min(x, y) (((x) < (y)) ? (x) : (y))
#endif

static uint32_t get_flash_data(file_t fd, uint32_t flash_addr, uint8_t* pData, uint32_t data_len)
{
    const uint32_t flash_base = 0x10000;
    uint32_t index = 0;
    uint32_t flash_data = 0;
    uint32_t flash_window = 0;
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

    litepcie_writel(fd, CSR_FLASH_ADAPTER_WINDOW0_ADDR, 0);

    return index;
}

#ifdef CSR_SPIFLASH_CORE_MMAP_DUMMY_BITS_ADDR
static void spiflash_dummy_bits_setup(file_t fd, unsigned int dummy_bits)
{
    litepcie_writel(fd, CSR_SPIFLASH_CORE_MMAP_DUMMY_BITS_ADDR, dummy_bits);
    LOG_DEBUG("Dummy bits set to: %" PRIx32 "\n\r", litepcie_readl(fd, CSR_SPIFLASH_CORE_MMAP_DUMMY_BITS_ADDR));

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

static volatile uint8_t w_buf[SPI_FLASH_PROG_SIZE + 5];
static volatile uint8_t r_buf[SPI_FLASH_PROG_SIZE + 5];

static uint32_t transfer_byte(file_t fd, uint8_t b)
{
    
    /* wait for tx ready */
    while (!spiflash_tx_ready(fd));
    
    litepcie_writel(fd, CSR_SPIFLASH_CORE_MASTER_RXTX_ADDR, (uint32_t)b);

    /* wait for rx ready */
    while (!spiflash_rx_ready(fd))
    {
        NS_DELAY(250);
    }

    return litepcie_readl(fd, CSR_SPIFLASH_CORE_MASTER_RXTX_ADDR);
}

static void transfer_cmd(file_t fd, volatile uint8_t *bs, volatile uint8_t *resp, int len)
{
    spiflash_len_mask_width_write(fd, 8, 1, 1);
    litepcie_writel(fd, CSR_SPIFLASH_CORE_MASTER_CS_ADDR, 1UL);

    for (int i=0; i < len; i++) {
        resp[i] = transfer_byte(fd, bs[i]);
    }

    litepcie_writel(fd, CSR_SPIFLASH_CORE_MASTER_CS_ADDR, 0UL);
}

static uint32_t spiflash_read_id_register(file_t fd)
{
    volatile uint8_t buf[4];
    w_buf[0] = 0x9F;
    w_buf[1] = 0x00;
    w_buf[2] = 0x00;
    w_buf[3] = 0x00;
    transfer_cmd(fd, w_buf, buf, 4);

    LOG_DEBUG("[ID: %02x %02x %02x %02x]", buf[0], buf[1], buf[2], buf[3]);

    /* FIXME normally the status should be in buf[1],
    but we have to read it a few more times to be
    stable for unknown reasons */
    return buf[3];
}

static uint32_t spiflash_read_status_register(file_t fd)
{
    volatile uint8_t buf[2];
    w_buf[0] = 0x05;
    w_buf[1] = 0x00;
    transfer_cmd(fd, w_buf, buf, 2);

    LOG_DEBUG("[SR: %02x %02x]", buf[0], buf[1]);

    return buf[1];
}

static void spiflash_write_enable(file_t fd)
{
    uint8_t buf[1];
    w_buf[0] = 0x06;
    transfer_cmd(fd, w_buf, buf, 1);
}

static void spiflash_write_disable(file_t fd)
{
    uint8_t buf[1];
    w_buf[0] = 0x04;
    transfer_cmd(fd, w_buf, buf, 1);
}

static void page_program(file_t fd, uint32_t addr, uint8_t *data, int len)
{
    w_buf[0] = 0x12;
    w_buf[1] = addr>>24;
    w_buf[2] = addr>>16;
    w_buf[3] = addr>>8;
    w_buf[4] = addr>>0;
    memcpy((void *)(&w_buf[5]), (void *)data, len);
    transfer_cmd(fd, w_buf, r_buf, len+5);
}

static void spiflash_sector_erase(file_t fd, uint32_t addr)
{
    w_buf[0] = 0xdc;
    w_buf[1] = addr>>24;
    w_buf[2] = addr>>16;
    w_buf[3] = addr>>8;
    w_buf[4] = addr>>0;
    transfer_cmd(fd, w_buf, r_buf, 5);
}

int32_t spiflash_erase(file_t fd, uint32_t addr, uint32_t len)
{
    uint32_t i = 0;
    uint32_t j = 0;
    //Check Address is aligned to the erase sector
    if(addr & 0xffff)
    {
        LOG_ERROR("Error: Flash Erase address must be 64K-aligned (0x%08X)", addr);
        return TS_STATUS_ERROR;
    }
    
    spiflash_write_enable(fd);

    for (i=0; i<len; i+=SPI_FLASH_ERASE_SIZE) {
        LOG_DEBUG("Erase SPI Flash @0x%08lx", ((uint32_t)addr+i));
        spiflash_sector_erase(fd, addr+i);

        while (spiflash_read_status_register(fd) & 1) {
            LOG_DEBUG(".");
            NS_DELAY(250000000);
        }
        LOG_DEBUG("\n");

        /* check if region was really erased */
        for (j = 0; j < SPI_FLASH_ERASE_SIZE; j+=4) {
            uint32_t flash_word;
            get_flash_data(fd, (addr+i+j), (uint8_t*)&flash_word, sizeof(uint32_t));
            if (flash_word != 0xffffffff) {
                LOG_ERROR("Error: location 0x%08lx not erased (0x%08x)\n", addr+i+j, flash_word);
                return TS_STATUS_ERROR;
            }
        }
    }
    spiflash_write_disable(fd);

    return TS_STATUS_OK;
}

int32_t spiflash_write(file_t fd, uint32_t addr, uint8_t *pData, uint32_t len)
{
    int res = 0;
    uint32_t w_len = min(len, SPI_FLASH_PROG_SIZE);
    uint32_t offset = 0;
    uint32_t j = 0;

    LOG_DEBUG("Write SPI Flash @0x%08lx", ((uint32_t)addr));

    spiflash_write_enable(fd);
    while(w_len) {
        page_program(fd, addr+offset, pData+offset, w_len);

        while(spiflash_read_status_register(fd) & 1) {
            LOG_DEBUG(".");
        }

        get_flash_data(fd, addr+offset, (uint8_t*)r_buf, w_len);
        for (j = 0; j < w_len; j++) {
            if (r_buf[j] != pData[offset+j]) {
                LOG_ERROR("Error: verify failed at 0x%08lx (0x%02x should be 0x%02x)", (uint32_t)(offset+j), r_buf[j], pData[offset+j]);
            }
        }

        offset += w_len;
        w_len = min(len-offset, SPI_FLASH_PROG_SIZE);
        res = offset;
    }
    spiflash_write_disable(fd);

    LOG_DEBUG("\n");

    return res;
}

void spiflash_init(file_t fd)
{
#ifdef CSR_SPIFLASH_CORE_MMAP_DUMMY_BITS_ADDR
    spiflash_dummy_bits_setup(fd, 8);
#endif

#ifdef CSR_SPIFLASH_CORE_MASTER_CS_ADDR

    spiflash_read_id_register(fd); //First ID read returns garbage?
    spiflash_read_id_register(fd);

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
#endif
}

int32_t spiflash_read(file_t fd, uint32_t addr, uint8_t* pData, uint32_t len)
{
    //Validate Address and Lengths
    if((len % 4) || (addr % 4))
    {
        //Only supports word-reads currently
        return TS_STATUS_ERROR;
    }
    int32_t read_len = get_flash_data(fd, addr, pData, len);

    return len;
}
