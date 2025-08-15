/* SPDX-License-Identifier: BSD-2-Clause
 *
 * LitePCIe driver
 *
 * This file is part of LitePCIe.
 *
 * Copyright (C) 2018-2023 / EnjoyDigital  / florent@enjoy-digital.fr
 *
 */

#ifndef _MAC_LITEPCIE_H
#define _MAC_LITEPCIE_H

#include <stdbool.h>

#include "csr.h"
#include "config.h"


struct litepcie_ioctl_dma {
	uint8_t loopback_enable;
	uint8_t channel;
} __attribute__((packed));

struct litepcie_ioctl_dma_writer {
	uint32_t enable;
	uint32_t channel;
	int64_t hw_count;
	int64_t sw_count;
	int64_t lost_count;
} __attribute__((packed));

struct litepcie_ioctl_dma_reader {
	uint32_t enable;
	uint32_t channel;
	int64_t hw_count;
	int64_t sw_count;
	int64_t lost_count;
} __attribute__((packed));

struct litepcie_ioctl_lock {
	uint8_t dma_reader_request;
	uint8_t dma_writer_request;
	uint8_t dma_reader_release;
	uint8_t dma_writer_release;
	uint8_t dma_reader_status;
	uint8_t dma_writer_status;
}__attribute__((packed));

struct litepcie_ioctl_mmap_dma_info {
	uint64_t dma_tx_buf_offset;
	uint64_t dma_tx_buf_size;
	uint64_t dma_tx_buf_count;

	uint64_t dma_rx_buf_offset;
	uint64_t dma_rx_buf_size;
	uint64_t dma_rx_buf_count;
};

struct litepcie_ioctl_mmap_dma_update {
	int64_t sw_count;
};

enum LitePCIeMessageType {
    LITEPCIE_CONFIG_DMA_READER_CHANNEL,
    LITEPCIE_CONFIG_DMA_WRITER_CHANNEL,
    LITEPCIE_READ_CSR,
    LITEPCIE_WRITE_CSR,
    LITEPCIE_ICAP,
    LITEPCIE_FLASH,
	LITEPCIE_CONFIG_DMA,
	LITEPCIE_CONFIG_DMA_LOCK,
    LITEPCIE_DMA_READ,
    LITEPCIE_DMA_WRITE
};

enum LitePCIeMemoryType {
    LITEPCIE_DMA_READER = 0x00010000,
    LITEPCIE_DMA_WRITER = 0x00020000,
    LITEPCIE_DMA_COUNTS = 0x00040000,
};

typedef struct DMACounts {
    uint64_t hwReaderCountTotal;
    uint64_t hwReaderCountPrev;
    uint64_t hwReaderLost;
    uint64_t hwWriterCountTotal;
    uint64_t hwWriterCountPrev;
    uint64_t hwWriterLost;
} __attribute__((packed)) DMACounts;

typedef struct litepcie_ioctl_flash {
    uint32_t tx_len; /* 8 to 40 */
    uint64_t tx_data; /* 8 to 40 bits */
    uint64_t rx_data; /* 40 bits */
} __attribute__((packed)) LitePCIeFlashCallData;

typedef struct litepcie_ioctl_icap {
    uint8_t addr;
    uint32_t data;
} __attribute__((packed)) LitePCIeICAPCallData;

typedef struct litepcie_ioctl_dma_transfer_s {
    uint32_t channel;
    uint32_t length;
    void* buffer_addr;
} __attribute__((packed)) litepcie_ioctl_dma_transfer_t;


#define LITEPCIE_IOCTL_FLASH             LITEPCIE_FLASH
#define LITEPCIE_IOCTL_ICAP              LITEPCIE_ICAP

#define LITEPCIE_IOCTL_DMA                       LITEPCIE_CONFIG_DMA
#define LITEPCIE_IOCTL_DMA_WRITER                LITEPCIE_CONFIG_DMA_WRITER_CHANNEL
#define LITEPCIE_IOCTL_DMA_READER                LITEPCIE_CONFIG_DMA_READER_CHANNEL
#define LITEPCIE_IOCTL_MMAP_DMA_INFO             LITEPCIE_CONFIG_DMA_WRITER_CHANNEL
#define LITEPCIE_IOCTL_LOCK                      LITEPCIE_CONFIG_DMA_LOCK
// #define LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE    //TBD
// #define LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE    //TBD
#define LITEPCIE_IOCTL_DMA_READ					LITEPCIE_DMA_READ
#define LITEPCIE_IOCTL_DMA_WRITE				LITEPCIE_DMA_WRITE

#endif /* _LINUX_LITEPCIE_H */
