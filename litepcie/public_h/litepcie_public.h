/* SPDX-License-Identifier: BSD-2-Clause
 *
 * LitePCIe Windows Driver
 *
 * Copyright (C) 2023 / Nate Meyer / Nate.Devel@gmail.com
 *
 */

#pragma once

#include "csr.h"
#include "soc.h"
#include "litepcie_dmadrv.h"

/* spi */
#define SPI_CTRL_START 0x1
#define SPI_CTRL_LENGTH (1<<8)
#define SPI_STATUS_DONE 0x1
#define SPI_TIMEOUT 100000 /* in us */

/* pcie */
#define DMA_TABLE_LOOP_INDEX (1 << 0)
#define DMA_TABLE_LOOP_COUNT (1 << 16)

struct litepcie_ioctl_reg {
	UINT32 reg;
	UINT32 val;
	UINT8 is_write;
};

struct litepcie_ioctl_flash {
	int tx_len; /* 8 to 40 */
	UINT64 tx_data; /* 8 to 40 bits */
	UINT64 rx_data; /* 40 bits */
};

struct litepcie_ioctl_icap {
	UINT8 addr;
	UINT32 data;
};

struct litepcie_ioctl_dma {
	UINT8 loopback_enable;
};

struct litepcie_ioctl_dma_writer {
	UINT8 enable;
	INT64 hw_count;
	INT64 sw_count;
};

struct litepcie_ioctl_dma_reader {
	UINT8 enable;
	INT64 hw_count;
	INT64 sw_count;
};

struct litepcie_ioctl_lock {
	UINT8 dma_reader_request;
	UINT8 dma_writer_request;
	UINT8 dma_reader_release;
	UINT8 dma_writer_release;
	UINT8 dma_reader_status;
	UINT8 dma_writer_status;
};

struct litepcie_ioctl_mmap_dma_info {
	UINT64 dma_tx_buf_offset;
	UINT64 dma_tx_buf_size;
	UINT64 dma_tx_buf_count;

	UINT64 dma_rx_buf_offset;
	UINT64 dma_rx_buf_size;
	UINT64 dma_rx_buf_count;
};

struct litepcie_ioctl_mmap_dma_update {
	INT64 sw_count;
};

#define LITEPCIE_IOCTL(id)		CTL_CODE(FILE_DEVICE_UNKNOWN, id, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define LITEPCIE_IOCTL_REG               LITEPCIE_IOCTL(0) // struct litepcie_ioctl_reg
#define LITEPCIE_IOCTL_FLASH             LITEPCIE_IOCTL(1) // struct litepcie_ioctl_flash
#define LITEPCIE_IOCTL_ICAP              LITEPCIE_IOCTL(2) // struct litepcie_ioctl_icap

#define LITEPCIE_IOCTL_DMA                       LITEPCIE_IOCTL(20) // struct litepcie_ioctl_dma
#define LITEPCIE_IOCTL_DMA_WRITER                LITEPCIE_IOCTL(21) // struct litepcie_ioctl_dma_writer
#define LITEPCIE_IOCTL_DMA_READER                LITEPCIE_IOCTL(22) // struct litepcie_ioctl_dma_reader
#define LITEPCIE_IOCTL_MMAP_DMA_INFO             LITEPCIE_IOCTL(24) // struct litepcie_ioctl_mmap_dma_info
#define LITEPCIE_IOCTL_LOCK                      LITEPCIE_IOCTL(25) // struct litepcie_ioctl_lock
#define LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE    LITEPCIE_IOCTL(26) // struct litepcie_ioctl_mmap_dma_update
#define LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE    LITEPCIE_IOCTL(27) // struct litepcie_ioctl_mmap_dma_update

//
// Define an Interface Guid so that apps can find the device and talk to it.
//

DEFINE_GUID (GUID_DEVINTERFACE_litepciedrv,
        0x164adc02, 0xe1ae, 0x4fe1, 0xa9, 0x4, 0x9a, 0x1, 0x35, 0x77, 0xb8, 0x91);
    // {164ADC02-E1AE-4FE1-A904-9A013577B891}

