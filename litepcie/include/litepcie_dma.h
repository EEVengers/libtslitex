/* SPDX-License-Identifier: BSD-2-Clause
 *
 * LitePCIe library
 *
 * This file is part of LitePCIe.
 *
 * Copyright (C) 2018-2023 / EnjoyDigital  / florent@enjoy-digital.fr
 *
 */

#ifndef LITEPCIE_LIB_DMA_H
#define LITEPCIE_LIB_DMA_H

#include <stdint.h>

#if defined(_WIN32)
typedef void* pollfd_s;
#else
#include <poll.h>
typedef struct pollfd pollfd_s;
#endif

#include "litepcie_public.h"
#include "litepcie_helpers.h"


struct litepcie_dma_ctrl {
    uint8_t use_reader, use_writer, loopback, zero_copy;
    file_t dma_fd;
    pollfd_s fds;
    char *buf_rd, *buf_wr;
    int64_t reader_hw_count, reader_sw_count;
    int64_t writer_hw_count, writer_sw_count;
    unsigned buffers_available_read, buffers_available_write;
    unsigned usr_read_buf_offset, usr_write_buf_offset;
    struct litepcie_ioctl_mmap_dma_info mmap_dma_info;
    struct litepcie_ioctl_mmap_dma_update mmap_dma_update;
};

void litepcie_dma_set_loopback(file_t fd, uint8_t loopback_enable);
void litepcie_dma_reader(file_t fd, uint8_t enable, int64_t *hw_count, int64_t *sw_count);
void litepcie_dma_writer(file_t fd, uint8_t enable, int64_t *hw_count, int64_t *sw_count);

uint8_t litepcie_request_dma(file_t fd, uint8_t reader, uint8_t writer);
void litepcie_release_dma(file_t fd, uint8_t reader, uint8_t writer);

int litepcie_dma_init(struct litepcie_dma_ctrl *dma, const char *device_name, uint8_t zero_copy);
void litepcie_dma_cleanup(struct litepcie_dma_ctrl *dma);
void litepcie_dma_process(struct litepcie_dma_ctrl *dma);
char *litepcie_dma_next_read_buffer(struct litepcie_dma_ctrl *dma);
char *litepcie_dma_next_write_buffer(struct litepcie_dma_ctrl *dma);

#endif /* LITEPCIE_LIB_DMA_H */
