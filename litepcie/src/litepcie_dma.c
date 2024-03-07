/* SPDX-License-Identifier: BSD-2-Clause
 *
 * LitePCIe library
 *
 * This file is part of LitePCIe.
 *
 * Copyright (C) 2018-2023 / EnjoyDigital  / florent@enjoy-digital.fr
 *
 */


#if defined(_WIN32)
#include <Windows.h>
#include <malloc.h>
#define ssize_t int64_t
#else
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/mman.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include "litepcie_dma.h"
#include <litepcie.h>
#include "litepcie_helpers.h"


void litepcie_dma_set_loopback(file_t fd, uint8_t loopback_enable) {
    struct litepcie_ioctl_dma m;
    m.loopback_enable = loopback_enable;
    checked_ioctl(ioctl_args(fd, LITEPCIE_IOCTL_DMA, m));
}

void litepcie_dma_writer(file_t fd, uint8_t enable, int64_t *hw_count, int64_t *sw_count) {
    struct litepcie_ioctl_dma_writer m;
    m.enable = enable;
    checked_ioctl(ioctl_args(fd, LITEPCIE_IOCTL_DMA_WRITER, m));
    *hw_count = m.hw_count;
    *sw_count = m.sw_count;
}

void litepcie_dma_reader(file_t fd, uint8_t enable, int64_t *hw_count, int64_t *sw_count) {
    struct litepcie_ioctl_dma_reader m;
    m.enable = enable;
    checked_ioctl(ioctl_args(fd, LITEPCIE_IOCTL_DMA_READER, m));
    *hw_count = m.hw_count;
    *sw_count = m.sw_count;
}

/* lock */

uint8_t litepcie_request_dma(file_t fd, uint8_t reader, uint8_t writer) {
    struct litepcie_ioctl_lock m;
    m.dma_reader_request = reader > 0;
    m.dma_writer_request = writer > 0;
    m.dma_reader_release = 0;
    m.dma_writer_release = 0;
    checked_ioctl(ioctl_args(fd, LITEPCIE_IOCTL_LOCK, m));
    return m.dma_reader_status;
}

void litepcie_release_dma(file_t fd, uint8_t reader, uint8_t writer) {
    struct litepcie_ioctl_lock m;
    m.dma_reader_request = 0;
    m.dma_writer_request = 0;
    m.dma_reader_release = reader > 0;
    m.dma_writer_release = writer > 0;
    checked_ioctl(ioctl_args(fd, LITEPCIE_IOCTL_LOCK, m));
}

int litepcie_dma_init(struct litepcie_dma_ctrl *dma, const char *device_name, uint8_t zero_copy)
{
    int32_t flags = 0;
    dma->reader_hw_count = 0;
    dma->reader_sw_count = 0;
    dma->writer_hw_count = 0;
    dma->writer_sw_count = 0;

    dma->zero_copy = zero_copy;

#if defined(_WIN32)
    flags = (FILE_ATTRIBUTE_NORMAL |
             FILE_FLAG_NO_BUFFERING |
             FILE_FLAG_OVERLAPPED);
#else
    if (dma->use_reader)
        dma->fds.events |= POLLOUT;
    if (dma->use_writer)
        dma->fds.events |= POLLIN;
    flags = O_RDWR | O_CLOEXEC;
#endif

    dma->fds.fd = litepcie_open(device_name, flags);
    if (dma->fds.fd < 0) {
        fprintf(stderr, "Could not open device\n");
        return -1;
    }

    /* request dma reader and writer */
    if ((litepcie_request_dma(dma->fds.fd, dma->use_reader, dma->use_writer) == 0)) {
        fprintf(stderr, "DMA not available\n");
        return -1;
    }

    litepcie_dma_set_loopback(dma->fds.fd, dma->loopback);

    if (dma->zero_copy) {
#if defined(_WIN32)
        fprintf(stderr, "Zero Copy not available in Windows\n");
        return -1;
#else
        /* if mmap: get it from the kernel */
        checked_ioctl(ioctl_args(dma->fds.fd, LITEPCIE_IOCTL_MMAP_DMA_INFO, dma->mmap_dma_info));
        if (dma->use_writer) {
            dma->buf_rd = mmap(NULL, DMA_BUFFER_TOTAL_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                               dma->fds.fd, dma->mmap_dma_info.dma_rx_buf_offset);
            if (dma->buf_rd == MAP_FAILED) {
                fprintf(stderr, "MMAP failed\n");
                return -1;
            }
        }
        if (dma->use_reader) {
            dma->buf_wr = mmap(NULL, DMA_BUFFER_TOTAL_SIZE, PROT_WRITE, MAP_SHARED,
                               dma->fds.fd, dma->mmap_dma_info.dma_tx_buf_offset);
            if (dma->buf_wr == MAP_FAILED) {
                fprintf(stderr, "MMAP failed\n");
                return -1;
            }
        }
#endif
    } else {
        /* else: allocate it */
        if (dma->use_writer) {
            dma->buf_rd = calloc(1, DMA_BUFFER_TOTAL_SIZE);
            if (!dma->buf_rd) {
                fprintf(stderr, "%d: alloc failed\n", __LINE__);
                return -1;
            }
        }
        if (dma->use_reader) {
            dma->buf_wr = calloc(1, DMA_BUFFER_TOTAL_SIZE);
            if (!dma->buf_wr) {
                free(dma->buf_rd);
                fprintf(stderr, "%d: alloc failed\n", __LINE__);
                return -1;
            }
        }
    }

    return 0;
}

void litepcie_dma_cleanup(struct litepcie_dma_ctrl *dma)
{
    if (dma->use_reader)
        litepcie_dma_reader(dma->fds.fd, 0, &dma->reader_hw_count, &dma->reader_sw_count);
    if (dma->use_writer)
        litepcie_dma_writer(dma->fds.fd, 0, &dma->writer_hw_count, &dma->writer_sw_count);

    litepcie_release_dma(dma->fds.fd, dma->use_reader, dma->use_writer);

    if (dma->zero_copy) {
#if !defined(_WIN32)
        if (dma->use_reader)
            munmap(dma->buf_wr, dma->mmap_dma_info.dma_tx_buf_size * dma->mmap_dma_info.dma_tx_buf_count);
        if (dma->use_writer)
            munmap(dma->buf_rd, dma->mmap_dma_info.dma_tx_buf_size * dma->mmap_dma_info.dma_tx_buf_count);
#endif
    } else {
        free(dma->buf_rd);
        free(dma->buf_wr);
    }

    litepcie_close(dma->fds.fd);
}

void litepcie_dma_process(struct litepcie_dma_ctrl *dma)
{
    ssize_t len = 0;
    int32_t retVal;

    /* set / get dma */
    if (dma->use_writer)
        litepcie_dma_writer(dma->fds.fd, 1, &dma->writer_hw_count, &dma->writer_sw_count);
    if (dma->use_reader)
        litepcie_dma_reader(dma->fds.fd, 1, &dma->reader_hw_count, &dma->reader_sw_count);

#if defined(_WIN32)
    uint32_t retLen = 0;

    if (dma->zero_copy) {
        /* count available buffers */
        dma->buffers_available_write = (DMA_BUFFER_COUNT / 2) - (dma->reader_sw_count - dma->reader_hw_count);
        if (dma->buffers_available_write >= (DMA_BUFFER_COUNT / 2))
        {
            dma->buffers_available_write = DMA_BUFFER_COUNT / 2;
        }
        dma->usr_write_buf_offset = dma->reader_sw_count % DMA_BUFFER_COUNT;

        /* update dma sw_count */
        dma->mmap_dma_update.sw_count = dma->reader_sw_count + dma->buffers_available_write;
        checked_ioctl(dma->fds.fd, LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE,
            &dma->mmap_dma_update, sizeof(struct litepcie_ioctl_mmap_dma_update),
            &dma->mmap_dma_update, sizeof(struct litepcie_ioctl_mmap_dma_update), &retLen, 0);

        /* count available buffers */
        dma->buffers_available_read = dma->writer_hw_count - dma->writer_sw_count;
        dma->usr_read_buf_offset = dma->writer_sw_count % DMA_BUFFER_COUNT;

        /* update dma sw_count*/
        dma->mmap_dma_update.sw_count = dma->writer_sw_count + dma->buffers_available_read;
        checked_ioctl(dma->fds.fd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE,
            &dma->mmap_dma_update, sizeof(struct litepcie_ioctl_mmap_dma_update),
            &dma->mmap_dma_update, sizeof(struct litepcie_ioctl_mmap_dma_update), &retLen, 0);

    }
    else {
        OVERLAPPED writeData = { 0 };
        OVERLAPPED readData = { 0 };
        
        //Start Write
        dma->buffers_available_write = (dma->reader_hw_count - dma->reader_sw_count);
        if (dma->buffers_available_write >= (DMA_BUFFER_COUNT - DMA_BUFFER_PER_IRQ))
        {
            dma->buffers_available_write = DMA_BUFFER_COUNT - DMA_BUFFER_PER_IRQ;
        }
        if (dma->buffers_available_write > 1)
        {
            WriteFile(dma->fds.fd, dma->buf_wr, dma->buffers_available_write * DMA_BUFFER_SIZE, &retLen, &writeData);
        }

        //Start Read
        dma->buffers_available_read = dma->writer_hw_count - dma->writer_sw_count;
        if (dma->buffers_available_read >= (DMA_BUFFER_COUNT - DMA_BUFFER_PER_IRQ))
        {
            dma->buffers_available_read = DMA_BUFFER_COUNT - DMA_BUFFER_PER_IRQ;
        }
        if (dma->buffers_available_read > 1)
        {
            ReadFile(dma->fds.fd, dma->buf_rd, dma->buffers_available_read * DMA_BUFFER_SIZE, &retLen, &readData);
        }
        //Complete Read
        retLen = 0;
        if (dma->buffers_available_read > 1)
        {
            if (!GetOverlappedResult(dma->fds.fd, &readData, &retLen, TRUE))
            {
                fprintf(stderr, "Read failed: %d\n", GetLastError());
                fprintf(stderr, "Read args: 0x%p - 0x%lx - 0x%x\n", dma->buf_rd, dma->buffers_available_read, retLen);
                fprintf(stderr, "DMA Writer: 0x%llx - 0x%llx\n", dma->writer_hw_count, dma->writer_sw_count);
                litepcie_dma_cleanup(dma);
                abort();
            }
        }
        len = (ssize_t)retLen;
        dma->buffers_available_read = len / DMA_BUFFER_SIZE;
        dma->usr_read_buf_offset = 0;

        //Complete Write
        retLen = 0;
        if (dma->buffers_available_write > 1)
        {
            if (!GetOverlappedResult(dma->fds.fd, &writeData, &retLen, TRUE))
            {
                fprintf(stderr, "Write failed: %d\n", GetLastError());
                fprintf(stderr, "Write args: 0x%p - 0x%lx - %x\n", dma->buf_wr, dma->buffers_available_write, retLen);
                fprintf(stderr, "DMA Reader: 0x%llx - 0x%llx\n", dma->reader_hw_count, dma->reader_sw_count);
                litepcie_dma_cleanup(dma);
                abort();
            }
        }
        len = (ssize_t)retLen;
        dma->buffers_available_write = len / DMA_BUFFER_SIZE;
        dma->usr_write_buf_offset = 0;

    }
#else
    /* polling */
    retVal = poll(&dma->fds, 1, 100);
    if (retVal < 0) {
        perror("poll");
        return;
    } else if (retVal == 0) {
        /* timeout */
        return;
    }


    /* read event */
    if (dma->fds.revents & POLLIN) {
        if (dma->zero_copy) {
            /* count available buffers */
            dma->buffers_available_read = dma->writer_hw_count - dma->writer_sw_count;
            dma->usr_read_buf_offset = dma->writer_sw_count % DMA_BUFFER_COUNT;

            /* update dma sw_count*/
            dma->mmap_dma_update.sw_count = dma->writer_sw_count + dma->buffers_available_read;
            checked_ioctl(dma->fds.fd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &dma->mmap_dma_update);
        } else {
            len = read(dma->fds.fd, dma->buf_rd, DMA_BUFFER_TOTAL_SIZE);
            if (len < 0) {
                perror("read");
                abort();
            }
            dma->buffers_available_read = len / DMA_BUFFER_SIZE;
            dma->usr_read_buf_offset = 0;
        }
    } else {
        dma->buffers_available_read = 0;
    }

    /* write event */
    if (dma->fds.revents & POLLOUT) {
        if (dma->zero_copy) {
            /* count available buffers */
            dma->buffers_available_write = DMA_BUFFER_COUNT / 2 - (dma->reader_sw_count - dma->reader_hw_count);
            dma->usr_write_buf_offset = dma->reader_sw_count % DMA_BUFFER_COUNT;

            /* update dma sw_count */
            dma->mmap_dma_update.sw_count = dma->reader_sw_count + dma->buffers_available_write;
            checked_ioctl(dma->fds.fd, LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE, &dma->mmap_dma_update);

        } else {
            len = write(dma->fds.fd, dma->buf_wr, DMA_BUFFER_TOTAL_SIZE);
            if (len < 0) {
                perror("write");
                abort();
            }
            dma->buffers_available_write = len / DMA_BUFFER_SIZE;
            dma->usr_write_buf_offset = 0;
        }
    } else {
        dma->buffers_available_write = 0;
    }
#endif
}

char *litepcie_dma_next_read_buffer(struct litepcie_dma_ctrl *dma)
{
    if (!dma->buffers_available_read)
        return NULL;
    dma->buffers_available_read--;
    char *ret = dma->buf_rd + dma->usr_read_buf_offset * DMA_BUFFER_SIZE;
    dma->usr_read_buf_offset = (dma->usr_read_buf_offset + 1) % DMA_BUFFER_COUNT;
    return ret;
}

char *litepcie_dma_next_write_buffer(struct litepcie_dma_ctrl *dma)
{
    if (!dma->buffers_available_write)
        return NULL;
    dma->buffers_available_write--;
    char *ret = dma->buf_wr + dma->usr_write_buf_offset * DMA_BUFFER_SIZE;
    dma->usr_write_buf_offset = (dma->usr_write_buf_offset + 1) % DMA_BUFFER_COUNT;
    return ret;
}
