/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Access the Sample Data stream from the Thunderscope
 * LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */


#include "ts_common.h"
#include "liblitepcie.h"

/**
 * @brief The sample buffers should be read in increments of this size so the kernel
 * drivers can manage buffers appropriately.
 */
#define TS_SAMPLE_BUFFER_SIZE   DMA_BUFFER_SIZE


typedef struct sampleStream_s
{
    file_t dma;
    int64_t dma_buffer_count;
    int64_t driver_buffer_count;
    uint8_t active;
} sampleStream_t;

/**
 * @brief Initialize the sample engine for reading samples
 * 
 * @param inst Sample Stream Instance pointer
 * @param devIdx Device Index
 * @param channel DMA Channel to read
 * @return int32_t TS_STATUS_OK if the DMA was initialized, else TS_STATUS_ERROR
 */
int32_t samples_init(sampleStream_t* inst, uint8_t devIdx, uint8_t channel);

/**
 * @brief Enable or Disable the sample engine
 * 
 * @param inst Sample Stream Instance pointer
 * @param en 1 to start the sample stream, 0 to stop
 * @return int32_t TS_STATUS_OK
 */
int32_t samples_enable_set(sampleStream_t* inst, uint8_t en);

/**
 * @brief Read sample data from the sample engine.
 *  Length must be a multiple of TS_SAMPLE_BUFFER_SIZE.
 * 
 * @param inst Sample Stream Instance pointer
 * @param sampleBuffer Byte buffer to read samples into
 * @param bufferLen Length of the sample buffer to read
 * @return int32_t Number of bytes read, or a negative error value
 */
int32_t samples_get_buffers(sampleStream_t* inst, uint8_t* sampleBuffer, uint32_t bufferLen);

/**
 * @brief Shutdown and Close the sample engine
 * 
 * @param inst Sample Stream Instance pointer
 * @return int32_t TS_STATUS_OK if successful
 */
int32_t samples_teardown(sampleStream_t* inst);