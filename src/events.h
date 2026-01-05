/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Event Subsystem for the Thunderscope LiteX design
 *
 * Copyright (C) 2025 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#ifndef EVENTS_H_
#define EVENTS_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "ts_common.h"
#include "liblitepcie.h"

#define TS_BYTES_PER_SAMPLE_COUNT   (128/8) // 128-bit sample bus

/**
 * @brief Set Initial conditions of the Event Controls
 * 
 * @param handle File handle
 * @return int32_t TS_STATUS_OK if the Event controller is initialized successfully
 */
int32_t events_initialize(file_t handle);

/**
 * @brief Flush any pending events in the FIFO
 * 
 * @param handle File handle
 * @return int32_t TS_STATUS_OK if the Event controller is flushed successfully
 */
int32_t events_flush(file_t handle);

/**
 * @brief Check status bit if an Event is pending in the queue
 * 
 * @param handle File handle
 * @return bool True if an event can be read, False otherwise
 */
bool events_available(file_t handle);

/**
 * @brief Pull an event off the queue
 * 
 * @param handle File handle
 * @param pEvent Pointer to an event structure to populate
 * @return int32_t TS_STATUS_OK if the event was retrieved successfully
 */
int32_t events_get_next(file_t handle, tsEvent_t *pEvent);

/**
 * @brief Configure the Event Generator to produce a timed event.  The event will repeat with the
 * given period.
 * 
 * @param handle File handle
 * @param period_us Event period (microseconds)
 * @return int32_t TS_STATUS_OK if the event was configured successfully
 */
int32_t events_set_periodic(file_t handle, uint32_t period_us);

/**
 * @brief Immediately trigger an event with the Event Generator
 * 
 * @param handle File handle
 * @return int32_t TS_STATUS_OK if the event was configured successfully
 */
int32_t events_set_immediate(file_t handle);

/**
 * @brief Retrieve the current state of the Event Source Inputs to the Event Engine
 * 
 * @param handle File handle
 * @return uint32_t Value of the Source Input status
 */
uint32_t events_get_source_status(file_t handle);

/**
 * @brief Configure the External Sync Pin direction
 * 
 * @param handle File handle
 * @param mode Sync I/O Mode
 * @return int32_t TS_STATUS_OK if the External Sync Pin was configured successfully
 */
int32_t events_set_ext_sync(file_t handle, tsSyncMode_t mode);



#ifdef __cplusplus
}
#endif

#endif
