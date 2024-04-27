/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Channel management functions for the Thunderscope
 * LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _TS_CHANNEL_H_
#define _TS_CHANNEL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ts_common.h"

typedef void* tsChannelHdl_t;

/**
 * @brief Initialize all Channel AFEs
 * 
 * @param pTsChannels Thunderscope Channel handle
 * @param ts    Thunderscope device handle
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
int32_t ts_channel_init(tsChannelHdl_t* pTsChannels, tsHandle_t ts);

/**
 * @brief Tear down and clean up Thunderscope Channel object(s)
 * 
 * @param tsChannels Thunderscope Channel handle
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
int32_t ts_channel_destroy(tsChannelHdl_t tsChannels);

#ifdef __cplusplus
}
#endif
#endif