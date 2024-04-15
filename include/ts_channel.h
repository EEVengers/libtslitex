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

#include "ts_common.h"

/**
 * @brief Initialize all Channel AFEs
 * 
 * @param ts Thunderscope device handle
 * @return int32_t TS_STATUS_OK on success, else TS_STATUS_ERROR
 */
int32_t ts_channel_init_all(tsHandle_t ts);


#ifdef __cplusplus
}
#endif
#endif