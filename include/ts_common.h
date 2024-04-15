/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Library configuration definitions
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _TS_COMMON_H_
#define _TS_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TS_STATUS_OK                (0)
#define TS_STATUS_ERROR             (-1)

typedef void* tsHandle_t;

#ifdef __cplusplus
}
#endif
#endif