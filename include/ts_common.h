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

#include <stdint.h>

#define TS_STATUS_OK                (0)
#define TS_STATUS_ERROR             (-1)
#define TS_INVALID_PARAM            (-2)


#if defined(_WIN32)
#include <windows.h>
typedef HANDLE tsHandle_t;
#else
typedef int32_t tsHandle_t;
#endif

#ifdef __cplusplus
}
#endif
#endif