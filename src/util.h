/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Miscellaneous helper functions.
 * 
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 */

#ifndef _UTIL_H_
#define _UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>
#include <stdio.h>

#define NANOSECOND	(1000000000)
#define NS_DELAY(ns)    { \
                        struct timespec start, now; \
                        timespec_get(&start, TIME_UTC); \
                        do{ \
                            timespec_get(&now, TIME_UTC); \
                            if((((int64_t)(now.tv_sec - start.tv_sec) * NANOSECOND) \
                                +(now.tv_nsec - start.tv_nsec)) \
                                >= (ns)) { break; } \
                        } while(1);}

#ifdef EN_LOGGING
#define LOG_ERROR(...)      fprintf(stderr, "[%s:%d] %s\r\n", __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOG_ERROR(...)
#endif

#ifdef EN_LOGGING_DEBUG
#define LOG_DEBUG(...)      fprintf(stdout, "[%s:%d] %s\r\n", __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOG_DEBUG(...)
#endif

#ifdef __cplusplus
}
#endif

#endif
