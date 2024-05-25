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

#ifdef __cplusplus
}
#endif

#endif
