/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Data Control and Management functions for the
 * Thunderscope LiteX design
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */
#ifndef _THUNDERSCOPE_H_
#define _THUNDERSCOPE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "ts_common.h"

int32_t thunderscopeInit(void);
int32_t thunderscopeListDevices(void);
int32_t thunderscopeOpen(void);

#ifdef __cplusplus
}
#endif
#endif