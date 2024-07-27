/* SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of libtslitex.
 * Digital Pot Configuration for the MCP443X
 *
 * Copyright (C) 2024 / Nate Meyer  / nate.devel@gmail.com
 *
 */

#ifndef _MCP443X_H_
#define _MCP443X_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"
#include "ts_common.h"


#define MCP4432_NUM_CH  (4)

#define MCP4432_MIN     (0)
#define MCP4432_MAX     (1 << 7)
#define MCP4432_DEFAULT (0x40)

#define MCP4432_503_INC         (50000 / MCP4432_MAX)
#define MCP4432_503_SET(x)      ((x) / MCP4432_503_INC)
#define MCP4432_503_OHM(x)      ((x) * MCP4432_503_INC)

int32_t mcp443x_set_wiper(i2c_t dev, uint8_t channel, uint8_t val);


#ifdef __cplusplus
}
#endif
#endif
