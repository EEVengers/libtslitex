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
#include <stdbool.h>

#define TS_STATUS_OK                (0)
#define TS_STATUS_ERROR             (-1)


#define TS_NUM_CHANNELS             (4)

/**
 * @brief Opaque Handle to a Thunderscope device instance
 *  
 */
typedef void* tsHandle_t;


typedef enum tsChannelCoupling_e
{
    TS_COUPLE_DC = 0,
    TS_COUPLE_AC = 1
} tsChannelCoupling_t;

typedef struct tsChannelParam_s
{
    bool active;
    uint32_t volt_scale;
    uint32_t bandwidth;
    tsChannelCoupling_t coupling;
    bool attenuation;
} tsChannelParam_t;

typedef struct tsChannelConfig_s
{
    tsChannelParam_t channels[TS_NUM_CHANNELS];
    uint32_t adc_sample_rate;
} tsChannelConfig_t;

#ifdef __cplusplus
}
#endif
#endif