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
#define TS_INVALID_PARAM            (-2)


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

typedef enum tsChannelTerm_e {
    TS_TERM_1M = 0,
    TS_TERM_50 = 1,
} tsChannelTerm_t;

typedef struct tsChannelParam_s
{
    uint32_t volt_scale_mV;     /**< Set full scale voltage in millivolts */
    uint32_t volt_offset_mV;    /**< Set offset voltage in millivolts */
    uint32_t bandwidth;         /**< Set Bandwidth Filter in MHz. Next highest filter will be selected */
    uint8_t coupling;           /**< Select AD/DC coupling for channel.  Use tsChannelCoupling_t enum */
    uint8_t term;               /**< Select Termination mode for channel.  Use tsChannelTerm_t enum */
    uint8_t active;             /**< Active flag for the channel. 1 to enable, 0 to disable */
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