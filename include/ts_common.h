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

#define TS_IDENT_STR_LEN            (256)

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

typedef enum tsChannelTerm_e
{
    TS_TERM_1M = 0,
    TS_TERM_50 = 1,
} tsChannelTerm_t;

typedef struct tsDeviceInfo_s
{
    uint32_t deviceID;
    char devicePath[TS_IDENT_STR_LEN];
    char identity[TS_IDENT_STR_LEN];
    char serialNumber[TS_IDENT_STR_LEN];
} tsDeviceInfo_t;

typedef struct tsChannelParam_s
{
    uint32_t volt_scale_mV;     /**< Set full scale voltage in millivolts */
    int32_t volt_offset_mV;     /**< Set offset voltage in millivolts */
    uint32_t bandwidth;         /**< Set Bandwidth Filter in MHz. Next highest filter will be selected */
    uint8_t coupling;           /**< Select AD/DC coupling for channel.  Use tsChannelCoupling_t enum */
    uint8_t term;               /**< Select Termination mode for channel.  Use tsChannelTerm_t enum */
    uint8_t active;             /**< Active flag for the channel. 1 to enable, 0 to disable */
    uint8_t reserved;           /**< Reserved byte for 32-bit alignment*/
} tsChannelParam_t;

typedef struct tsScopeState_s
{
    uint32_t adc_sample_rate;           /**< Samples per Second captured by the ADC */
    uint32_t adc_sample_bits;           /**< Number of bits used to represent each sample in the data stream.  
                                             This indicates how to read the sample stream, i.e. how many bits wide each sample is. */
    uint32_t adc_sample_resolution;     /**< Maximum Sample value. 2^n bits as captured by the ADC for each sample */
    uint32_t adc_lost_buffer_count;     /**< Buffers dropped that weren't read by the application */
    union {
        uint32_t flags;
        struct {
            uint8_t adc_state:1;
            uint8_t power_state:1;
            uint8_t pll_state:1;
            uint8_t afe_state:1;
        };
    };
    struct {
        uint32_t temp_c;
        uint32_t vcc_int;
        uint32_t vcc_aux;
        uint32_t vcc_bram;
    } sys_health;
} tsScopeState_t;


typedef struct ts_afe_cal_s
{
    uint32_t buffer_mv;
    uint32_t bias_mv;
}ts_afe_cal_t;

#ifdef __cplusplus
}
#endif
#endif