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

#define TS_MAX_8BIT_SAMPLE_RATE     (1000000000)
#define TS_MAX_12BIT_SAMPLE_RATE    (660000000)
#define TS_MAX_14BIT_SAMPLE_RATE    (125000000)
#define TS_MIN_SAMPLE_RATE          (15000000)

#define TS_HW_ID_REV_MASK           (7)
#define TS_HW_ID_VARIANT_MASK       (1 << 8)
#define TS_HW_ID_VALID_MASK         (1 << 9)

#define TS_GW_VERSION(major, minor, patch)  ((((major) & 0xFFFF) << 16) + \
                                             (((minor) & 0xFF)   << 8) + \
                                             (((patch) & 0x3F)   << 1))

#define LITEX_VERSION(major, minor)  ((((major) & 0xFFFF) << 16) + ((minor) & 0xFFFF))
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
    uint32_t device_id;
    uint32_t hw_id;   /**< hw_id[9] - ID Valid, hw_id[8] - PCIe/USB, hw_id[7:4] - Reserved, hw_id[3:0] - Revision */
    uint32_t gw_id;  /**< 32-bit version ID: gw_id[31:16] - Major, gw_id[15:8] - Minor, gw_id[7:1] - Patch, gw_id[0] - next */
    uint32_t litex; /**< 32-bit LiteX version: litex[31:16] - Year, litex[15:0] - Month */
    char device_path[TS_IDENT_STR_LEN];
    char identity[TS_IDENT_STR_LEN];
    char serial_number[TS_IDENT_STR_LEN];
} tsDeviceInfo_t;

typedef struct tsChannelParam_s
{
    uint32_t volt_scale_uV;     /**< Set full scale voltage in microvolts */
    int32_t volt_offset_uV;     /**< Set offset voltage in microvolts */
    uint32_t bandwidth;         /**< Set Bandwidth Filter in MHz. Next highest filter will be selected */
    uint8_t coupling;           /**< Select AD/DC coupling for channel.  Use tsChannelCoupling_t enum */
    uint8_t term;               /**< Select Termination mode for channel.  Use tsChannelTerm_t enum */
    uint8_t active;             /**< Active flag for the channel. 1 to enable, 0 to disable */
    uint8_t reserved;           /**< Reserved byte for 32-bit alignment*/
} tsChannelParam_t;

typedef struct sysHealth_s {
    uint32_t temp_c;
    uint32_t vcc_int;
    uint32_t vcc_aux;
    uint32_t vcc_bram;
    uint8_t frontend_power_good;
    uint8_t acq_power_good;
} sysHealth_t;

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
            uint8_t adc_sync:1;
            uint8_t power_state:1;
            uint8_t pll_state:1;
            uint8_t afe_state:1;
        };
    };
    sysHealth_t sys_health;
} tsScopeState_t;


#ifdef __cplusplus
}
#endif
#endif