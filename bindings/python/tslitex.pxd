# cython: embedsignature=True, language_level=3
# SPDX-License-Identifier: BSD-2-Clause
#
# This file is part of libtslitex.
# Mapped definitions of thunderscope.h
#
# Copyright (C) 2025 / Nate Meyer  / nate.devel@gmail.com

from libc.stdint cimport uint32_t, int32_t, uint8_t

cdef extern from "thunderscope.h":

    ctypedef void* tsHandle_t

    cdef enum:
        TS_STATUS_OK
        TS_STATUS_ERROR
        TS_INVALID_PARAM

    cpdef enum tsChannelCoupling_e:
        TS_COUPLE_DC
        TS_COUPLE_AC

    ctypedef tsChannelCoupling_e tsChannelCoupling_t

    cpdef enum tsChannelTerm_e:
        TS_TERM_1M
        TS_TERM_50

    ctypedef tsChannelTerm_e tsChannelTerm_t

    cdef enum:
        TS_MODE_8_BIT = 256

    cdef struct tsDeviceInfo_s:
        uint32_t device_id
        uint32_t hw_id
        uint32_t gw_id
        char device_path[256]
        char identity[256]
        char serial_number[256]

    ctypedef tsDeviceInfo_s tsDeviceInfo_t

    cdef struct tsChannelParam_s:
        uint32_t volt_scale_uV
        int32_t volt_offset_uV
        uint32_t bandwidth
        uint8_t coupling
        uint8_t term
        uint8_t active
        uint8_t reserved

    ctypedef tsChannelParam_s tsChannelParam_t

    cdef struct sysHealth_s:
        uint32_t temp_c
        uint32_t vcc_int
        uint32_t vcc_aux
        uint32_t vcc_bram
        uint8_t frontend_power_good
        uint8_t acq_power_good

    ctypedef sysHealth_s sysHealth_t

    cdef struct tsScopeState_s:
        uint32_t adc_sample_rate
        uint32_t adc_sample_bits
        uint32_t adc_sample_resolution
        uint32_t adc_lost_buffer_count
        uint32_t flags
        uint8_t adc_state
        uint8_t power_state
        uint8_t pll_state
        uint8_t afe_state
        sysHealth_t sys_health

    ctypedef tsScopeState_s tsScopeState_t

    int32_t thunderscopeListDevices(uint32_t devIndex, tsDeviceInfo_t* info)

    tsHandle_t thunderscopeOpen(uint32_t devIdx, bint skip_init)

    int32_t thunderscopeClose(tsHandle_t ts)

    int32_t thunderscopeChannelConfigGet(tsHandle_t ts, uint32_t channel, tsChannelParam_t* conf)

    int32_t thunderscopeChannelConfigSet(tsHandle_t ts, uint32_t channel, tsChannelParam_t* conf)

    int32_t thunderscopeStatusGet(tsHandle_t ts, tsScopeState_t* conf)

    int32_t thunderscopeSampleModeSet(tsHandle_t ts, uint32_t rate, uint32_t resolution)

    int32_t thunderscopeDataEnable(tsHandle_t ts, uint8_t enable)

    int32_t thunderscopeRead(tsHandle_t ts, uint8_t* buffer, uint32_t len) nogil

    int32_t thunderscopeFwUpdate(tsHandle_t ts, const char* bitstream, uint32_t len) nogil

    int32_t thunderscopeUserDataRead(tsHandle_t ts, char* buffer, uint32_t offset, uint32_t readLen) nogil
    
    int32_t thunderscopeUserDataWrite(tsHandle_t ts, const char* buffer, uint32_t offset, uint32_t writeLen) nogil

    int32_t thunderscopeGetFwProgress(tsHandle_t ts, uint32_t* progress)
