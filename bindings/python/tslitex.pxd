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

    cpdef enum tsChannelCoupling_e:
        TS_COUPLE_DC
        TS_COUPLE_AC

    ctypedef tsChannelCoupling_e tsChannelCoupling_t

    cpdef enum tsChannelTerm_e:
        TS_TERM_1M
        TS_TERM_50

    ctypedef tsChannelTerm_e tsChannelTerm_t

    cdef struct tsDeviceInfo_s:
        uint32_t device_id
        char device_path[256]
        char identity[256]
        char serial_number[256]

    ctypedef tsDeviceInfo_s tsDeviceInfo_t

    cdef struct tsChannelParam_s:
        uint32_t volt_scale_mV
        int32_t volt_offset_mV
        uint32_t bandwidth
        uint8_t coupling
        uint8_t term
        uint8_t active
        uint8_t reserved

    ctypedef tsChannelParam_s tsChannelParam_t

    cdef struct _syshealth:
        uint32_t temp_c
        uint32_t vcc_int
        uint32_t vcc_aux
        uint32_t vcc_bram

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

    ctypedef tsScopeState_s tsScopeState_t

    int32_t thunderscopeListDevices(uint32_t devIndex, tsDeviceInfo_t* info)

    tsHandle_t thunderscopeOpen(uint32_t devIdx)

    int32_t thunderscopeClose(tsHandle_t ts)

    int32_t thunderscopeChannelConfigGet(tsHandle_t ts, uint32_t channel, tsChannelParam_t* conf)

    int32_t thunderscopeChannelConfigSet(tsHandle_t ts, uint32_t channel, tsChannelParam_t* conf)

    int32_t thunderscopeStatusGet(tsHandle_t ts, tsScopeState_t* conf)

    int32_t thunderscopeSampleModeSet(tsHandle_t ts, uint32_t rate, uint32_t resolution)

    int32_t thunderscopeDataEnable(tsHandle_t ts, uint8_t enable)

    int32_t thunderscopeRead(tsHandle_t ts, uint8_t* buffer, uint32_t len)

    int32_t thunderscopeFwUpdate(tsHandle_t ts, char* bitstream, uint32_t len)
