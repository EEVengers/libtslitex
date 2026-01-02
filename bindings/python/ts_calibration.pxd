# cython: embedsignature=True, language_level=3
# SPDX-License-Identifier: BSD-2-Clause
#
# This file is part of libtslitex.
# Mapped definitions of ts_calibration.h
#
# Copyright (C) 2025 / Nate Meyer  / nate.devel@gmail.com

from libc.stdint cimport uint32_t, int32_t, uint8_t, uint16_t

cdef extern from "ts_calibration.h":

    ctypedef void* tsHandle_t

    cdef struct tsChannelCalibration_s:
        int32_t buffer_uV
        int32_t bias_uV
        int32_t attenuatorGain1M_mdB
        int32_t attenuatorGain50_mdB
        int32_t bufferGain_mdB
        int32_t trimRheostat_range
        int32_t preampLowGainError_mdB
        int32_t preampHighGainError_mdB
        int32_t preampAttenuatorGain_mdB[11]
        int32_t preampOutputGainError_mdB
        int32_t preampLowOffset_uV
        int32_t preampHighOffset_uV
        int32_t preampInputBias_uA

    ctypedef tsChannelCalibration_s tsChannelCalibration_t

    cdef struct tsAdcCalibration_s:
        uint8_t branchFineGain[8]

    ctypedef tsAdcCalibration_s tsAdcCalibration_t

    cdef struct tsChannelCtrl_s:
        uint8_t atten
        uint8_t term
        uint8_t dc_couple
        uint8_t dpot
        uint16_t dac
        uint8_t pga_high_gain
        uint8_t pga_atten
        uint8_t pga_bw

    ctypedef tsChannelCtrl_s tsChannelCtrl_t

    cdef struct tsScopeCalibration_s:
        tsChannelCalibration_t afeCal[4]
        tsAdcCalibration_t adcCal

    ctypedef tsScopeCalibration_s tsScopeCalibration_t

    int32_t thunderscopeChanCalibrationSet(tsHandle_t ts, uint32_t channel, tsChannelCalibration_t* cal)

    int32_t thunderscopeChanCalibrationGet(tsHandle_t ts, uint32_t channel, tsChannelCalibration_t* cal)

    int32_t thunderscopeAdcCalibrationSet(tsHandle_t ts, tsAdcCalibration_t* cal)

    int32_t thunderscopeAdcCalibrationGet(tsHandle_t ts, tsAdcCalibration_t* cal)

    int32_t thunderscopeCalibrationManualCtrl(tsHandle_t ts, uint32_t channel, tsChannelCtrl_t* ctrl)
