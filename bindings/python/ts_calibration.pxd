# cython: embedsignature=True, language_level=3
# SPDX-License-Identifier: BSD-2-Clause
#
# This file is part of libtslitex.
# Mapped definitions of ts_calibration.h
#
# Copyright (C) 2025 / Nate Meyer  / nate.devel@gmail.com

from libc.stdint cimport uint32_t, int32_t, uint8_t, int8_t, uint16_t


cdef extern from "ts_calibration.h":

    ctypedef void* tsHandle_t

    cdef enum:
        TS_CAL_NUM_PATHS = 11
        TS_CAL_NUM_LOADS = 11
        TS_CAL_NUM_RATES = 8

    cdef enum tsChannelsActive_e:
        TS_CHAN_NONE = 0b0000
        TS_CHAN_0 = 0b0001
        TS_CHAN_1 = 0b0010
        TS_CHAN_0_1 = 0b0011
        TS_CHAN_2 = 0b0100
        TS_CHAN_0_2 = 0b0101
        TS_CHAN_1_2 = 0b0110
        TS_CHAN_3 = 0b1000
        TS_CHAN_0_3 = 0b1001
        TS_CHAN_1_3 = 0b1010
        TS_CHAN_2_3 = 0b1100
        TS_CHAN_0_1_2_3 = 0b1111

    ctypedef tsChannelsActive_e tsChannelsActive_t

    cdef struct tsAfePathCalibration_s:
        double bufferInputVpp
        double trimOffsetDacZeroC
        double trimOffsetDacZeroM
        double trimOffsetDacScale
        uint32_t trimDPot

    ctypedef tsAfePathCalibration_s tsAfePathCalibration_t

    cdef struct tsAfeCalibration_s:
        double attenuatorScale
        tsAfePathCalibration_t highPgaPathCal[TS_CAL_NUM_PATHS]
        tsAfePathCalibration_t lowPgaPathCal[TS_CAL_NUM_PATHS]

    ctypedef tsAfeCalibration_s tsChannelCalibration_t

    cdef struct tsAdcLoad_s:
        uint32_t rate
        double[4] scale

    ctypedef tsAdcLoad_s tsAdcLoad_t

    cdef struct tsAdcLoadCalibration_s:
        tsChannelsActive_t channels
        tsAdcLoad_t conf[TS_CAL_NUM_RATES]

    ctypedef tsAdcLoadCalibration_s tsAdcLoadCalibration_t

    cdef struct tsAdcGain_s:
        uint32_t rate
        int8_t[8] gain


    cdef struct tsAdcBranchGain_s:
        tsChannelsActive_t channels
        tsAdcGain_s conf[TS_CAL_NUM_RATES]

    ctypedef tsAdcBranchGain_s tsAdcBranchGain_t

    cdef struct tsAdcCalibration_s:
        tsAdcLoadCalibration_t loadCal[TS_CAL_NUM_LOADS]
        tsAdcBranchGain_t branchFineGain[TS_CAL_NUM_LOADS]

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

    int32_t thunderscopeChanCalibrationSet(tsHandle_t ts, uint32_t channel, tsChannelCalibration_t* cal)

    int32_t thunderscopeChanCalibrationGet(tsHandle_t ts, uint32_t channel, tsChannelCalibration_t* cal)

    int32_t thunderscopeAdcCalibrationSet(tsHandle_t ts, tsAdcCalibration_t* cal)

    int32_t thunderscopeAdcCalibrationGet(tsHandle_t ts, tsAdcCalibration_t* cal)

    int32_t thunderscopeCalibrationManualCtrl(tsHandle_t ts, uint32_t channel, tsChannelCtrl_t* ctrl)
