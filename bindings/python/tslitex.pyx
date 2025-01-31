# cython: embedsignature=True, language_level=3
# SPDX-License-Identifier: BSD-2-Clause
#
# This file is part of libtslitex.
# Provide a wrapper class for the tslitex Thunderscope library
#
# Copyright (C) 2025 / Nate Meyer  / nate.devel@gmail.com

cimport cython
cimport tslitex
import numpy

def ThunderscopeListDevs(devIdx:int):
    cdef tslitex.tsDeviceInfo_t infos
    retVal = tslitex.thunderscopeListDevices(devIdx, &infos)
    return retVal, infos


cdef class Channel:
    cdef tslitex.tsChannelParam_t _params
    cdef uint32_t _channel
    cdef tslitex.tsHandle_t dev

    def __init__(self, chan):
        if chan not in range(0,4):
            raise ValueError(f"Invalid channel {chan}")
        self._channel = chan

    @staticmethod
    cdef create(tslitex.tsHandle_t ptr, ch: int):
        if ptr == NULL:
            raise ValueError(f"Invalid device handle for Channel")
        p = Channel(ch)
        p.dev = ptr
        tslitex.thunderscopeChannelConfigGet(<tslitex.tsHandle_t>p.dev, <uint32_t>ch, &p._params)
        return p

    @property
    def Active(self):
        cdef int32_t retVal = tslitex.thunderscopeChannelConfigGet(self.dev, self._channel, &self._params)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Failed to retrieve Channel {self._channel} parameters")
        return self._params.active

    
    @Active.setter
    def Active(self, enable: bool):
        self._params.active = <uint8_t>enable
        cdef int32_t retVal
        retVal = tslitex.thunderscopeChannelConfigSet(self.dev, self._channel, &self._params)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Failed to set Channel {self._channel} active parameter")

    @property
    def VoltScale(self):
        cdef int32_t retVal = tslitex.thunderscopeChannelConfigGet(self.dev, self._channel, &self._params)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Failed to retrieve Channel {self._channel} parameters")
        return <float>self._params.volt_scale_mV / 1000.0

    
    @VoltScale.setter
    def VoltScale(self, volts: float):
        self._params.volt_scale_mV = <uint32_t>(volts*1000)
        cdef int32_t retVal
        retVal = tslitex.thunderscopeChannelConfigSet(self.dev, self._channel, &self._params)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Failed to set Channel {self._channel} voltage scale parameter")

    @property
    def VoltOffset(self):
        cdef int32_t retVal = tslitex.thunderscopeChannelConfigGet(self.dev, self._channel, &self._params)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Failed to retrieve Channel {self._channel} parameters")
        return <float>self._params.volt_offset_mV / 1000.0

    
    @VoltOffset.setter
    def VoltOffset(self, volts: float):
        self._params.volt_offset_mV = <uint32_t>(volts*1000)
        cdef int32_t retVal
        retVal = tslitex.thunderscopeChannelConfigSet(self.dev, self._channel, &self._params)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Failed to set Channel {self._channel} voltage offset parameter")

    @property
    def Bandwidth(self):
        cdef int32_t retVal = tslitex.thunderscopeChannelConfigGet(self.dev, self._channel, &self._params)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Failed to retrieve Channel {self._channel} parameters")
        return self._params.bandwidth

    
    @Bandwidth.setter
    def Bandwidth(self, bandwidth: int):
        self._params.bandwidth = <uint32_t>bandwidth
        cdef int32_t retVal
        retVal = tslitex.thunderscopeChannelConfigSet(self.dev, self._channel, &self._params)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Failed to set Channel {self._channel} bandwidth parameter")

    @property
    def Coupling(self):
        cdef int32_t retVal = tslitex.thunderscopeChannelConfigGet(self.dev, self._channel, &self._params)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Failed to retrieve Channel {self._channel} parameters")
        return self._params.coupling

    
    @Coupling.setter
    def Coupling(self, coupling: tslitex.tsChannelCoupling_t):
        if coupling not in [tslitex.TS_COUPLE_AC, tslitex.TS_COUPLE_DC]:
            raise ValueError(f"Invalid value for channel coupling")
        self._params.coupling = <uint8_t>coupling
        cdef int32_t retVal
        retVal = tslitex.thunderscopeChannelConfigSet(self.dev, self._channel, &self._params)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Failed to set Channel {self._channel} coupling parameter")

    @property
    def Termination(self):
        cdef int32_t retVal = tslitex.thunderscopeChannelConfigGet(self.dev, self._channel, &self._params)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Failed to retrieve Channel {self._channel} parameters")
        return self._params.termination

    
    @Termination.setter
    def Termination(self, term: tslitex.tsChannelTerm_t):
        if term not in [tslitex.TS_TERM_1M, tslitex.TS_TERM_50]:
            raise ValueError(f"Invalid value for channel termination")
        self._params.term = term
        cdef int32_t retVal
        retVal = tslitex.thunderscopeChannelConfigSet(self.dev, self._channel, &self._params)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Failed to set Channel {self._channel} termination parameter")

cdef class Thunderscope:
    cdef uint32_t _sample_rate
    cdef uint32_t _sample_mode
    cdef uint8_t _enable
    cdef tslitex.tsHandle_t _tsHandle
    cdef public object channel
    
    def __cinit__(self, dev_idx: int):
        self.channel = []

    def __init__(self, dev_idx: int):
        self._sample_rate = 1000000000
        self._sample_mode = 256
        self._enable = 0
        self._tsHandle = <tslitex.tsHandle_t> tslitex.thunderscopeOpen(dev_idx)
        if self._tsHandle == NULL:
            raise ValueError(f"Failed to Open Thunderscope Device {dev_idx}", dev_idx)
        for ch in range(4):
            self.channel.append(Channel.create(self._tsHandle, ch))
        
    def __del__(self):
        if self._tsHandle != NULL:
            tslitex.thunderscopeClose(self._tsHandle)
            self._tsHandle = NULL

    def Status(self):
        cdef tslitex.tsScopeState_t status_vals
        if self._tsHandle != NULL:
            tslitex.thunderscopeStatusGet(<tslitex.tsHandle_t>self._tsHandle, &status_vals)
            return status_vals

    def firmwareUpdate(self, bitfile):
        if type(bitfile) is not bytes:
            raise TypeError(f"bitfile arg must be 'bytes' type")
        cdef uint32_t file_len = len(bitfile)
        cdef char* pFile = bitfile
        return tslitex.thunderscopeFwUpdate(self._tsHandle, pFile, file_len)

    @property
    def SampleRate(self):
        return self._sample_rate

    @SampleRate.setter
    def SampleRate(self, rate: int):
        self._sample_rate = rate
        retval = <int32_t> tslitex.thunderscopeSampleModeSet(<tslitex.tsHandle_t>self._tsHandle,
                                                            <uint32_t>self._sample_rate, <uint32_t>self._sample_mode)

    @property
    def SampleResolution(self):
        return self._sample_resolution

    @SampleResolution.setter
    def SampleResolution(self, resolution: int):
        assert resolution is 256, f"Support for 8-bit mode only"
        self._sample_resolution = resolution
        retval = <int32_t> tslitex.thunderscopeSampleModeSet(<tslitex.tsHandle_t>self._tsHandle,
                                                            <uint32_t>self._sample_rate, <uint32_t>self._sample_resolution)

    def Enable(self, enable: bool):
        cdef int32_t retVal = tslitex.thunderscopeDataEnable(self._tsHandle, <uint8_t>enable)
        if retVal != tslitex.TS_STATUS_OK:
            raise ValueError(f"Unable to set Thunderscope Enable to {enable}")

    def Read(self, uint8_t[:] data not None, dataLen: int):
        readLen = tslitex.thunderscopeRead(self._tsHandle, &data[0], dataLen)
        return readLen

