# cython: embedsignature=True, language_level=3
# SPDX-License-Identifier: BSD-2-Clause
#
# This file is part of libtslitex.
# Provide a wrapper class for the tslitex Thunderscope library
#
# Copyright (C) 2025 / Nate Meyer  / nate.devel@gmail.com

cimport cython
cimport tslitex

cdef ThunderscopeListDevs(devIdx:int):
    cdef tslitex.tsDeviceInfo_t infos
    retVal = tslitex.thunderscopeListDevices(devIdx, &infos)
    return retVal, infos


cdef class Thunderscope:
    cdef tslitex.tsHandle_t tsHandle
    def __init__(self, dev_idx: int):
        self.tsHandle = <tslitex.tsHandle_t> tslitex.thunderscopeOpen(dev_idx)
        if self.tsHandle == NULL:
            raise ValueError(f"Failed to Open Thunderscope Device {dev_idx}", dev_idx)

    def __del__(self):
        if self.tsHandle != NULL:
            tslitex.thunderscopeClose(self.tsHandle)
            self.tsHandle = NULL

    def Status(self):
        cdef tslitex.tsScopeState_t status_vals
        if self.tsHandle != NULL:
            tslitex.thunderscopeStatusGet(<tslitex.tsHandle_t>self.tsHandle, &status_vals)
            return status_vals

