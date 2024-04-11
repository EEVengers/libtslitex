/* SPDX-License-Identifier: BSD-2-Clause
 *
 * LitePCIe library
 *
 * This file is part of LitePCIe.
 *
 * Copyright (C) 2018-2023 / EnjoyDigital  / florent@enjoy-digital.fr
 *
 */

#if defined(_WIN32)
#include <Windows.h>
#include <ioapiset.h>
#include <SetupAPI.h>
#include <INITGUID.H>
#else
#include <sys/ioctl.h>
#include <unistd.h>
#endif

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <litepcie_public.h>
#include "litepcie_helpers.h"


 //Find Devices
static void getDeviceName(PWCHAR devName, DWORD maxLen)
{
    DWORD detailLen = 0;
    DWORD dataSize = 0;
    SP_DEVICE_INTERFACE_DATA devData;
    PSP_DEVICE_INTERFACE_DETAIL_DATA pDetail;
    HDEVINFO hwDevInfo = SetupDiGetClassDevs(&GUID_DEVINTERFACE_litepciedrv, NULL, NULL, DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);

    devData.cbSize = sizeof(devData);
    if (!SetupDiEnumDeviceInterfaces(hwDevInfo, NULL, &GUID_DEVINTERFACE_litepciedrv, 0, &devData))
    {
        //Print Error
        fprintf(stderr, "No Devices Found\n");
        goto cleanup;
    }

    SetupDiGetDeviceInterfaceDetail(hwDevInfo, &devData, NULL, 0, &detailLen, NULL);
    if (detailLen <= 0)
    {
        //Print Error
        fprintf(stderr, "Bad length for device\n");
        goto cleanup;
    }

    dataSize = detailLen + sizeof(DWORD);
    pDetail = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(dataSize);

    if (pDetail == NULL)
    {
        fprintf(stderr, "Failed to allocate detail buffer, length %d\n", detailLen);
        goto cleanup;
    }

    memset(pDetail, 0x00, dataSize);
    pDetail->cbSize = sizeof(SP_INTERFACE_DEVICE_DETAIL_DATA);

    if (SetupDiGetDeviceInterfaceDetail(hwDevInfo, &devData, pDetail, detailLen, NULL, NULL))
    {
        wcsncpy_s(devName, maxLen, pDetail->DevicePath, _TRUNCATE);
        fwprintf(stdout, L"Found device: %s\n", pDetail->DevicePath);
    }
    else
    {
        fprintf(stderr, "Failed to retrieve device detail\n");
    }

    free(pDetail);

cleanup:
    SetupDiDestroyDeviceInfoList(hwDevInfo);
    return;
}


uint32_t litepcie_readl(file_t fd, uint32_t addr) {
    struct litepcie_ioctl_reg regData = { 0 };
    DWORD len = 0;

    regData.reg = addr;
    regData.is_write = 0;
    checked_ioctl(fd, LITEPCIE_IOCTL_REG,
        &regData, sizeof(struct litepcie_ioctl_reg),
        &regData, sizeof(struct litepcie_ioctl_reg), &len, 0);
    if (len != sizeof(struct litepcie_ioctl_reg))
    {
        fprintf(stderr, "read_reg returned bad len data. %d\n", len);
    }
    return regData.val;
}

void litepcie_writel(file_t fd, uint32_t addr, uint32_t val) {
    struct litepcie_ioctl_reg regData;
    DWORD len = 0;

    regData.reg = addr;
    regData.val = val;
    regData.is_write = 1;
    checked_ioctl(fd, LITEPCIE_IOCTL_REG,
        &regData, sizeof(struct litepcie_ioctl_reg),
        NULL, 0, &len, 0);
}

void litepcie_reload(file_t fd) {
    struct litepcie_ioctl_icap m;
    m.addr = 0x4;
    m.data = 0xf;
    DWORD len = 0;

    checked_ioctl(fd, LITEPCIE_IOCTL_ICAP,
        &m, sizeof(struct litepcie_ioctl_reg),
        NULL, 0, NULL, 0);
}

void _check_ioctl(bool status, const char* file, int line)
{
    if (status)
    {
#if defined(_WIN32)
        fprintf(stderr, "Failed ioctl at %s:%d: %d\n", file, line, GetLastError());
#else
        fprintf(stderr, "Failed ioctl at %s:%d: %s\n", file, line, strerror(errno));
#endif
        abort();
    }
}

file_t litepcie_open(const char* name, int32_t flags)
{
    file_t fd;
    /* Open LitePCIe device. */
    WCHAR devName[1024] = { 0 };
    getDeviceName(devName, 1024);
    UINT32 devLen = lstrlenW(devName);
    mbstowcs(&devName[devLen], name, 1024-devLen);
    fd = CreateFile(devName, (GENERIC_READ | GENERIC_WRITE), 0, NULL,
        OPEN_EXISTING, flags, NULL);
    return fd;
}

void litepcie_close(file_t fd)
{
    CloseHandle(fd);
}