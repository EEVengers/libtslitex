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
#elif defined(__APPLE__)
#include <IOKit/IOKitLib.h>
#include <IOKit/IOReturn.h>
#include <sys/ioctl.h>
#else
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#endif

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include "litepcie_helpers.h"
#include "litepcie.h"


#if defined (_WIN32)
//Find Devices
static void getDeviceName(PWCHAR devName, DWORD maxLen, DWORD devIdx)
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
#elif defined(__APPLE__)

void _print_kerr_details(kern_return_t ret)
{
    printf("\tSystem: 0x%02x\n", err_get_system(ret));
    printf("\tSubsystem: 0x%03x\n", err_get_sub(ret));
    printf("\tCode: 0x%04x\n", err_get_code(ret));
}
#endif

uint32_t litepcie_readl(file_t fd, uint32_t addr) {
#if defined(__APPLE__)
    kern_return_t ret = kIOReturnSuccess;

    uint32_t olen = 1;
    uint64_t output = 0;
    uint64_t input = addr;
    
    ret = IOConnectCallScalarMethod(fd, LITEPCIE_READ_CSR, &input, 1, &output, &olen);
    
    if (ret != kIOReturnSuccess) {
        printf("LITEPCIE_READ_CSR failed with error: 0x%08x.\n", ret);
        _print_kerr_details(ret);
    }
    
    return (uint32_t)output;
#else
    struct litepcie_ioctl_reg regData = { 0 };

    regData.addr = addr;
    regData.is_write = 0;
    checked_ioctl(ioctl_args(fd, LITEPCIE_IOCTL_REG, regData));
    return regData.val;
#endif
}

void litepcie_writel(file_t fd, uint32_t addr, uint32_t val) {
#if defined(__APPLE__)
    kern_return_t ret = kIOReturnSuccess;

    uint32_t olen = 0;
    uint64_t input[2] = { addr, val };
    ret = IOConnectCallScalarMethod(fd, LITEPCIE_WRITE_CSR, input, 2, NULL, &olen);
    
    if (ret != kIOReturnSuccess) {
        printf("LITEPCIE_WRITE_CSR failed with error: 0x%08x.\n", ret);
        _print_kerr_details(ret);
    }
#else
    struct litepcie_ioctl_reg regData;

    regData.addr = addr;
    regData.val = val;
    regData.is_write = 1;
    checked_ioctl(ioctl_args(fd, LITEPCIE_IOCTL_REG, regData));
#endif
}

void litepcie_reload(file_t fd) {
    struct litepcie_ioctl_icap m;
    m.addr = 0x4;
    m.data = 0xf;
#if defined(__APPLE__)
    size_t outlen = sizeof(struct litepcie_ioctl_icap);
#endif

    checked_ioctl(ioctl_args(fd, LITEPCIE_IOCTL_ICAP, m));
}

void _check_ioctl(int status, const char *file, int line)
{
    if (status)
    {
#if defined(_WIN32)
        fprintf(stderr, "Failed ioctl at %s:%d: %d\n", file, line, GetLastError());

#elif defined(__APPLE__)
        fprintf(stderr, "Failed ioctl at %s:%d: %08x\n", file, line, status);
#else
        fprintf(stderr, "Failed ioctl at %s:%d: %s\n", file, line, strerror(errno));
#endif
        abort();
    }
}

file_t litepcie_open(const char* name, int32_t flags)
{
    file_t fd;
#if defined(_WIN32)
    /* Open LitePCIe device. */
    wchar_t devName[1024] = { 0 };
    uint32_t devNum = atoi(&name[strlen(name) - 1]);
    getDeviceName(devName, 1024, devNum);
    uint32_t devLen = lstrlenW(devName);
    mbstowcs(&devName[devLen], name, 1024-devLen);
    devName[devLen + strlen(name) - 1] = '\0';
    fd = CreateFile(devName, (GENERIC_READ | GENERIC_WRITE), 0, NULL,
        OPEN_EXISTING, flags, NULL);
#elif defined(__APPLE__)
    kern_return_t ret = kIOReturnSuccess;
    io_iterator_t iterator = IO_OBJECT_NULL;
    io_service_t service = IO_OBJECT_NULL;
    io_connect_t connection = IO_OBJECT_NULL;
    fd = IO_OBJECT_NULL;

    /// - Tag: ClientApp_Connect
    ret = IOServiceGetMatchingServices(kIOMainPortDefault, IOServiceNameMatching(name), &iterator);
    if (ret != kIOReturnSuccess) {
        printf("Unable to find service for identifier with error: 0x%08x.\n", ret);
        _print_kerr_details(ret);
    }
    else
    {
        printf("Searching for dext service...\n");
        while ((service = IOIteratorNext(iterator)) != IO_OBJECT_NULL) {
            // Open a connection to this user client as a server to that client, and store the instance in "service"
            ret = IOServiceOpen(service, mach_task_self_, kIOHIDServerConnectType, &connection);

            if (ret == kIOReturnSuccess) {
                printf("\tOpened service.\n");
                fd = connection;
                break;
            } else {
                printf("\tFailed opening service with error: 0x%08x.\n", ret);
            }

            IOObjectRelease(service);
        }
    }
    IOObjectRelease(iterator);

    if (service == IO_OBJECT_NULL) {
        printf("Failed to match to device.\n");
    }
#else
    fd = open(name, flags);
#endif
    return fd;
}

void litepcie_close(file_t fd)
{
#if defined(_WIN32)
    CloseHandle(fd);
#elif defined(__APPLE__)
    IOServiceClose(fd);
#else
    close(fd);
#endif
}
