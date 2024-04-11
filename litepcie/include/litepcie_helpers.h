/* SPDX-License-Identifier: BSD-2-Clause
 *
 * LitePCIe library
 *
 * This file is part of LitePCIe.
 *
 * Copyright (C) 2018-2023 / EnjoyDigital  / florent@enjoy-digital.fr
 *
 */

#ifndef LITEPCIE_LIB_HELPERS_H
#define LITEPCIE_LIB_HELPERS_H

#include <stdint.h>
#include <stdbool.h>

#if defined(_WIN32)
#include <ioapiset.h>
typedef HANDLE file_t;
//IOCTL Args: HANDLE fd, DWORD dwIoControlCode, PVOID lpInBuffer, DWORD nInBufferSize,
//				PVOID lpOutBuffer, DWORD nOutBufferSize, PDWORD lpOutBytesReturned,
//				POVERLAPPED lpOverlapped
#define checked_ioctl(...) _check_ioctl(!DeviceIoControl(__VA_ARGS__), __FILE__, __LINE__)
void _check_ioctl(bool status, const char* file, int line);
#else
#include <sys/ioctl.h>
typedef int file_t
#define checked_ioctl(...) _check_ioctl(ioctl(__VA_ARGS__), __FILE__, __LINE__) 
void _check_ioctl(file_t status, const char *file, int line);
#endif

uint32_t litepcie_readl(file_t fd, uint32_t addr);
void litepcie_writel(file_t fd, uint32_t addr, uint32_t val);
void litepcie_reload(file_t fd);

file_t litepcie_open(const char* name, int32_t flags);

void litepcie_close(file_t fd);

#endif /* LITEPCIE_LIB_HELPERS_H */
