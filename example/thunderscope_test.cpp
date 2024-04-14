/**
 * This file is part of libtslitex.
 *
 * Copyright (c) 2020-2021 Florent Kermarrec <florent@enjoy-digital.fr>
 * Copyright (c) 2022 Franck Jullien <franck.jullien@collshade.fr>
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 * Copyright (c) 2024 John Simons <jammsimons@gmail.com>
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>

#include <chrono>
#include <iostream>
#include <thread>


#ifdef _WIN32
#include <Windows.h>
#endif

#include "liblitepcie.h"

// Test low-level library functions
#include "../src/spi.h"
#include "../src/i2c.h"
#include "../src/gpio.h"

#if !defined(_WIN32)
#define INVALID_HANDLE_VALUE (-1)
#endif


#ifdef _WIN32
#define FILE_FLAGS  (FILE_ATTRIBUTE_NORMAL)
#else
#define FILE_FLAGS  (O_RDWR)
#endif


/* Parameters */
/*------------*/

/* Variables */
/*-----------*/

static int litepcie_device_num;

uint32_t AFE_CONTROL_LDO_EN = (1 << 0);
uint32_t AFE_CONTROL_COUPLING = (1 << 8);
uint32_t AFE_CONTROL_ATTENUATION = (1 << 16);

uint32_t AFE_STATUS_LDO_PWR_GOOD = (1 << 0);

uint32_t AFE_AC_COUPLING = 0;
uint32_t AFE_DC_COUPLING = 1;

uint32_t AFE_1X_ATTENUATION = 0;
uint32_t AFE_10X_ATTENUATION = 1;

// ADC Constants------------------------------------------------------------------------------------

uint32_t ADC_CONTROL_LDO_EN = (1 << 0);
uint32_t ADC_CONTROL_PLL_EN = (1 << 1);
uint32_t ADC_CONTROL_RST = (1 << 2);
uint32_t ADC_CONTROL_PWR_DOWN = (1 << 3);

uint32_t ADC_STATUS_LDO_PWR_GOOD = (1 << 0);

uint32_t _SPI_CONTROL_START = (1 << 0);
uint32_t _SPI_CONTROL_LENGTH = (1 << 8);
uint32_t _SPI_STATUS_DONE = (1 << 0);


/* Main */
/*------*/

void configure_frontend_ldo(file_t fd, uint32_t enable) {
    uint32_t control_value = litepcie_readl(fd, CSR_FRONTEND_CONTROL_ADDR);
    control_value &= ~(1 * AFE_CONTROL_LDO_EN);
    control_value |= (enable * AFE_CONTROL_LDO_EN);
    litepcie_writel(fd, CSR_FRONTEND_CONTROL_ADDR, control_value);
}

void configure_adc_ldo(file_t fd, uint32_t enable) {
    uint32_t control_value = litepcie_readl(fd, CSR_ADC_CONTROL_ADDR);
    control_value &= ~(1 * ADC_CONTROL_LDO_EN);
    control_value |= (enable * ADC_CONTROL_LDO_EN);
    litepcie_writel(fd, CSR_ADC_CONTROL_ADDR, control_value);
}

void configure_pll_en(file_t fd, uint32_t enable) {
    uint32_t control_value = litepcie_readl(fd, CSR_ADC_CONTROL_ADDR);
    control_value &= ~(1 * ADC_CONTROL_PLL_EN);
    control_value |= (enable * ADC_CONTROL_PLL_EN);
    litepcie_writel(fd, CSR_ADC_CONTROL_ADDR, control_value);
}

void control_led(file_t fd, uint32_t enable) {
    uint32_t control_value = litepcie_readl(fd, CSR_LEDS_OUT_ADDR);
    control_value &= ~(1 * AFE_STATUS_LDO_PWR_GOOD);
    control_value |= (enable * AFE_STATUS_LDO_PWR_GOOD);
    litepcie_writel(fd, CSR_LEDS_OUT_ADDR, control_value);
}


// Functioning returning 
// current time
auto now()
{
    return std::chrono::steady_clock::now();
}

// Function calculating sleep time 
// with 500ms delay
auto awake_time()
{
    using std::chrono::operator"" ms;
    return now() + 500ms;
}


/* Main */
/*------*/

int main(int argc, char** argv)
{
    const char* cmd = argv[0];
    static uint8_t litepcie_device_zero_copy;
    static uint8_t litepcie_device_external_loopback;
    static int litepcie_data_width;
    static int litepcie_auto_rx_delay;

    litepcie_device_num = 0;
    litepcie_data_width = 16;
    litepcie_auto_rx_delay = 0;
    litepcie_device_zero_copy = 0;
    litepcie_device_external_loopback = 0;
    file_t fd;
    int i;
    unsigned char fpga_identifier[256];
    fd = litepcie_open(LITEPCIE_CTRL_NAME(0), FILE_FLAGS);
    if (fd == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }


    printf("\x1b[1m[> FPGA/SoC Information:\x1b[0m\n");
    printf("------------------------\n");

    for (i = 0; i < 256; i++)
    {
        fpga_identifier[i] = litepcie_readl(fd, CSR_IDENTIFIER_MEM_BASE + 4 * i);
    }
    printf("FPGA Identifier:  %s.\n", fpga_identifier);

#ifdef CSR_DNA_BASE
    printf("FPGA DNA:         0x%08x%08x\n",
        litepcie_readl(fd, CSR_DNA_ID_ADDR + 4 * 0),
        litepcie_readl(fd, CSR_DNA_ID_ADDR + 4 * 1));
#endif
#ifdef CSR_XADC_BASE
    printf("FPGA Temperature: %0.1f �C\n",
        (double)litepcie_readl(fd, CSR_XADC_TEMPERATURE_ADDR) * 503.975 / 4096 - 273.15);
    printf("FPGA VCC-INT:     %0.2f V\n",
        (double)litepcie_readl(fd, CSR_XADC_VCCINT_ADDR) / 4096 * 3);
    printf("FPGA VCC-AUX:     %0.2f V\n",
        (double)litepcie_readl(fd, CSR_XADC_VCCAUX_ADDR) / 4096 * 3);
    printf("FPGA VCC-BRAM:    %0.2f V\n",
        (double)litepcie_readl(fd, CSR_XADC_VCCBRAM_ADDR) / 4096 * 3);
#endif


    printf("\x1b[1m[> Scratch register test:\x1b[0m\n");
    printf("-------------------------\n");


    /* Write to scratch register. */
    printf("Write 0x12345678 to Scratch register:\n");
    litepcie_writel(fd, CSR_CTRL_SCRATCH_ADDR, 0x0);
    printf("Read: 0x%08x\n", litepcie_readl(fd, CSR_CTRL_SCRATCH_ADDR));

    /* Read from scratch register. */
    printf("Write 0xdeadbeef to Scratch register:\n");
    litepcie_writel(fd, CSR_CTRL_SCRATCH_ADDR, 0xdeadbeef);
    printf("Read: 0x%08x\n", litepcie_readl(fd, CSR_CTRL_SCRATCH_ADDR));

    printf("Enabling frontend LDO:\n");
    configure_frontend_ldo(fd, 1);

    printf("Enabling ADC LDO:\n");
    configure_adc_ldo(fd, 1);

    printf("Disabling PLL EN & waiting 500ms:\n");
    configure_pll_en(fd, 0);

    std::this_thread::sleep_until(awake_time());

    printf("Enabling PLL EN:\n");
    configure_pll_en(fd, 1);

    i2c_t i2cDev;
    i2cDev.fd = fd;

    for (unsigned char addr = 0; addr < 0x80; addr++) {
        i2cDev.devAddr = addr;
        bool result = i2c_poll(i2cDev);
        if (addr % 0x10 == 0) {
            printf("\n0x%02X", addr);
        }
        if (result) {
            printf(" %02X", addr);
        }
        else {
            printf(" --");
        }
    }

    spi_bus_t spimaster;
    spi_bus_init(&spimaster, fd, CSR_MAIN_SPI_BASE, CSR_MAIN_SPI_CS_SEL_SIZE);

    uint8_t data[2] = {0x01, 0x02};

    for (int i = 0; i < CSR_MAIN_SPI_CS_SEL_SIZE; i++) {
        spi_dev_t spiDev;
        spi_dev_init(&spiDev, &spimaster, i);
        for (int reg = 0; reg < 10; reg++) {
            spi_write(spiDev, reg, data, 2);
            spi_busy_wait(spiDev);
        }
    }

    /* Close LitePCIe device. */
    litepcie_close(fd);

    return 0;
}
