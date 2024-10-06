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
#include <fstream>
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

#define OPTPARSE_IMPLEMENTATION
#include "optparse.h"

#include "AudioFile.h"

#include "liblitepcie.h"

// Test low-level library functions
#include "../src/spi.h"
#include "../src/i2c.h"
#include "../src/gpio.h"
#include "../src/util.h"
#include "../src/hmcad15xx.h"

#include "../src/ts_channel.h"
#include "../src/samples.h"

#include "thunderscope.h"

#ifdef _WIN32
#define FILE_FLAGS  (FILE_ATTRIBUTE_NORMAL)
#else
#define FILE_FLAGS  (O_RDWR)
#endif


/* Parameters */
/*------------*/
#define TS_TEST_SAMPLE_FILE     "test_data.bin"
#define TS_TEST_WAV_FILE        "test_data.wav"

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

static void test_io(file_t fd)
{
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
}

static void test_capture(file_t fd, uint32_t idx, uint8_t channelBitmap, uint16_t bandwidth, 
    uint32_t volt_scale_mV, int32_t offset_mV, uint8_t ac_couple, uint8_t term)
{
    uint8_t numChan = 0;
    tsHandle_t tsHdl = thunderscopeOpen(idx);

    // tsChannelHdl_t channels;
    // ts_channel_init(&channels, fd);
    // if(channels == NULL)
    // {
    //     printf("Failed to create channels handle");
    //     return;
    // }

    // sampleStream_t samp;
    // samples_init(&samp, 0, 0);

    uint8_t* sampleBuffer = (uint8_t*)calloc(TS_SAMPLE_BUFFER_SIZE * 0x8000, 1);
    uint64_t sampleLen = 0;

    //Setup and Enable Channels
    tsChannelParam_t chConfig = {0};
    uint8_t channel = 0;
    while(channelBitmap > 0)
    {
        if(channelBitmap & 0x1)
        {
            thunderscopeChannelConfigGet(tsHdl, channel, &chConfig);
            chConfig.volt_scale_mV = volt_scale_mV;
            chConfig.volt_offset_mV = offset_mV;
            chConfig.bandwidth = bandwidth;
            chConfig.coupling = ac_couple ? TS_COUPLE_AC : TS_COUPLE_DC;
            chConfig.term =  term ? TS_TERM_50 : TS_TERM_1M;
            chConfig.active = 1;
            // ts_channel_params_set(channels, channel, &chConfig);
            thunderscopeChannelConfigSet(tsHdl, channel, &chConfig);
            numChan++;
        }
        channel++;
        channelBitmap >>= 1;
    }

    // Uncomment to use Test Pattern
    // ts_channel_set_adc_test(channels, HMCAD15_TEST_SYNC, 0, 0);

    printf("- Checking HMCAD1520 Sample Rate...");
    litepcie_writel(fd, CSR_ADC_HAD1511_CONTROL_ADDR, 1 << CSR_ADC_HAD1511_CONTROL_STAT_RST_OFFSET);
    NS_DELAY(500000000);
    uint32_t rate = litepcie_readl(fd, CSR_ADC_HAD1511_SAMPLE_COUNT_ADDR) * 2;
    printf(" %d Samples/S\r\n", rate);


    //Only start taking samples if the rate is non-zero
    if(rate > 0)
    {
        uint64_t data_sum = 0;
        //Start Sample capture
        // samples_enable_set(&samp, 1);
        // ts_channel_run(channels, 1);
        thunderscopeDataEnable(tsHdl, 1);
        
        auto startTime = std::chrono::steady_clock::now();
        if(sampleBuffer != NULL)
        {
            for(uint32_t loop=0; loop < 100; loop++)
            {
                uint32_t readReq = (TS_SAMPLE_BUFFER_SIZE * 0x1000);
                //Collect Samples
                // int32_t readRes = samples_get_buffers(&samp, sampleBuffer, readReq);
                int32_t readRes = thunderscopeRead(tsHdl, sampleBuffer, readReq);
                if(readRes < 0)
                {
                    printf("ERROR: Sample Get Buffers failed with %" PRIi32, readRes);
                }
                if(readRes != readReq)
                {
                    printf("WARN: Read returned different number of bytes for loop %" PRIu32 ", %" PRIu32 " / %" PRIu32 "\r\n", loop, readRes, readReq);
                }
                data_sum += readReq;
                sampleLen = readRes;
            }
        }
        auto endTime = std::chrono::steady_clock::now();

        //Stop Samples
        // samples_enable_set(&samp, 0);
        // ts_channel_run(channels, 0);
        thunderscopeDataEnable(tsHdl, 0);
        
        auto deltaNs = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
        uint64_t bw = (data_sum * 8 * 1000)/deltaNs.count();
        printf("Collected %" PRIu64 " samples in %" PRIu64 " Mbps\r\n", data_sum, bw);
    }

    //Disable channels
    for(uint8_t i=0; i < TS_NUM_CHANNELS; i++)
    {
        // ts_channel_params_get(channels, i, &chConfig);
        thunderscopeChannelConfigGet(tsHdl, i, &chConfig);
        chConfig.active = 0;
        // ts_channel_params_set(channels, i, &chConfig);
        thunderscopeChannelConfigSet(tsHdl, i, &chConfig);
    }

    // ts_channel_destroy(channels);
    // samples_teardown(&samp);
    thunderscopeClose(tsHdl);

    if(sampleLen > 0)
    {
        auto outFile = std::fstream(TS_TEST_SAMPLE_FILE, std::ios::out | std::ios::binary | std::ios::trunc);
        outFile.write(reinterpret_cast<const char*>(const_cast<const uint8_t*>(sampleBuffer)), sampleLen);
        outFile.flush();
        outFile.close();
        
        AudioFile<uint8_t> outWav;
        outWav.setBitDepth(8);
        outWav.setSampleRate(1000000000/numChan);
        outWav.setNumChannels(numChan);

        AudioFile<uint8_t>::AudioBuffer wavBuffer;
        wavBuffer.resize(numChan);
        wavBuffer[0].resize(sampleLen/numChan);
        if(numChan > 1)
        {
            wavBuffer[1].resize(sampleLen/numChan);
        }
        if(numChan > 2)
        {
            wavBuffer[2].resize(sampleLen/numChan);
            wavBuffer[3].resize(sampleLen/numChan);
        }
        uint64_t sample = 0;
        uint64_t idx = 0;
        while (idx < sampleLen)
        {
            wavBuffer[0][sample] = sampleBuffer[idx++];
            if(numChan > 1)
            {
                wavBuffer[1][sample] = sampleBuffer[idx++];
                if(numChan > 2)
                {
                    wavBuffer[2][sample] = sampleBuffer[idx++];
                    wavBuffer[3][sample] = sampleBuffer[idx++];
                }
            }
            sample++;
        }
        outWav.setAudioBuffer(wavBuffer);
        outWav.printSummary();
        outWav.save(TS_TEST_WAV_FILE);
    }
}

static void print_help(void)
{
    printf("TS Test Util Usage:\r\n");
    printf("\t io - run I/O Test\r\n");
    printf("\t capture - run Sample Capture Test\r\n");
    printf("\t\t -d <device>      Device Index\r\n");
    printf("\t\t -c <channels>    Channel bitmap\r\n");
    printf("\t\t -b <bw>          Channel Bandwidth [MHz]\r\n");
    printf("\t\t -v <mvolts>      Channel Full Scale Volts [millivolt]\r\n");
    printf("\t\t -o <mvolts>      Channel Offset [millivolt]\r\n");
    printf("\t\t -a               AC Couple\r\n");
    printf("\t\t -t               50 Ohm termination\r\n");
}

/* Main */
/*------*/

int main(int argc, char** argv)
{
    const char* cmd = argv[0];
    unsigned char fpga_identifier[256];
    char devicePath[TS_IDENT_STR_LEN];


    file_t fd;
    uint32_t idx = 0;
    int i;
    uint8_t channelBitmap = 0x0F;
    uint16_t bandwidth = 350;
    uint32_t volt_scale_mV = 10000;
    int32_t offset_mV = 0;
    uint8_t ac_couple = 0;
    uint8_t term = 0;

    struct optparse_long argList[] = {
        {"dev",      'd', OPTPARSE_REQUIRED},
        {"chan",     'c', OPTPARSE_REQUIRED},
        {"bw",       'b', OPTPARSE_REQUIRED},
        {"voltsmv",  'v', OPTPARSE_REQUIRED},
        {"offsetmv", 'o', OPTPARSE_REQUIRED},
        {"ac",       'a', OPTPARSE_NONE},
        {"term",     't', OPTPARSE_NONE},
        {0}
    };

    auto argCount = 1;
    char *arg = argv[argCount];
    int option;
    struct optparse options;

    (void)argc;
    optparse_init(&options, argv);
    while ((option = optparse_long(&options, argList, NULL)) != -1)
    {
        switch (option) {
        case 'b':
            bandwidth = strtol(options.optarg, NULL, 0);
            argCount+=2;
            break;
        case 'c':
            channelBitmap = strtol(options.optarg, NULL, 0);
            argCount+=2;
            break;
        case 'd':
            idx = strtol(options.optarg, NULL, 0);
            argCount+=2;
            break;
        case 'v':
            volt_scale_mV = strtol(options.optarg, NULL, 0);
            argCount+=2;
            break;
        case 'o':
            offset_mV = strtol(options.optarg, NULL, 0);
            argCount+=2;
            break;
        case 'a':
            ac_couple = 1;
            argCount++;
            break;
        case 't':
            term = 1;
            argCount++;
            break;
        case '?':
            fprintf(stderr, "%s: %s\n", argv[0], options.errmsg);
            print_help();
            exit(EXIT_FAILURE);
        }
    }
    arg = argv[argCount];

    if(argCount == argc)
    {
        print_help();
        exit(EXIT_FAILURE);
    }


    snprintf(devicePath, TS_IDENT_STR_LEN, LITEPCIE_CTRL_NAME(%d), idx);
    printf("Opening Device %s\n", devicePath);
    fd = litepcie_open((const char*)devicePath, FILE_FLAGS);
    if(fd == INVALID_HANDLE_VALUE) {
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

    // Run Example IO
    if(0 == strcmp(arg, "io"))
    {
        test_io(fd);
    }
    // Setup Channel, record samples to buffer, save buffer to file
    else if(0 == strcmp(arg, "capture"))
    {
        test_capture(fd, idx, channelBitmap, bandwidth, volt_scale_mV, offset_mV, ac_couple, term);
    }
    //Print Help
    else
    {
        print_help();
    }

    /* Close LitePCIe device. */
    litepcie_close(fd);

    return 0;
}
