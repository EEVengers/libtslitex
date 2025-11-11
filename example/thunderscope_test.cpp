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
#else
#include <csignal>
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
#include "../src/mcp_zl3026x.h"
#include "../src/mcp_clkgen.h"
#include "../src/spiflash.h"

#include "../src/ts_channel.h"
#include "../src/samples.h"
#include "../src/platform.h"

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
#define TS_FLASH_DUMP_FILE      "flash_dump.bin"

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

static volatile bool g_program_loop = true;

/* Main */
/*------*/

#ifdef _WIN32
BOOL WINAPI SigHandler(DWORD ctrlType)
{
    if(ctrlType == CTRL_C_EVENT)
    {
        g_program_loop = false;
        return TRUE;
    }
    return FALSE;
}
#else
extern "C" void SigHandler(int s)
{
    g_program_loop = false;
}
#endif

void configure_frontend_ldo(file_t fd, uint32_t enable) {
    uint32_t control_value = litepcie_readl(fd, CSR_FRONTEND_CONTROL_ADDR);
    control_value &= ~(1 * AFE_CONTROL_LDO_EN);
    control_value |= (enable * AFE_CONTROL_LDO_EN);
    litepcie_writel(fd, CSR_FRONTEND_CONTROL_ADDR, control_value);
}

bool query_frontend_pg(file_t fd) {
    uint32_t status_value = litepcie_readl(fd, CSR_FRONTEND_STATUS_ADDR);
    return (status_value & (1 << CSR_FRONTEND_STATUS_FE_PG_OFFSET));
}

void configure_adc_ldo(file_t fd, uint32_t enable) {
    uint32_t control_value = litepcie_readl(fd, CSR_ADC_CONTROL_ADDR);
    control_value &= ~(1 * ADC_CONTROL_LDO_EN);
    control_value |= (enable * ADC_CONTROL_LDO_EN);
    litepcie_writel(fd, CSR_ADC_CONTROL_ADDR, control_value);
}

bool query_adc_pg(file_t fd) {
    uint32_t status_value = litepcie_readl(fd, CSR_ADC_STATUS_ADDR);
    return (status_value & (1 << CSR_ADC_STATUS_ACQ_PG_OFFSET));
}

void configure_pll_en(file_t fd, uint32_t enable) {
    uint32_t control_value = litepcie_readl(fd, CSR_ADC_CONTROL_ADDR);
    control_value &= ~(1 * ADC_CONTROL_PLL_EN);
    control_value |= (enable * ADC_CONTROL_PLL_EN);
    litepcie_writel(fd, CSR_ADC_CONTROL_ADDR, control_value);
}

void control_led(file_t fd, uint8_t val) {
    litepcie_writel(fd, CSR_DEV_STATUS_LEDS_ADDR, (uint32_t)val);
}

static uint32_t read_flash_word(file_t fd, uint32_t flash_addr)
{
    const uint32_t flash_base = 0x10000;
    uint32_t flash_data = 0;
    uint32_t flash_window = flash_addr >> 16;
    litepcie_writel(fd, CSR_FLASH_ADAPTER_WINDOW0_ADDR, flash_window);
    flash_data = litepcie_readl(fd, (flash_base + (flash_addr & 0xFFFF)));
    return flash_data;
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

static void test_io(file_t fd, bool isBeta)
{
    printf("\x1b[1m[> Scratch register test:\x1b[0m\n");
    printf("-------------------------\n");


    /* Write to scratch register. */
    printf("Write 0x12345678 to Scratch register:\n");
    litepcie_writel(fd, CSR_CTRL_SCRATCH_ADDR, 0x12345678);
    printf("Read: 0x%08x\n", litepcie_readl(fd, CSR_CTRL_SCRATCH_ADDR));

    /* Read from scratch register. */
    printf("Write 0xdeadbeef to Scratch register:\n");
    litepcie_writel(fd, CSR_CTRL_SCRATCH_ADDR, 0xdeadbeef);
    printf("Read: 0x%08x\n", litepcie_readl(fd, CSR_CTRL_SCRATCH_ADDR));

    printf("Enabling frontend LDO:\n");
    configure_frontend_ldo(fd, 1);

    std::this_thread::sleep_until(awake_time());
    if(query_frontend_pg(fd))
    {
        printf("\tFrontend Power Good\n");
    }

    printf("Enabling ADC LDO:\n");
    configure_adc_ldo(fd, 1);

    std::this_thread::sleep_until(awake_time());
    if(query_adc_pg(fd))
    {
        printf("\tADC Power Good\n");
    }

    printf("Disabling PLL EN & waiting 500ms:\n");
    configure_pll_en(fd, 0);

    std::this_thread::sleep_until(awake_time());

    printf("Enabling PLL EN:\n");
    configure_pll_en(fd, 1);

    //Cycle LEDs
    const led_signals_t *leds;
    if(isBeta)
    {
        leds = &ts_beta_leds;
    }
    else
    {
        leds = &ts_dev_leds;
    }
    
    printf("Set READY LED\n");
    control_led(fd, leds->ready);
    using std::chrono::operator"" s;
    std::this_thread::sleep_for(3s);
    printf("Set ERROR LED\n");
    control_led(fd, leds->error);
    using std::chrono::operator"" s;
    std::this_thread::sleep_for(3s);
    printf("Set ACTIVE LED\n");
    control_led(fd, leds->active);
    using std::chrono::operator"" s;
    std::this_thread::sleep_for(3s);

    control_led(fd, leds->disabled);

    i2c_t i2cDev;
    i2cDev.fd = fd;
    i2cDev.peripheral_baseaddr = CSR_I2CBUS_I2C0_PHY_SPEED_MODE_ADDR;
    i2c_rate_set(i2cDev, I2C_400KHz);
    printf("\nScan I2C Bus 0:\n");
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
    
    if(!isBeta)
    {
        i2cDev.peripheral_baseaddr = CSR_I2CBUS_I2C1_PHY_SPEED_MODE_ADDR;
        i2c_rate_set(i2cDev, I2C_400KHz);
        printf("\nScan I2C Bus 1:\n");
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
    }

    spi_bus_t spibus0;
    spi_bus_init(&spibus0, fd, CSR_SPIBUS_BASE, CSR_SPIBUS_SPI0_CS_SEL_SIZE);

    uint8_t data[2] = {0x01, 0x02};

    for (int i = 0; i < CSR_SPIBUS_SPI0_CS_SEL_SIZE; i++) {
        spi_dev_t spiDev;
        spi_dev_init(&spiDev, &spibus0, i);
        for (int reg = 0; reg < 10; reg++) {
            spi_write(spiDev, reg, data, 2);
            spi_busy_wait(spiDev);
        }
    }

    if(!isBeta)
    {
        spi_bus_t spibus1;
        spi_bus_init(&spibus1, fd, CSR_SPIBUS_SPI1_CONTROL_ADDR, CSR_SPIBUS_SPI1_CS_SEL_SIZE);

        uint8_t data[2] = {0x01, 0x02};

        for (int i = 0; i < CSR_SPIBUS_SPI1_CS_SEL_SIZE; i++) {
            spi_dev_t spiDev;
            spi_dev_init(&spiDev, &spibus1, i);
            for (int reg = 0; reg < 10; reg++) {
                spi_write(spiDev, reg, data, 2);
                spi_busy_wait(spiDev);
            }
        }
    }

    printf("\nPress ENTER to continue...\n");
    std::cin.ignore(1);

    // Cleanup
    configure_frontend_ldo(fd, 0);
    configure_adc_ldo(fd, 0);
}

static void test_capture(file_t fd, uint32_t idx, uint8_t channelBitmap, uint16_t bandwidth, 
    uint32_t volt_scale_uV, int32_t offset_uV, uint8_t ac_couple, uint8_t term, bool watch_bitslip,
    bool is12bit, bool is14bit, bool inRefClk, bool outRefClk, uint32_t refclkFreq)
{
    uint8_t numChan = 0;
    tsHandle_t tsHdl = thunderscopeOpen(idx, false);
    uint32_t bitslip_count = 0;
    uint32_t dbg_monitor;

    uint8_t* sampleBuffer = (uint8_t*)calloc(TS_SAMPLE_BUFFER_SIZE, 0x1000);
    uint64_t sampleLen = 0;
    uint32_t sampleRate = 1000000000;

    if(inRefClk)
    {
        printf("Setting Ref In Clock @ %u Hz\n", refclkFreq);
        printf("\t Result: %i\n", thunderscopeRefClockSet(tsHdl, TS_REFCLK_IN, refclkFreq));
    }
    else if(outRefClk)
    {
        printf("Setting Ref Out Clock @ %u Hz\n", refclkFreq);
        printf("\t Result: %i\n", thunderscopeRefClockSet(tsHdl, TS_REFCLK_OUT, refclkFreq));
    }

    //Setup and Enable Channels
    tsChannelParam_t chConfig = {0};
    uint8_t channel = 0;
    while(channelBitmap > 0)
    {
        if(channelBitmap & 0x1)
        {
            thunderscopeChannelConfigGet(tsHdl, channel, &chConfig);
            chConfig.volt_scale_uV = volt_scale_uV;
            chConfig.volt_offset_uV = offset_uV;
            chConfig.bandwidth = bandwidth;
            chConfig.coupling = ac_couple ? TS_COUPLE_AC : TS_COUPLE_DC;
            chConfig.term =  term ? TS_TERM_50 : TS_TERM_1M;
            chConfig.active = 1;
            thunderscopeChannelConfigSet(tsHdl, channel, &chConfig);
            numChan++;
        }
        channel++;
        channelBitmap >>= 1;
    }

    // Uncomment to use Test Pattern
    // thunderscopeCalibrationAdcTest(tsHdl, TS_ADC_TEST_RAMP, 0);

    if(is12bit)
    {
        sampleRate = 660000000;
        thunderscopeSampleModeSet(tsHdl, sampleRate/numChan, TS_12_BIT_MSB);
    }
    else if(is14bit)
    {
        sampleRate = 125000000;
        thunderscopeSampleModeSet(tsHdl, sampleRate, TS_14_BIT);
    }
    else
    {
        thunderscopeSampleModeSet(tsHdl, sampleRate/numChan, TS_8_BIT);
    }

    printf("- Checking HMCAD1520 Sample Rate...");
    litepcie_writel(fd, CSR_ADC_HMCAD1520_CONTROL_ADDR, 1 << CSR_ADC_HMCAD1520_CONTROL_STAT_RST_OFFSET);
    NS_DELAY(500000000);
    uint32_t rate = litepcie_readl(fd, CSR_ADC_HMCAD1520_SAMPLE_COUNT_ADDR) * 2;
    printf(" %d Samples/S\r\n", rate);
    if(watch_bitslip)
    {
        bitslip_count = litepcie_readl(fd, CSR_ADC_HMCAD1520_BITSLIP_COUNT_ADDR);
        printf("Bitslip Snapshot: %lu\r\n", bitslip_count);
        dbg_monitor = litepcie_readl(fd, CSR_ADC_HMCAD1520_FRAME_DEBUG_ADDR);
        printf("FRAME Debug: 0x%08x\r\n", dbg_monitor);
        dbg_monitor = litepcie_readl(fd, CSR_ADC_HMCAD1520_RANGE_ADDR);
        printf("RANGE: 0x%08x\r\n", dbg_monitor);
    }


    //Only start taking samples if the rate is non-zero
    if(rate > 0)
    {
        uint64_t data_sum = 0;
        //Start Sample capture
        thunderscopeDataEnable(tsHdl, 1);
        litepcie_writel(fd, CSR_ADC_HMCAD1520_CONTROL_ADDR, 1 << CSR_ADC_HMCAD1520_CONTROL_STAT_RST_OFFSET);
        
        auto startTime = std::chrono::steady_clock::now();
        if(sampleBuffer != NULL)
        {
            for(uint32_t loop=0; loop < 150; loop++)
            {
                uint32_t readReq = (TS_SAMPLE_BUFFER_SIZE * 0x100);
                //Collect Samples
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
                
                if(watch_bitslip)
                {
                    tsScopeState_t scopeState = {0};
                    bitslip_count = litepcie_readl(fd, CSR_ADC_HMCAD1520_BITSLIP_COUNT_ADDR);
                    printf("Bitslip Snapshot: %lu\r\n", bitslip_count);
                    dbg_monitor = litepcie_readl(fd, CSR_ADC_HMCAD1520_FRAME_DEBUG_ADDR);
                    printf("FRAME Debug: 0x%08x\r\n", dbg_monitor);
                    dbg_monitor = litepcie_readl(fd, CSR_ADC_HMCAD1520_RANGE_ADDR);
                    printf("RANGE: 0x%08x\r\n", dbg_monitor);
                    thunderscopeStatusGet(tsHdl, &scopeState);
                    printf("Scope State Flags: 0x%08x\r\n", scopeState.flags);
                }
            }
        }
        auto endTime = std::chrono::steady_clock::now();

        //Stop Samples
        thunderscopeDataEnable(tsHdl, 0);
        
        auto deltaNs = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime);
        uint64_t bw = (data_sum * 8 * 1000)/deltaNs.count();
        printf("Collected %" PRIu64 " samples in %" PRIu64 " Mbps\r\n", data_sum, bw);
    }

    //Disable channels
    for(uint8_t i=0; i < TS_NUM_CHANNELS; i++)
    {
        thunderscopeChannelConfigGet(tsHdl, i, &chConfig);
        chConfig.active = 0;
        thunderscopeChannelConfigSet(tsHdl, i, &chConfig);
    }

    thunderscopeClose(tsHdl);

    if(sampleLen > 0)
    {
        auto outFile = std::fstream(TS_TEST_SAMPLE_FILE, std::ios::out | std::ios::binary | std::ios::trunc);
        outFile.write(reinterpret_cast<const char*>(const_cast<const uint8_t*>(sampleBuffer)), sampleLen);
        outFile.flush();
        outFile.close();
        
        AudioFile<uint8_t> outWav;
        is12bit ? outWav.setBitDepth(16) : outWav.setBitDepth(8);
        outWav.setNumChannels(numChan);
        if(numChan > 2)
        {
            outWav.setSampleRate(sampleRate/4);
        }
        else
        {
            outWav.setSampleRate(sampleRate/numChan);
        }

        AudioFile<uint8_t>::AudioBuffer wavBuffer;
        wavBuffer.resize(numChan);
        if(numChan == 1)
        {
            wavBuffer[0].resize(sampleLen);
        }
        else if(numChan == 2)
        {
            wavBuffer[0].resize(sampleLen/numChan);
            wavBuffer[1].resize(sampleLen/numChan);
        }
        else
        {
            wavBuffer[0].resize(sampleLen/4);
            wavBuffer[1].resize(sampleLen/4);
            wavBuffer[2].resize(sampleLen/4);

            if(numChan == 4)
            {
                wavBuffer[3].resize(sampleLen/4);
            }
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
                    if(numChan == 4)
                    {
                        wavBuffer[3][sample] = sampleBuffer[idx++];
                    }
                    else
                    {
                        idx++;
                    }
                }
            }
            sample++;
        }
        outWav.setAudioBuffer(wavBuffer);
        outWav.printSummary();
        outWav.save(TS_TEST_WAV_FILE);
    }
    free(sampleBuffer);
}

static void flash_test(char* arg, file_t fd)
{
    spiflash_dev_t spiflash;
    if(0 == strcmp(arg, "read"))
    {
        spiflash_init(fd, &spiflash);
        uint32_t address = 0x00B00000;
        uint32_t flash_word = read_flash_word(fd, address);
        printf("Read Flash Word 0x%08x : 0x%08X\n", address, flash_word);
        address = 0x00B00004;
        flash_word = read_flash_word(fd, address);
        printf("Read Flash Word 0x%08x : 0x%08X\n", address, flash_word);
    }
    else if(0 == strcmp(arg, "dump"))
    {
        spiflash_init(fd, &spiflash);
        auto outFile = std::fstream(TS_FLASH_DUMP_FILE, std::ios::out | std::ios::binary | std::ios::trunc);
        uint8_t *flash_data = (uint8_t*) malloc(0x40000);
        printf("Dumping Flash to file.\nProgress: ");
        for(uint32_t address = 0x0000000; address < 0x2000000; address+=0x40000)
        {
            spiflash_read(&spiflash, address, flash_data, 0x40000);
            printf("|");
            outFile.write(reinterpret_cast<const char*>(const_cast<const uint8_t*>(flash_data)), 0x40000);
        }
        printf("Done!\n");
        outFile.flush();
        outFile.close();
        free(flash_data);
    }
    else if(0 == strcmp(arg, "test"))
    {
        uint8_t test_data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01, 0x23, 0x45};
        uint32_t test_addr = 0x00B00000;
        int32_t status;
        spiflash_init(fd, &spiflash);
        
        printf("Erasing 64k Sector @ Address 0x%08X: ", test_addr);
        status = spiflash_erase(&spiflash, test_addr, 0x40000);
        if(TS_STATUS_OK == status)
        {
            printf("OK\n");
        }
        else
        {
            printf("FAIL (%d)\n", status);
        }
        
        printf("Test Write @ Address 0x%08X: ", test_addr);
        status = spiflash_write(&spiflash, test_addr, test_data, 8);
        if(8 == status)
        {
            printf("OK\n");
        }
        else
        {
            printf("FAIL (%d)\n", status);
        }

        printf("Read Values @ Address 0x%08X: [", test_addr);
        uint8_t readback_data[8] = {0};
        status = spiflash_read(&spiflash, test_addr, readback_data, 8);
        for(uint8_t i=0; i < 8; i++)
        {
            printf("%02X ", readback_data[i]);
        }
        if(8 == status)
        {
            printf("] OK\n");
        }
        else
        {
            printf("] FAIL (%d)\n", status);
        }
        
        printf("Done!\n");
        
    }
}

static void test_clock(uint32_t idx, bool refoutclk, bool refinclk, uint32_t refclkfreq)
{
    tsHandle_t tsHdl = thunderscopeOpen(idx, false);
    bool firstStatus = false;

    if(tsHdl)
    {
#ifdef _WIN32
        SetConsoleCtrlHandler(SigHandler, TRUE);
#else
        struct sigaction signalHandler;
        signalHandler.sa_handler = SigHandler;
        sigemptyset(&signalHandler.sa_mask);
        signalHandler.sa_flags = 0;
        sigaction(SIGINT, &signalHandler, NULL);
#endif
        if(refinclk)
        {
            printf("Setting Ref In Clock @ %u Hz\n", refclkfreq);
            printf("\t Result: %i\n", thunderscopeRefClockSet(tsHdl, TS_REFCLK_IN, refclkfreq));
        }
        else if(refoutclk)
        {
            printf("Setting Ref Out Clock @ %u Hz\n", refclkfreq);
            printf("\t Result: %i\n", thunderscopeRefClockSet(tsHdl, TS_REFCLK_OUT, refclkfreq));
        }

        while (g_program_loop)
        {
            tsScopeState_t state = {0};
            thunderscopeStatusGet(tsHdl, &state);
            // if(!firstStatus)
            // {
            //     printf("\x1b[A\x1b[A\x1b[A\x1b[A\x1b[A\x1b[A\x1b[A\x1b[A\x1b[A\x1b[A");
            //     firstStatus = true;
            // }
            printf("-------\r\n");
            printf("PLL Status:\r\n");
            printf("\tPLL LOCK - %01x\r\n", state.pll_lock);
            printf("\tPLL HIGH - %01x\r\n", state.pll_high);
            printf("\tPLL LOW  - %01x\r\n", state.pll_low);
            printf("\tPLL ALT  - %01x\r\n", state.pll_alt);
            printf("INPUT Clock Status:\r\n");
            printf("\tLocal Valid  - %01x\r\n", state.local_osc_clk);
            printf("\tREF IN Valid - %01x\r\n", state.ref_in_clk);
            printf("\r\n-- PRESS CTRL+C TO STOP --\r\n");
            
            std::cout.flush();
            std::this_thread::sleep_until(awake_time());
        }

        thunderscopeClose(tsHdl);
    }
}

static void print_help(void)
{
    printf("TS Test Util Usage:\r\n");
    printf("\t io - run I/O Test\r\n");
    printf("\t flash - Test Flash device\r\n");
    printf("\t\t read             Read a word from flash\r\n");
    printf("\t\t dump             Dump the contents of flash to a file\r\n");
    printf("\t\t test             Destructive Flash erase/read/write test\r\n");
    printf("\t capture - run Sample Capture Test\r\n");
    printf("\t\t -d <device>      Device Index\r\n");
    printf("\t\t -c <channels>    Channel bitmap\r\n");
    printf("\t\t -b <bw>          Channel Bandwidth [MHz]\r\n");
    printf("\t\t -v <uvolts>      Channel Full Scale Volts [microvolt]\r\n");
    printf("\t\t -o <uvolts>      Channel Offset [microvolt]\r\n");
    printf("\t\t -a               AC Couple\r\n");
    printf("\t\t -t               50 Ohm termination\r\n");
    printf("\t refclk - run the PLL source with different clock configurations\r\n");
    printf("\t\t -i <hz>          Set the Ref IN Clock frequency\r\n");
    printf("\t\t -r <hz>          Set the Ref OUT Clock frequency\r\n");
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
    uint32_t volt_scale_uV = 10000000;
    int32_t offset_uV = 0;
    uint8_t ac_couple = 0;
    uint8_t term = 0;
    bool bitslip = false;
    bool mode12bit = false, mode14bit = false;
    bool refInClk = false;
    bool refOutClk = false;
    uint32_t refclkFreq = 0;

    struct optparse_long argList[] = {
        {"dev",      'd', OPTPARSE_REQUIRED},
        {"chan",     'c', OPTPARSE_REQUIRED},
        {"bw",       'b', OPTPARSE_REQUIRED},
        {"voltsuv",  'v', OPTPARSE_REQUIRED},
        {"offsetuv", 'o', OPTPARSE_REQUIRED},
        {"refinclk", 'i', OPTPARSE_REQUIRED},
        {"refoutclk",'r', OPTPARSE_REQUIRED},
        {"ac",       'a', OPTPARSE_NONE},
        {"term",     't', OPTPARSE_NONE},
        {"bits",     's', OPTPARSE_NONE},
        {"12bit",    'm', OPTPARSE_NONE},
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
            volt_scale_uV = strtol(options.optarg, NULL, 0);
            argCount+=2;
            break;
        case 'o':
            offset_uV = strtol(options.optarg, NULL, 0);
            argCount+=2;
            break;
        case 'i':
            refInClk = true;
            refclkFreq = strtol(options.optarg, NULL, 0);
            argCount+=2;
            break;
        case 'r':
            refOutClk = true;
            refclkFreq = strtol(options.optarg, NULL, 0);
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
        case 's':
            bitslip = true;
            argCount++;
            break;
        case 'm':
            mode12bit = true;
            argCount++;
            break;
        case 'p':
            mode14bit = true;
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

    if(0 == strcmp(arg, "list"))
    {
        uint32_t i = 0;
        tsDeviceInfo_t infos;
        while(TS_STATUS_OK == thunderscopeListDevices(i, &infos))
        {
            if(i==0)
            {
                printf("Found ThunderScope(s):\n");
            }
            printf("\t%3d | Serial Number: %s\n", i, infos.serial_number);
            printf("\t    | HW Rev:    0x%x\n", infos.hw_id);
            printf("\t    | GW Rev:    0x%x\n", infos.gw_id);
            printf("\t    | LiteX Rev: 0x%x\n", infos.litex);
            i++;
        }
        if(i == 0)
        {
            printf("No devices present\n");
        }
        exit(EXIT_SUCCESS);
    }

    if(0 == strcmp(arg, "clk"))
    {
        mcp_clkgen_conf_t test_conf[1024];
        zl3026x_clk_config_t clk_config = {0};
        clk_config.in_clks[1].enable = 1;
        clk_config.in_clks[1].input_freq = 10000000;
        clk_config.input_select = ZL3026X_INPUT_IC2;
        clk_config.out_clks[0].enable = 1;
        clk_config.out_clks[0].output_freq = 10000000;
        clk_config.out_clks[0].output_mode = ZL3026X_OUT_CMOS_P;
        clk_config.out_clks[0].output_pll_select = ZL3026X_PLL_BYPASS;
        clk_config.out_clks[5].enable = 1;
        clk_config.out_clks[5].output_freq = 1000000000;
        clk_config.out_clks[5].output_mode = ZL3026X_OUT_DIFF;
        clk_config.out_clks[5].output_pll_select = ZL3026X_PLL_INT_DIV;

        int32_t clk_result = mcp_zl3026x_build_config(test_conf, 1024, clk_config);

        if(clk_result < 1)
        {
            printf("Invalid Clock Configuration: %d\n", clk_result);
            return -1;
        }
        else
        {
            printf("MCP Clock Configuration Array (Len %d)\n", clk_result);
            int32_t idx = 0;
            while(idx < clk_result)
            {
                if(test_conf[idx].action == MCP_CLKGEN_WRITE_REG)
                {
                    printf("\t{WRITE_REG: 0x%04X  0x%02X}\n", test_conf[idx].addr, test_conf[idx].value);
                }
                else
                {
                    printf("\t{DELAY: %d us}\n", test_conf[idx].delay_us);
                }
                idx++;
            }
        }
        return 0;
    }

    snprintf(devicePath, TS_IDENT_STR_LEN, LITEPCIE_CTRL_NAME(%d), idx);
    printf("Opening Device %s\n", devicePath);
    fd = litepcie_open((const char*)devicePath, FILE_FLAGS);
    if(fd == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }


    printf("\x1b[1m[> Device Information:\x1b[0m\n");
    printf("------------------------\n");

    for (i = 0; i < 256; i++)
    {
        fpga_identifier[i] = litepcie_readl(fd, CSR_IDENTIFIER_MEM_BASE + 4 * i);
    }
    printf("FPGA Identifier:  %s.\n", fpga_identifier);
    uint32_t hw_info = litepcie_readl(fd, CSR_DEV_STATUS_HW_ID_ADDR);
    if(hw_info & TS_HW_ID_VALID_MASK)
    {
        printf("HW Rev %02d - %s\n", hw_info & TS_HW_ID_REV_MASK, 
            (hw_info & TS_HW_ID_VARIANT_MASK) ? "TB" : "PCIe" );
    }
    else
    {
        printf("HW Rev Beta\n");
    }
    printf("Gateware Rev:     0x%08X\n",
        litepcie_readl(fd, CSR_DEV_STATUS_GW_REV_ADDR));
    printf("LiteX Release:    0x%08X\n",
        litepcie_readl(fd, CSR_DEV_STATUS_LITEX_REL_ADDR));
        
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
        test_io(fd, ((hw_info & TS_HW_ID_VALID_MASK) == 0));
    }
    // Setup Channel, record samples to buffer, save buffer to file
    else if(0 == strcmp(arg, "capture"))
    {
        test_capture(fd, idx, channelBitmap, bandwidth, volt_scale_uV, offset_uV, ac_couple, term, bitslip, mode12bit, mode14bit, refInClk, refOutClk, refclkFreq);
    }
    // Flash test
    else if(0 == strcmp(arg, "flash"))
    {
        flash_test(argv[argCount+1], fd);
    }
    // Test REF Clock Modes
    else if(0 == strcmp(arg, "clock"))
    {
        test_clock(idx, refOutClk, refInClk, refclkFreq);
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