// litepcie_thunderscope.cpp : This file contains the 'main test' function. Program execution begins and ends there.
//
// This file contains parts Copyright (c) 2020 Antmicro <www.antmicro.com>
// This file contains parts Copyright (c) 2022 Franck Jullien <franck.jullien@collshade.fr>

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
//#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>

#include <chrono>
#include <iostream>
#include <thread>



#ifdef _WIN32
#include <Windows.h>
//#include <SetupAPI.h>
//#include <INITGUID.H>
//#include <ioapiset.h>
#endif

#include "liblitepcie.h"
#include "litepcie_thunderscope.h"
//#include "litepcie_public.h"
#include <csr.h>
#include <soc.h>

#define DMA_EN
#define FLASH_EN

/* Parameters */
/*------------*/

//#define DMA_CHECK_DATA   /* Un-comment to disable data check */
//#define DMA_RANDOM_DATA  /* Un-comment to disable data random */

/* Variables */
/*-----------*/

static WCHAR litepcie_device[1024];
static int litepcie_device_num;

sig_atomic_t keep_running = 1;

void intHandler(int dummy) {
    keep_running = 0;
}

static int64_t get_time_ms(void)
{
    struct timespec timeNow;
    timespec_get(&timeNow, TIME_UTC);

    int64_t timeMS = (timeNow.tv_sec * 1000) + (timeNow.tv_nsec / 1000000);
    return timeMS;
}

UINT32 AFE_CONTROL_LDO_EN = (1 << 0);
UINT32 AFE_CONTROL_COUPLING = (1 << 8);
UINT32 AFE_CONTROL_ATTENUATION = (1 << 16);

UINT32 AFE_STATUS_LDO_PWR_GOOD = (1 << 0);

UINT32 AFE_AC_COUPLING = 0;
UINT32 AFE_DC_COUPLING = 1;

UINT32 AFE_1X_ATTENUATION = 0;
UINT32 AFE_10X_ATTENUATION = 1;

// ADC Constants------------------------------------------------------------------------------------

UINT32 ADC_CONTROL_LDO_EN = (1 << 0);
UINT32 ADC_CONTROL_PLL_EN = (1 << 1);
UINT32 ADC_CONTROL_RST = (1 << 2);
UINT32 ADC_CONTROL_PWR_DOWN = (1 << 3);

UINT32 ADC_STATUS_LDO_PWR_GOOD = (1 << 0);

UINT32 _SPI_CONTROL_START = (1 << 0);
UINT32 _SPI_CONTROL_LENGTH = (1 << 8);
UINT32 _SPI_STATUS_DONE = (1 << 0);


/* Main */
/*------*/

void configure_frontend_ldo(HANDLE fd, UINT32 enable) {
    UINT32 control_value = litepcie_readl(fd, CSR_FRONTEND_CONTROL_ADDR);
    control_value &= ~(1 * AFE_CONTROL_LDO_EN);
    control_value |= (enable * AFE_CONTROL_LDO_EN);
    litepcie_writel(fd, CSR_FRONTEND_CONTROL_ADDR, control_value);
}

void configure_adc_ldo(HANDLE fd, UINT32 enable) {
    UINT32 control_value = litepcie_readl(fd, CSR_ADC_CONTROL_ADDR);
    control_value &= ~(1 * ADC_CONTROL_LDO_EN);
    control_value |= (enable * ADC_CONTROL_LDO_EN);
    litepcie_writel(fd, CSR_ADC_CONTROL_ADDR, control_value);
}

void configure_pll_en(HANDLE fd, UINT32 enable) {
    UINT32 control_value = litepcie_readl(fd, CSR_ADC_CONTROL_ADDR);
    control_value &= ~(1 * ADC_CONTROL_PLL_EN);
    control_value |= (enable * ADC_CONTROL_PLL_EN);
    litepcie_writel(fd, CSR_ADC_CONTROL_ADDR, control_value);
}

void control_led(HANDLE fd, UINT32 enable) {
    UINT32 control_value = litepcie_readl(fd, CSR_LEDS_OUT_ADDR);
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

#ifdef __cplusplus
extern "C" {
#endif

    // I2C

    HANDLE fd;

#define U_SECOND	(1000000)
#define I2C_PERIOD	(U_SECOND / I2C_FREQ_HZ)
#define I2C_DELAY(n)	std::this_thread::sleep_for(std::chrono::microseconds((n * I2C_PERIOD)));


    char I2C_SCL = 0x01;
    char I2C_SDAOE = 0x02;
    char I2C_SDAOUT = 0x04;
    char I2C_SDAIN = 0x01;

    char I2C_DELAY = 1;
    char I2C_WRITE = 0;
    char I2C_READ = 1;

    static inline void i2c_oe_scl_sda(bool oe, bool scl, bool sda)
    {
        //struct i2c_ops ops = i2c_devs[current_i2c_dev].ops;

        litepcie_writel(fd, CSR_I2C_W_ADDR, ((oe & 1) << CSR_I2C_W_OE_OFFSET) |
            ((scl & 1) << CSR_I2C_W_SCL_OFFSET) |
            ((sda & 1) << CSR_I2C_W_SDA_OFFSET));

    }

    // START condition: 1-to-0 transition of SDA when SCL is 1
    static void i2c_start(void)
    {
        i2c_oe_scl_sda(1, 1, 1);
        I2C_DELAY(1);
        i2c_oe_scl_sda(1, 1, 0);
        I2C_DELAY(1);
        i2c_oe_scl_sda(1, 0, 0);
        I2C_DELAY(1);
    }

    // STOP condition: 0-to-1 transition of SDA when SCL is 1
    static void i2c_stop(void)
    {
        i2c_oe_scl_sda(1, 0, 0);
        I2C_DELAY(1);
        i2c_oe_scl_sda(1, 1, 0);
        I2C_DELAY(1);
        i2c_oe_scl_sda(1, 1, 1);
        I2C_DELAY(1);
        i2c_oe_scl_sda(0, 1, 1);
    }

    // Call when in the middle of SCL low, advances one clk period
    static void i2c_transmit_bit(int value)
    {
        i2c_oe_scl_sda(1, 0, value);
        I2C_DELAY(1);
        i2c_oe_scl_sda(1, 1, value);
        I2C_DELAY(2);
        i2c_oe_scl_sda(1, 0, value);
        I2C_DELAY(1);
    }

    // Call when in the middle of SCL low, advances one clk period
    static int i2c_receive_bit(void)
    {
        int value;
        i2c_oe_scl_sda(0, 0, 0);
        I2C_DELAY(1);
        i2c_oe_scl_sda(0, 1, 0);
        I2C_DELAY(1);
        // read in the middle of SCL high
        value = litepcie_readl(fd, CSR_I2C_R_ADDR) & 1;
        I2C_DELAY(1);
        i2c_oe_scl_sda(0, 0, 0);
        I2C_DELAY(1);
        return value;
    }

    // Send data byte and return 1 if slave sends ACK
    static bool i2c_transmit_byte(unsigned char data)
    {
        int i;
        int ack;

        // SCL should have already been low for 1/4 cycle
        // Keep SDA low to avoid short spikes from the pull-ups
        i2c_oe_scl_sda(1, 0, 0);
        for (i = 0; i < 8; ++i) {
            // MSB first
            i2c_transmit_bit((data & (1 << 7)) != 0);
            data <<= 1;
        }
        i2c_oe_scl_sda(0, 0, 0); // release line
        ack = i2c_receive_bit();

        // 0 from slave means ack
        return ack == 0;
    }

    // Read data byte and send ACK if ack=1
    static unsigned char i2c_receive_byte(bool ack)
    {
        int i;
        unsigned char data = 0;

        for (i = 0; i < 8; ++i) {
            data <<= 1;
            data |= i2c_receive_bit();
        }
        i2c_transmit_bit(!ack);
        i2c_oe_scl_sda(0, 0, 0); // release line

        return data;
    }

    // Reset line state
    void i2c_reset(void)
    {
        int i;
        i2c_oe_scl_sda(1, 1, 1);
        I2C_DELAY(8);
        for (i = 0; i < 9; ++i) {
            i2c_oe_scl_sda(1, 0, 1);
            I2C_DELAY(2);
            i2c_oe_scl_sda(1, 1, 1);
            I2C_DELAY(2);
        }
        i2c_oe_scl_sda(0, 0, 1);
        I2C_DELAY(1);
        i2c_stop();
        i2c_oe_scl_sda(0, 1, 1);
        I2C_DELAY(8);
    }

    /*
     * Read slave memory over I2C starting at given address
     *
     * First writes the memory starting address, then reads the data:
     *   START WR(slaveaddr) WR(addr) STOP START WR(slaveaddr) RD(data) RD(data) ... STOP
     * Some chips require that after transmiting the address, there will be no STOP in between:
     *   START WR(slaveaddr) WR(addr) START WR(slaveaddr) RD(data) RD(data) ... STOP
     */
    bool i2c_read(unsigned char slave_addr, unsigned int addr, unsigned char* data, unsigned int len, bool send_stop, unsigned int addr_size)
    {
        int i, j;

        if ((addr_size < 1) || (addr_size > 4)) {
            return false;
        }

        i2c_start();

        if (!i2c_transmit_byte(I2C_ADDR_WR(slave_addr))) {
            i2c_stop();
            return false;
        }
        for (j = addr_size - 1; j >= 0; j--) {
            if (!i2c_transmit_byte((unsigned char)(0xff & (addr >> (8 * j))))) {
                i2c_stop();
                return false;
            }
        }

        if (send_stop) {
            i2c_stop();
        }
        i2c_start();

        if (!i2c_transmit_byte(I2C_ADDR_RD(slave_addr))) {
            i2c_stop();
            return false;
        }
        for (i = 0; i < len; ++i) {
            data[i] = i2c_receive_byte(i != len - 1);
        }

        i2c_stop();

        return true;
    }

    /*
     * Write slave memory over I2C starting at given address
     *
     * First writes the memory starting address, then writes the data:
     *   START WR(slaveaddr) WR(addr) WR(data) WR(data) ... STOP
     */
    bool i2c_write(unsigned char slave_addr, unsigned int addr, const unsigned char* data, unsigned int len, unsigned int addr_size)
    {
        int i, j;

        if ((addr_size < 1) || (addr_size > 4)) {
            return false;
        }

        i2c_start();

        if (!i2c_transmit_byte(I2C_ADDR_WR(slave_addr))) {
            i2c_stop();
            return false;
        }
        for (j = addr_size - 1; j >= 0; j--) {
            if (!i2c_transmit_byte((unsigned char)(0xff & (addr >> (8 * j))))) {
                i2c_stop();
                return false;
            }
        }
        for (i = 0; i < len; ++i) {
            if (!i2c_transmit_byte(data[i])) {
                i2c_stop();
                return false;
            }
        }

        i2c_stop();

        return true;
    }

    /*
     * Poll I2C slave at given address, return true if it sends an ACK back
     */
    bool i2c_poll(unsigned char slave_addr)
    {
        bool result;

        i2c_start();
        result = i2c_transmit_byte(I2C_ADDR_WR(slave_addr));
        if (!result) {
            i2c_start();
            result |= i2c_transmit_byte(I2C_ADDR_RD(slave_addr));
            if (result)
                i2c_receive_byte(false);
        }
        i2c_stop();

        return result;
    }

    //  SPI
    void spi_write(HANDLE fd, int cs, char reg, char data[2]) {

        litepcie_writel(fd, CSR_MAIN_SPI_CS_ADDR, (1 << cs));

        uint32_t mosi_data = (reg << 16) + (data[1] << 8) + data[0];

        litepcie_writel(fd, CSR_MAIN_SPI_MOSI_ADDR, mosi_data);

        litepcie_writel(fd, CSR_MAIN_SPI_CONTROL_ADDR, 24 * _SPI_CONTROL_LENGTH | _SPI_CONTROL_START);

        while (litepcie_readl(fd, CSR_MAIN_SPI_STATUS_ADDR) != SPI_STATUS_DONE)
        {
            continue;
        }
    }
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
    //HANDLE fd;
    int i;
    unsigned char fpga_identifier[256];
    fd = litepcie_open("\\CTRL", GENERIC_READ | GENERIC_WRITE);
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
    printf("FPGA Temperature: %0.1f °C\n",
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

    /* Open LitePCIe device. */
    fd = litepcie_open("\\CTRL", FILE_ATTRIBUTE_NORMAL);
    if (fd == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }


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


    fd = fd;
//    I2CDriver i2cDriver = I2CDriver(fd);
    //init_zl30250Driver(i2cDriver);

    for (unsigned char addr = 0; addr < 0x80; addr++) {
        auto result = i2c_poll(addr);
        if (addr % 0x10 == 0) {
            printf("\n0x%02X", addr);
        }
        if (result == 1) {
            printf(" %02X", addr);
        }
        else {
            printf(" --");
        }

    }

    char data[2] = {0x01, 0x02};

    for (int i = 0; i < CSR_MAIN_SPI_CS_SEL_SIZE; i++) {
        for (int reg = 0; reg < 10; reg++) {

            spi_write(fd, i, reg, data);
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        
    }


    /* Close LitePCIe device. */
    litepcie_close(fd);


    return 0;

}
