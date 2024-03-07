// litepcie_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>

#ifdef _WIN32
#include <Windows.h>
#endif

#include "liblitepcie.h"
#include <csr.h>
#include <soc.h>

#if !defined(_WIN32)
#define INVALID_HANDLE_VALUE (-1)
#endif


#ifdef _WIN32
#define FILE_FLAGS  (FILE_ATTRIBUTE_NORMAL)
#else
#define FILE_FLAGS  (O_RDWR)
#endif

#define DMA_EN
//#define FLASH_EN

/* Parameters */
/*------------*/

//#define DMA_CHECK_DATA   /* Un-comment to disable data check */
//#define DMA_RANDOM_DATA  /* Un-comment to disable data random */

/* Variables */
/*-----------*/

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

/* Info */
/*------*/
static void info(void)
{
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
        litepcie_readl(fd, CSR_DNA_ID_ADDR + 4 * 1)
    );
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
    litepcie_close(fd);
}

/* Scratch */
/*---------*/

void scratch_test(void)
{
    file_t fd;

    printf("\x1b[1m[> Scratch register test:\x1b[0m\n");
    printf("-------------------------\n");

    /* Open LitePCIe device. */
    fd = litepcie_open(LITEPCIE_CTRL_NAME(0), FILE_FLAGS);
    if (fd == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    /* Write to scratch register. */
    printf("Write 0x12345678 to Scratch register:\n");
    litepcie_writel(fd, CSR_CTRL_SCRATCH_ADDR, 0x12345678);
    printf("Read: 0x%08x\n", litepcie_readl(fd, CSR_CTRL_SCRATCH_ADDR));

    /* Read from scratch register. */
    printf("Write 0xdeadbeef to Scratch register:\n");
    litepcie_writel(fd, CSR_CTRL_SCRATCH_ADDR, 0xdeadbeef);
    printf("Read: 0x%08x\n", litepcie_readl(fd, CSR_CTRL_SCRATCH_ADDR));

    /* Close LitePCIe device. */
    litepcie_close(fd);
}

/* SPI Flash */
/*-----------*/

#ifdef FLASH_EN
#ifdef CSR_FLASH_BASE

static void flash_progress(void* opaque, const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    fflush(stdout);
    va_end(ap);
}

static void flash_program(uint32_t base, const uint8_t* buf1, int size1)
{
    file_t fd;

    uint32_t size;
    uint8_t *buf;
    int sector_size;
    int errors;

    /* Open LitePCIe device. */
    fd = litepcie_open(LITEPCIE_CTRL_NAME(0), FILE_FLAGS);
    if (fd == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    /* Get flash sector size and pad size to it. */
    sector_size = litepcie_flash_get_erase_block_size(fd);
    size = ((size1 + sector_size - 1) / sector_size) * sector_size;

    /* Alloc buffer and copy data to it. */
    buf = (uint8_t*)calloc(1, size);
    if (!buf) {
        fprintf(stderr, "%d: alloc failed\n", __LINE__);
        exit(1);
    }
    memcpy(buf, buf1, size1);

    /* Program flash. */
    printf("Programming (%d bytes at 0x%08x)...\n", size, base);
    errors = litepcie_flash_write(fd, buf, base, size, flash_progress, NULL);
    if (errors) {
        printf("Failed %d errors.\n", errors);
        exit(1);
    } else {
        printf("Success.\n");
    }

    /* Free buffer and close LitePCIe device. */
    free(buf);
    litepcie_close(fd);
}

static void flash_write(const char* filename, uint32_t offset)
{
    uint8_t* data;
    int size;
    FILE* f;

    /* Open data source file. */
    fopen_s(&f, filename, "rb");
    if (!f) {
        perror(filename);
        exit(1);
    }

    /* Get size, alloc buffer and copy data to it. */
    fseek(f, 0L, SEEK_END);
    size = ftell(f);
    fseek(f, 0L, SEEK_SET);
    data = (uint8_t*)malloc(size);
    if (!data) {
        fprintf(stderr, "%d: malloc failed\n", __LINE__);
        exit(1);
    }
    size_t ret = fread(data, size, 1, f);
    fclose(f);

    /* Program file to flash */
    if (ret != 1)
        perror(filename);
    else
        flash_program(offset, data, size);

    /* Free buffer */
    free(data);
}

static void flash_read(const char* filename, uint32_t size, uint32_t offset)
{
    file_t fd;
    FILE* f;
    uint32_t base;
    uint32_t sector_size;
    uint8_t byte;
    int i;

    /* Open data destination file. */
    fopen_s(&f, filename, "wb");
    if (!f) {
        perror(filename);
        exit(1);
    }

    /* Open LitePCIe device. */
    fd = litepcie_open(LITEPCIE_CTRL_NAME(0), FILE_FLAGS);
    if (fd == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    /* Get flash sector size. */
    sector_size = litepcie_flash_get_erase_block_size(fd);

    /* Read flash and write to destination file. */
    base = offset;
    for (i = 0; i < size; i++) {
        if ((i % sector_size) == 0) {
            printf("Reading 0x%08x\r", base + i);
            fflush(stdout);
        }
        byte = litepcie_flash_read(fd, base + i);
        fwrite(&byte, 1, 1, f);
    }

    /* Close destination file and LitePCIe device. */
    fclose(f);
    litepcie_close(fd);
}

static void flash_reload(void)
{
    file_t fd;

    /* Open LitePCIe device. */
    fd = litepcie_open(LITEPCIE_CTRL_NAME(0), FILE_FLAGS);
    if (fd == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    /* Reload FPGA through ICAP.*/
    litepcie_reload(fd);

    /* Notice user to reboot the hardware.*/
    printf("================================================================\n");
    printf("= PLEASE REBOOT YOUR HARDWARE TO START WITH NEW FPGA GATEWARE  =\n");
    printf("================================================================\n");

    /* Close LitePCIe device. */
    litepcie_close(fd);
}
#endif
#endif //FLASH_EN

/* DMA */
/*-----*/
#ifdef DMA_EN
static inline int64_t add_mod_int(int64_t a, int64_t b, int64_t m)
{
    a += b;
    if (a >= m)
        a -= m;
    return a;
}

static int get_next_pow2(int data_width)
{
    int x = 1;
    while (x < data_width)
        x <<= 1;
    return x;
}

#ifdef DMA_CHECK_DATA

static inline uint32_t seed_to_data(uint32_t seed)
{
#ifdef DMA_RANDOM_DATA
    /* Return pseudo random data from seed. */
    return seed * 69069 + 1;
#else
    /* Return seed. */
    return seed;
#endif
}

static uint32_t get_data_mask(int data_width)
{
    int i;
    uint32_t mask;
    mask = 0;
    for (i = 0; i < 32 / get_next_pow2(data_width); i++) {
        mask <<= get_next_pow2(data_width);
        mask |= (1 << data_width) - 1;
    }
    return mask;
}

static void write_pn_data(uint32_t* buf, int count, uint32_t* pseed, int data_width)
{
    int i;
    uint32_t seed;
    uint32_t mask = get_data_mask(data_width);

    seed = *pseed;
    for (i = 0; i < count; i++) {
        buf[i] = (seed_to_data(seed) & mask);
        seed = add_mod_int(seed, 1, DMA_BUFFER_SIZE / sizeof(uint32_t));
    }
    *pseed = seed;
}

static int check_pn_data(const uint32_t* buf, int count, uint32_t* pseed, int data_width)
{
    int i, errors;
    uint32_t seed;
    uint32_t mask = get_data_mask(data_width);

    errors = 0;
    seed = *pseed;
    for (i = 0; i < count; i++) {
        if (buf[i] != (seed_to_data(seed) & mask)) {
            errors++;
        }
        seed = add_mod_int(seed, 1, DMA_BUFFER_SIZE / sizeof(uint32_t));
    }
    *pseed = seed;
    return errors;
}
#endif

static void dma_test(uint8_t zero_copy, uint8_t external_loopback, int data_width, int auto_rx_delay)
{
    static struct litepcie_dma_ctrl dma = {0};
    dma.use_reader = 1;
    dma.use_writer = 1;
    dma.loopback = external_loopback ? 0 : 1;

    if (data_width > 32 || data_width < 1) {
        fprintf(stderr, "Invalid data width %d\n", data_width);
        exit(1);
    }

    /* Statistics */
    int i = 0;
    int64_t reader_sw_count_last = 0;
    int64_t reader_hw_count_last = 0;
    int64_t writer_hw_count_last = 0;
    int64_t last_time;
    uint32_t errors = 0;

#ifdef DMA_CHECK_DATA
    uint32_t seed_wr = 0;
    uint32_t seed_rd = 0;
    uint8_t  run = (auto_rx_delay == 0);
#else
    uint8_t run = 1;
#endif

    signal(SIGINT, intHandler);

    printf("\x1b[1m[> DMA loopback test:\x1b[0m\n");
    printf("---------------------\n");

    if (litepcie_dma_init(&dma, LITEPCIE_DMA_NAME(0, 0), zero_copy))
        exit(1);

#ifdef DMA_CHECK_DATA
    /* DMA-TX Write. */
    while (1) {
        char* buf_wr;
        /* Get Write buffer. */
        buf_wr = litepcie_dma_next_write_buffer(&dma);
        /* Break when no buffer available for Write. */
        if (!buf_wr)
            break;
        /* Write data to buffer. */
        write_pn_data((uint32_t*)buf_wr, DMA_BUFFER_SIZE / sizeof(uint32_t), &seed_wr, data_width);
    }
#endif

    /* Test loop. */
    last_time = get_time_ms();
    for (;;) {
        /* Exit loop on CTRL+C. */
        if (!keep_running)
            break;

        /* Update DMA status. */
        litepcie_dma_process(&dma);

        /* DMA-TX Write. */
        while (1) {
            char* buf_wr;
            /* Get Write buffer. */
            buf_wr = litepcie_dma_next_write_buffer(&dma);
            /* Break when no buffer available for Write. */
            if (!buf_wr)
                break;
#ifdef DMA_CHECK_DATA
            /* Write data to buffer. */
            write_pn_data((uint32_t*)buf_wr, DMA_BUFFER_SIZE / sizeof(uint32_t), &seed_wr, data_width);
#endif
        }

        /* DMA-RX Read/Check */
        while (1) {
            char* buf_rd;
            /* Get Read buffer. */
            buf_rd = litepcie_dma_next_read_buffer(&dma);
            /* Break when no buffer available for Read. */
            if (!buf_rd)
                break;
#ifdef DMA_CHECK_DATA
            /* Skip the first 128 DMA loops. */
            if (dma.writer_hw_count < 128 * DMA_BUFFER_COUNT)
                break;
            /* When running... */
            if (run) {
                /* Check data in Read buffer. */
                errors += check_pn_data((uint32_t*)buf_rd, DMA_BUFFER_SIZE / sizeof(uint32_t), &seed_rd, data_width);
                /* Clear Read buffer */
                memset(buf_rd, 0, DMA_BUFFER_SIZE);
            }
            else {
                /* Find initial Delay/Seed (Useful when loopback is introducing delay). */
                uint32_t errors_min = 0xffffffff;
                for (int delay = 0; delay < DMA_BUFFER_SIZE / sizeof(uint32_t); delay++) {
                    seed_rd = delay;
                    errors = check_pn_data((uint32_t*)buf_rd, DMA_BUFFER_SIZE / sizeof(uint32_t), &seed_rd, data_width);
                    //printf("delay: %d / errors: %d\n", delay, errors);
                    if (errors < errors_min)
                        errors_min = errors;
                    if (errors < (DMA_BUFFER_SIZE / sizeof(uint32_t)) / 2) {
                        printf("RX_DELAY: %d (errors: %d)\n", delay, errors);
                        run = 1;
                        break;
                    }
                }
                if (!run) {
                    printf("Unable to find DMA RX_DELAY (min errors: %d/%lld), exiting.\n",
                        errors_min,
                        DMA_BUFFER_SIZE / sizeof(uint32_t));
                    goto end;
                }
            }
#endif

        }

        /* Statistics every 200ms. */
        int64_t duration = get_time_ms() - last_time;
        if (run && (duration > 200)) {
            /* Print banner every 10 lines. */
            if (i % 10 == 0)
                printf("\x1b[1mDMA_SPEED(Gbps)\tTX_BUFFERS\tRX_BUFFERS\tDIFF\tERRORS\x1b[0m\n");
            i++;
            /* Print statistics. */
            printf("%14.2f\t%10" PRIu64 "\t%10" PRIu64 "\t%4" PRIu64 "\t%6u\n",
                   (double)(dma.reader_sw_count - reader_sw_count_last) * DMA_BUFFER_SIZE * 8 * data_width / (get_next_pow2(data_width) * (double)duration * 1e6),
                   dma.reader_sw_count,
                   dma.writer_sw_count,
                   dma.reader_sw_count - dma.writer_sw_count,
                   errors);
            printf("%14.2f Gbps TX \t%14.2f Gbps RX\n",
                (double)(dma.reader_hw_count - reader_hw_count_last) * DMA_BUFFER_SIZE * 8 / ((double)duration * 1e6),
                (double)(dma.writer_hw_count - writer_hw_count_last) * DMA_BUFFER_SIZE * 8 / ((double)duration * 1e6));
            printf("\t\t%10" PRIi64 "\t%10" PRIi64 "\n",
                (dma.reader_hw_count - dma.reader_sw_count),
                (dma.writer_hw_count - dma.writer_sw_count));
            /* Update errors/time/count. */
            errors = 0;
            last_time = get_time_ms();
            reader_sw_count_last = dma.reader_sw_count;
            reader_hw_count_last = dma.reader_hw_count;
            writer_hw_count_last = dma.writer_hw_count;
        }
    }


    /* Cleanup DMA. */
#ifdef DMA_CHECK_DATA
    end :
#endif
    litepcie_dma_cleanup(&dma);
}
#endif

/* Help */
/*------*/

static void help(void)
{
    printf("LitePCIe utilities\n"
        "usage: litepcie_util [options] cmd [args...]\n"
        "\n"
        "options:\n"
        "-h                                Help.\n"
        "-c device_num                     Select the device (default = 0).\n"
        "-z                                Enable zero-copy DMA mode.\n"
        "-e                                Use external loopback (default = internal).\n"
        "-w data_width                     Width of data bus (default = 16).\n"
        "-a                                Automatic DMA RX-Delay calibration.\n"
        "\n"
        "available commands:\n"
        "info                              Get Board information.\n"
        "\n"
        "dma_test                          Test DMA.\n"
        "scratch_test                      Test Scratch register.\n"
        "\n"
#ifdef CSR_FLASH_BASE
        "flash_write filename [offset]     Write file contents to SPI Flash.\n"
        "flash_read filename size [offset] Read from SPI Flash and write contents to file.\n"
        "flash_reload                      Reload FPGA Image.\n"
#endif
    );
    exit(1);
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

    /* Parameters. */
    int argIdx = 1;
/**    for (;;) {
        c = getopt(argc, argv, "hc:w:zea");
        if (c == -1)
            break;
        switch (c) {
        case 'h':
            help();
            break;
        case 'c':
            litepcie_device_num = atoi(optarg);
            break;
        case 'w':
            litepcie_data_width = atoi(optarg);
            break;
        case 'z':
            litepcie_device_zero_copy = 1;
            break;
        case 'e':
            litepcie_device_external_loopback = 1;
            break;
        case 'a':
            litepcie_auto_rx_delay = 1;
            break;
        default:
            exit(1);
        }
    }
    */
    /* Show help when too much args. */
    if (argc < 2)
        help();

    /* Select device. */
    //getDeviceName(litepcie_device, 1024);

    cmd = argv[argIdx];

    /* Info cmds. */
    if (!strcmp(cmd, "info"))
        info();
    /* Scratch cmds. */
    else if (!strcmp(cmd, "scratch_test"))
        scratch_test();
    /* SPI Flash cmds. */
#ifdef FLASH_EN
#if CSR_FLASH_BASE
    else if (!strcmp(cmd, "flash_write")) {
        const char* filename;
        uint32_t offset = 0;
        if (argIdx + 1 > argc)
            goto show_help;
        filename = argv[argIdx++];
        if (argIdx < argc)
            offset = strtoul(argv[argIdx++], NULL, 0);
        flash_write(filename, offset);
    }
    else if (!strcmp(cmd, "flash_read")) {
        const char* filename;
        uint32_t size = 0;
        uint32_t offset = 0;
        if (argIdx + 2 > argc)
            goto show_help;
        filename = argv[argIdx++];
        size = strtoul(argv[argIdx++], NULL, 0);
        if (argIdx < argc)
            offset = strtoul(argv[argIdx++], NULL, 0);
        flash_read(filename, size, offset);
    }
    else if (!strcmp(cmd, "flash_reload"))
        flash_reload();
#endif
#endif
#ifdef DMA_EN
    /* DMA cmds. */
    else if (!strcmp(cmd, "dma_test"))
        dma_test(
            litepcie_device_zero_copy,
            litepcie_device_external_loopback,
            litepcie_data_width,
            litepcie_auto_rx_delay);
#endif
    /* Show help otherwise. */
    else
        goto show_help;

    return 0;

show_help:
    help();

    return 0;
}
