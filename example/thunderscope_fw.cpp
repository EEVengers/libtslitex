/**
 * This file is part of libtslitex.
 * Simple CLI tool for managing the firmware on Thunderscope.
 *
 * Copyright (c) 2024 Nate Meyer <nate.devel@gmail.com>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <iostream>
#include <fstream>
#include <filesystem>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <fcntl.h>
#include <thread>
#include <math.h>

#define OPTPARSE_IMPLEMENTATION
#include "optparse.h"

#include <thunderscope.h>

#define MAX_USER_DATA_SIZE  (0x500000)

static void user_write(tsHandle_t ts, const char* file_path, uint32_t offset)
{
    std::error_code err;
    auto file_size = std::filesystem::file_size(file_path, err);
    if(err)
    {
        std::cout << "Error opening \"" << file_path << "\" : " << err.message() << "\r\n";
        return;
    }

    int32_t status;
    // Open data file
    std::ifstream file(file_path, std::ios::binary);
    if(file)
    {
        char* bitstream = new char[file_size];
        file.read(bitstream, file_size);

        printf("User Data write in progress....");

        // Load New bitstream
        status = thunderscopeUserDataWrite(ts, bitstream, offset, (uint32_t)file_size);

        printf("Data Write %dB Complete!\r\n", status);

        // Close File
        file.close();
        delete bitstream;
    }
    else
    {
        printf("ERROR User Data Write: Failed to open file %s\r\n", file_path);
    }
}

static void user_read(tsHandle_t ts, const char* file_path)
{
    int32_t read_len = 0;
    // Open file to store data
    std::ofstream file(file_path, std::ios::binary);
    if(file)
    {
        char* data_buffer = new char[MAX_USER_DATA_SIZE];
        
        printf("Reading User Data ....");
        
        // Get data from TS
        read_len = thunderscopeUserDataRead(ts, data_buffer, 0, MAX_USER_DATA_SIZE);
        file.write(data_buffer, read_len);
        
        printf("Read %dB Complete!\r\n", read_len);

        // Close File
        file.flush();
        file.close();
        delete data_buffer;
    }
    else
    {
        printf("ERROR User Data Read: Failed to open file %s\r\n", file_path);
    }
}

static void fw_upgrade(tsHandle_t ts, const char* file_path)
{
    std::error_code err;
    auto file_size = std::filesystem::file_size(file_path, err);
    if(err)
    {
        std::cout << "Error opening \"" << file_path << "\" : " << err.message() << "\r\n";
        return;
    }

    // Open Bitstream File
    std::ifstream file(file_path, std::ios::binary);
    if(file)
    {
        char* bitstream = new char[file_size];
        file.read(bitstream, file_size);

        printf("Gateware update in progress....");

        // Load New bitstream
        thunderscopeFwUpdate(ts, bitstream, file_size);

        printf("Update Complete!\r\n");

        // Close File
        file.close();
        delete bitstream;
    }
    else
    {
        printf("ERROR FW Update: Failed to open file %s\r\n", file_path);
    }
}

static void fw_restore(tsHandle_t ts)
{

    
}

static void print_help(void)
{
    printf("TS FW Update Util Usage:\r\n");
    printf("\tuser_read [dest_file] - Save user data to a file\r\n");
    printf("\t\t[dest_file] - Name of the file to save\r\n");
    printf("\tuser_write [data_file] - Load user data to the Thunderscope\r\n");
    printf("\t\t[data_file] - Name of the XML Calibration file to load\r\n");
    printf("\tfw_upgrade [bitstream_file] - Load a new Bitstream\r\n");
    printf("\t\t[bitstream_file] - Name of the Gateware Bitstream file to load\r\n");
    printf("\tfactory_restore - Restores the factory bitstream to the primary location\r\n");
    printf("\tinfo - Print FPGA Information and Exit\r\n");
    printf("\thelp - Print this help message\r\n");
    printf("\tCommon Options:\r\n");
    printf("\t\t-d <device>      Device Index\r\n");
}

int main(int argc, char** argv)
{
    const char* file_path;
    uint32_t idx = 0;
    int i;
    int32_t result;
    tsHandle_t ts;
    tsDeviceInfo_t infos;
    tsScopeState_t status;
    uint32_t offs = 0;

    struct optparse_long argList[] = {
        {"dev",      'd', OPTPARSE_REQUIRED},
        {"offs",     'o', OPTPARSE_REQUIRED},
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
        case 'd':
            idx = strtol(options.optarg, NULL, 0);
            argCount+=2;
            break;
        case 'o':
            offs = strtol(options.optarg, NULL, 0);
            argCount += 2;
            break;
        case '?':
            fprintf(stderr, "%s: %s\n", argv[0], options.errmsg);
            print_help();
            exit(EXIT_FAILURE);
        }
    }
    arg = argv[argCount];

    if(argCount++ == argc)
    {
        print_help();
        exit(EXIT_FAILURE);
    }

    if(0 == strcmp(arg, "help"))
    {
        print_help();
        exit(0);
    }

    if(0 == strcmp(arg, "list"))
    {
        uint32_t i = 0;
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

    result = thunderscopeListDevices(idx, &infos);
    if(result == TS_STATUS_ERROR)
    {
        fprintf(stderr, "Could not query device info\n");
        exit(EXIT_FAILURE);
    }

    ts = thunderscopeOpen(idx, true);
    if(ts == NULL) {
        fprintf(stderr, "Could not init driver\n");
        exit(EXIT_FAILURE);
    }

    printf("\x1b[1m[> Device Information:\x1b[0m\n");
    printf("------------------------\n");
    printf("FPGA Identifier:  %s.\n", infos.identity);
    if(infos.hw_id & TS_HW_ID_VALID_MASK)
    {
        printf("HW Rev %02d - %s\n", infos.hw_id & TS_HW_ID_REV_MASK, 
            (infos.hw_id & TS_HW_ID_VARIANT_MASK) ? "TB" : "PCIe" );
    }
    else
    {
        printf("HW Rev Beta\n");
    }
    printf("Gateware Rev  0x%08X\n", infos.gw_id);
    printf("LiteX Release 0x%08X\n", infos.litex);

    if(0 == strcmp(arg, "factory_restore"))
    {
        fw_restore(ts);
    }
    else if(0 == strcmp(arg, "info"))
    {
        //Nothing else to do
        ;;
    }
    else if(argCount == argc)
    {
        printf("**Missing File path argument**\r\n");
        print_help();
    }
    else
    {
        file_path = argv[argCount];

        if(0 == strcmp(arg, "user_write"))
        {
            user_write(ts, file_path, offs);
        }
        // Setup Channel, record samples to buffer, save buffer to file
        else if(0 == strcmp(arg, "user_read"))
        {
            user_read(ts, file_path);
        }
        else if(0 == strcmp(arg, "fw_upgrade"))
        {
            fw_upgrade(ts, file_path);
        }
        //Print Help
        else
        {
            print_help();
        }
    }

    /* Close LitePCIe device. */
    thunderscopeClose(ts);

    return 0;
}