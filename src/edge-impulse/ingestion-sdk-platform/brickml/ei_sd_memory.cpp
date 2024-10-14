/*
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "FreeRTOS.h"
#include "semphr.h"
#include "hal_data.h"

#include <brickml/ei_sd_memory.h>
#include "peripheral/sd/sd_handler.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/**
 * @brief Construct a new Ei SD Memory:: Ei SD Memory object
 * 
 */
EiSDMemory::EiSDMemory():
    EiDeviceMemory(0, 1, (0x800000), 512)
{
    used_blocks = 0;    // config is done in data flash (for now)

    total_file = 0;
    sd_card_inserted = false;
    file_is_open = false;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiSDMemory::init(void)
{
#if (BRICKML_SOM == 0)
    sd_card_inserted = sd_handler_is_present();
    if (sd_card_inserted == true) {
        memory_size = sd_handler_get_sectors();
    }
#else
    sd_card_inserted = false;
#endif

    return sd_card_inserted;
}

/**
 * @brief 
 * 
 * @param data 
 * @param address 
 * @param num_bytes 
 * @return uint32_t 
 */
uint32_t EiSDMemory::read_data(uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    // address
    uint32_t read_bytes = 0;

    sd_handler_read_file(data, num_bytes, &read_bytes);

    return read_bytes;
}

/**
 * @brief 
 * 
 * @param data 
 * @param address 
 * @param num_bytes 
 * @return uint32_t 
 */
uint32_t EiSDMemory::write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    // address ?

    return sd_handler_write_file(data, num_bytes);
}

/**
 * @brief 
 * 
 * @param address 
 * @param num_bytes 
 * @return uint32_t 
 */
uint32_t EiSDMemory::erase_data(uint32_t address, uint32_t num_bytes)
{

    return num_bytes;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiSDMemory::close_sample_file(void)
{
    return sd_handler_close_file();
}

/**
 * @brief 
 * 
 * @param filename 
 * @param write 
 * @return true 
 * @return false 
 */
bool EiSDMemory::open_sample_file(char* filename, bool write)
{
    bool created = false;
    if (sd_card_inserted == false){
		return false;
    }

    char path_filename[128];
    strcpy(path_filename, drive);
    strcat(path_filename, filename);

    created = sd_handler_open_file(path_filename, (uint8_t)write);

    if (created == true) {
        latest_file = path_filename;
        file_is_open = true;
    }
    else {
        latest_file.clear();
    }

    return created;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiSDMemory::read_content(void)
{

    return false;
}

uint32_t EiSDMemory::test(void)
{
    uint32_t ret = 0;
    uint8_t write_test[512] = {0};
    uint8_t read_test[1024] = {0};
    uint16_t i;

    for (i = 0; i < 512; i++) {
        write_test[i] = (uint8_t)i;
    }

    ret = open_sample_file("/sd0/testfile.txt", true);
    ei_printf("open_sample_file %d\r\n", ret);
    ret = write_data(write_test, 0, 512);
    ei_printf("write_data %d\r\n", ret);
    close_sample_file();

    ret = open_sample_file("/sd0/testfile.txt", false);
    ei_printf("open_sample_file %d\r\n", ret);
    ret = read_sample_data(read_test, 0, 1024);
    ei_printf("read_test %d\r\n", ret);
    close_sample_file();

    if (ret > 0) {
        for (i = 0; i < ret; i++) {
            ei_printf("%d ", read_test[i]);
            if (i%10 == 0) {
                ei_printf("\r\n");
            }
        }
    }

    return ret;
}

/**
 * @brief 
 * 
 */
bool EiSDMemory::setup_sampling(const char* sensor_name, const char* lable_name)
{
    char filename[256] = {0};
    uint16_t file_number;

    snprintf(filename, 256, "%s_%s", sensor_name, lable_name);
    file_number = get_file_number(filename);

    snprintf(filename, 256, "%s_%s_%d.cbor", sensor_name, lable_name, file_number);
    
    ei_printf("Filename on SD: %s\r\n", filename);

    //get how many ?
    return open_sample_file((char*)filename, true); // filename, write mode
}

/**
 * @brief 
 * 
 */
void EiSDMemory::finalize_samplig(void)
{
    close_sample_file();
}

/**
 * @brief 
 * 
 * @param write 
 * @return true 
 * @return false 
 */
bool EiSDMemory::open_latest_file(bool write)
{
    bool fres;

    if (sd_card_inserted == false){
        return false;
    }

    if (latest_file.empty() == true) {
        return false;
    }

    fres = sd_handler_open_file(latest_file.c_str(), write);

    if (fres == true) {
        file_is_open = true;
    }
    else {
        latest_file.clear();
    }

    return fres;
}

/**
 * @brief Returns total number of file
 * 
 * @return uint16_t 
 */
uint16_t EiSDMemory::get_file_number(char* pattern)
{
    char filename[256] = {0};

    total_file = sd_handler_get_file_numer(drive, pattern);

    do {
        snprintf(filename, 256, "%s_%d.cbor", pattern, total_file);
        if (sd_handler_check_is_available(drive, filename) == false) {
            total_file++;
        }
        else {
            break;
        }
    }while(1);

    return total_file;
}
