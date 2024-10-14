/*
 * Copyright (c) 2022 EdgeImpulse Inc.
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
/* Include ----------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "task.h"

#include <cmath>
#include "ei_at_handlers.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ei_fusion.h"
#include "ei_device_lib.h"
#include "inference/ei_run_impulse.h"
#include "firmware-sdk/at-server/ei_at_command_set.h"
#include "firmware-sdk/ei_image_lib.h"
#include "ei_flash_memory.h"
#include "lib/xmodem/xmodem.h"
#include "peripheral/board_ctrl.h"
#include "peripheral/flash_handler.h"
#include "ingestion-sdk-platform/brickml/ei_sd_memory.h"
#include "ingestion_thread.h"
#include "at_base64_lib.h"
#include "comms.h"

#define AT_UPDATEFIRMWARE               "UPDATEFIRMWARE"
#define AT_UPDATEFILE_HELP_TEXT         "Update firmware"

#define AT_SETSTORAGESD                 "SETSTORAGESD"
#define AT_SETSTORAGESD_HELP_TEXT       "Set storage on SD or flash"
#define AT_SETSTORAGESD_ARGS            "SET_SD or SET_FLASH"

EiBrickml *pei_device;

/* Private function declaration */
static bool at_list_config(void);
static bool at_clear_config(void);

static bool at_device_info(void);
static bool at_get_sample_settings(void);
static bool at_set_sample_settings(const char **argv, const int argc);
static bool at_get_upload_settings(void);
static bool at_set_upload_settings(const char **argv, const int argc);
static bool at_set_upload_host(const char **argv, const int argc);
static bool at_list_sensors(void);
static bool at_list_fusion_sensors(void);

static bool at_read_buffer(const char **argv, const int argc);

static bool at_sample_start(const char **argv, const int argc);

static bool at_unlink_file(const char **argv, const int argc);
static bool at_read_raw(const char **argv, const int argc);

static bool at_run_nn_normal(void);
static bool at_run_nn_normal_cont(void);
static bool at_run_impulse_debug(const char **argv, const int argc);

static bool at_get_mgmt_settings(void);
static bool at_set_mgmt_settings(const char **argv, const int argc);

static bool at_update_firmware(void);

static bool at_set_sd_storage(const char **argv, const int argc);
static bool at_set_ingestion_settings(const char **argv, const int argc);
static bool at_get_ingestion_settings(void);
static bool at_start_ingestion(void);

static inline bool check_args_num(const int &required, const int &received);
static bool local_read_encode_send_sample_buffer(size_t address, size_t length);

/* Public function definition */
/**
 *
 * @return
 */
ATServer *ei_at_init(EiBrickml *ei_device)
{
    ATServer *at;

    at = ATServer::get_instance();
    pei_device = ei_device;

    at->register_command(AT_CONFIG, AT_CONFIG_HELP_TEXT, nullptr, at_list_config, nullptr, nullptr);
    at->register_command(AT_CLEARCONFIG, AT_CLEARCONFIG_HELP_TEXT, at_clear_config, nullptr, nullptr, nullptr);
    at->register_command(AT_SAMPLESTART, AT_SAMPLESTART_HELP_TEXT, nullptr, nullptr, at_sample_start, AT_SAMPLESTART_ARGS);
    at->register_command(AT_SAMPLESETTINGS, AT_SAMPLESETTINGS_HELP_TEXT, nullptr, at_get_sample_settings, at_set_sample_settings, AT_SAMPLESETTINGS_ARGS);
    at->register_command(AT_RUNIMPULSE, AT_RUNIMPULSE_HELP_TEXT, at_run_nn_normal, nullptr, nullptr, nullptr);
    at->register_command(AT_RUNIMPULSEDEBUG, AT_RUNIMPULSEDEBUG_HELP_TEXT, nullptr, nullptr, at_run_impulse_debug, AT_RUNIMPULSEDEBUG_ARGS);
    at->register_command(AT_RUNIMPULSECONT, AT_RUNIMPULSECONT_HELP_TEXT, at_run_nn_normal_cont, nullptr, nullptr, nullptr);
    at->register_command(AT_READBUFFER, AT_READBUFFER_HELP_TEXT, nullptr, nullptr, at_read_buffer, AT_READBUFFER_ARGS);
    at->register_command(AT_MGMTSETTINGS, AT_MGMTSETTINGS_HELP_TEXT, nullptr, at_get_mgmt_settings, at_set_mgmt_settings, AT_MGMTSETTINGS_ARGS);
    at->register_command(AT_UPLOADSETTINGS, AT_UPLOADSETTINGS_HELP_TEXT, nullptr, at_get_upload_settings, at_set_upload_settings, AT_UPLOADSETTINGS_ARGS);
    at->register_command(AT_UPLOADHOST, AT_UPLOADHOST_HELP_TEXT, nullptr, nullptr, at_set_upload_host, AT_UPLOADHOST_ARGS);
    at->register_command(AT_READRAW, AT_READRAW_HELP_TEXT, nullptr, nullptr, at_read_raw, AT_READRAW_ARS);
    at->register_command(AT_UNLINKFILE, AT_UNLINKFILE_HELP_TEXT, nullptr, nullptr, at_unlink_file, AT_UNLINKFILE_ARGS);
    at->register_command(AT_UPDATEFIRMWARE, AT_UPDATEFILE_HELP_TEXT, at_update_firmware, nullptr, nullptr, nullptr);
    at->register_command(AT_SETSTORAGESD, AT_SETSTORAGESD_HELP_TEXT, nullptr, nullptr, at_set_sd_storage, AT_SETSTORAGESD_ARGS);
    at->register_command(AT_STARTINGESTIONCYCLE, AT_STARTINGESTIONCYCLE_HELP_TEXT, at_start_ingestion, nullptr, nullptr, nullptr);
    at->register_command(AT_INGESTIONCYCLESETTINGS, AT_INGESTIONCYCLESETTINGS_HELP_TEXT, nullptr, at_get_ingestion_settings, at_set_ingestion_settings, AT_INGESTIONCYCLESETTINGS_ARGS);
    
    return at;
}

/* Private function definition */
/**
 *
 * @return
 */
static bool at_list_config(void)
{
    ei_printf("===== Device info =====\n");
    at_device_info();
    ei_printf("\n");
    ei_printf("===== Sensors ======\n");
    at_list_sensors();
    at_list_fusion_sensors();
    ei_printf("\n");
    ei_printf("===== Snapshot ======\n");
    ei_printf("\n");
    ei_printf("===== WIFI =====\n");
    ei_printf("SSID:      \n");
    ei_printf("Password:  \n");
    ei_printf("Security:  0\n");
    ei_printf("MAC:       00:00:00:00:00:00\n");
    ei_printf("Connected: 0\n");
    ei_printf("Present:   0\n");
    ei_printf("\n");
    ei_printf("===== Sampling parameters =====\n");
    at_get_sample_settings();
    ei_printf("\n");
    ei_printf("===== Ingestion parameters =====\n");
    at_get_ingestion_settings();
    ei_printf("\n");
    ei_printf("===== Upload settings =====\n");
    at_get_upload_settings();
    ei_printf("\n");
    ei_printf("===== Remote management =====\n");
    at_get_mgmt_settings();
    ei_printf("\n");

    return true;
}

/**
 *
 * @return
 */
static bool at_device_info(void)
{
    bool ret_val = false;

    if (pei_device != nullptr)
    {
        ei_printf("ID:         %s\n", pei_device->get_device_id().c_str());
        ei_printf("Type:       %s\n", pei_device->get_device_type().c_str());
        ei_printf("AT Version: %s\n", AT_COMMAND_VERSION);
        ei_printf("Data Transfer Baudrate: %lu\n", pei_device->get_data_output_baudrate());
        ret_val = true;
    }
    else
    {

    }

    return ret_val;
}

/**
 *
 * @return
 */
static bool at_clear_config(void)
{
    bool ret_val = false;

    if (pei_device != nullptr)
    {
        ei_printf("Clearing config and restarting system...\n");
        pei_device->clear_config();
        //pei_device->init_device_id(); // done in clear config
        ret_val = true;
    }
    else
    {

    }


    return ret_val;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_sample_start(const char **argv, const int argc)
{
    if(argc < 1) {
        ei_printf("Missing sensor name!\n");
        return true;
    }

    const ei_device_sensor_t *sensor_list;
    size_t sensor_list_size;

    pei_device->get_sensor_list((const ei_device_sensor_t **)&sensor_list, &sensor_list_size);

    for (size_t ix = 0; ix < sensor_list_size; ix++) {
        if (strcmp(sensor_list[ix].name, argv[0]) == 0) {
            pei_device->set_sensor_label(argv[0], true);
            if (!sensor_list[ix].start_sampling_cb()) {
                ei_printf("ERR: Failed to start sampling\n");
            }
            return true;
        }
    }

    if (ei_connect_fusion_list(argv[0], SENSOR_FORMAT)) {
        pei_device->set_sensor_label(argv[0], true);
        if (!ei_fusion_setup_data_sampling()) {
            ei_printf("ERR: Failed to start sensor fusion sampling\n");
        }
    }
    else {
        ei_printf("ERR: Failed to find sensor '%s' in the sensor list\n", argv[0]);
    }


    return true;
}

/**
 *
 * @return
 */
static bool at_get_sample_settings(void)
{
    bool ret_val = false;

    if (pei_device != nullptr)
    {
        ei_printf("Label:     %s\n", pei_device->get_sample_label().c_str());
        ei_printf("Interval:  ");
        ei_printf_float(pei_device->get_sample_interval_ms());
        ei_printf("ms.\n");
        ei_printf("Length:    %lu ms.\n", pei_device->get_sample_length_ms());
        ei_printf("HMAC key:  %s\n", pei_device->get_sample_hmac_key().c_str());
        ret_val = true;
    }
    else
    {

    }

    return ret_val;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_set_sample_settings(const char **argv, const int argc)
{
    if ((argc < 3) || (pei_device == nullptr)) {
        ei_printf("Missing argument! Required: " AT_SAMPLESETTINGS_ARGS "\n");
        return false;
    }

    pei_device->set_sample_label(argv[0], false);

    //TODO: sanity check and/or exception handling
    std::string interval_ms_str(argv[1]);
    pei_device->set_sample_interval_ms(stof(interval_ms_str), false);

    //TODO: sanity check and/or exception handling
    std::string sample_length_str(argv[2]);
    pei_device->set_sample_length_ms(stoi(sample_length_str), false);

    if(argc >= 4) {
        pei_device->set_sample_hmac_key(argv[3], false);
    }

    pei_device->save_config();

    ei_printf("OK\n");

    return true;
}

/**
 * @brief Handler for RUNIMPULE
 *
 * @return
 */
static bool at_run_nn_normal(void)
{
    ei_start_impulse(false, false, false);

    return (is_inference_running());
}

/**
 * @brief
 *
 * @param argv
 * @param argc
 * @return true
 * @return false
 */
static bool at_run_impulse_debug(const char **argv, const int argc)
{
    bool use_max_uart_speed = false;
    if (argc > 0 && argv[0][0] == 'y') {
        use_max_uart_speed = true;
    }

    ei_start_impulse(false, true, use_max_uart_speed);

    return true;
}

/**
 * @brief Handler for RUNIMPULSECONT
 *
 * @return
 */
static bool at_run_nn_normal_cont(void)
{
    ei_start_impulse(true, false, false);

    return (is_inference_running());
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_read_buffer(const char **argv, const int argc)
{
    bool success = false;

    if(argc < 2) {
        ei_printf("Missing argument! Required: " AT_READBUFFER_ARGS "\n");
        return true;
    }

    if (pei_device != nullptr)
    {
        pei_device->set_state(eiBrickMLStateUploading);

        size_t start = (size_t)atoi(argv[0]);
        size_t length = (size_t)atoi(argv[1]);

        bool use_max_baudrate = false;
        if (argc >= 3 && argv[2][0] == 'y') {
           use_max_baudrate = true;
        }

        if (use_max_baudrate) {
            ei_printf("OK\r\n");
            ei_sleep(100);
            pei_device->set_max_data_output_baudrate();
            do {
                ei_sleep(50);
            }while(pei_device->is_max_baudrate() == false);
        }
        
        success = local_read_encode_send_sample_buffer(start, length);

        if (use_max_baudrate) {
            ei_printf("\r\nOK\r\n");
            ei_sleep(250);
            pei_device->set_default_data_output_baudrate();
            ei_sleep(250);
        }

        if (!success) {
            ei_printf("Failed to read from buffer\n");
        }
        else {
            ei_printf("\n");
        }
    }
    pei_device->set_state(eiBrickMLStateIdle);

    return success;
}

/**
 *
 * @return
 */
static bool at_get_mgmt_settings(void)
{
    ei_printf("%s\n", pei_device->get_management_url().c_str());

    return true;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_set_mgmt_settings(const char **argv, const int argc)
{
    bool ret_val = false;

    if (check_args_num(1, argc) == false) {
        return true;
    }

    if (pei_device != nullptr)
    {
        pei_device->set_management_url(argv[0], false);
        ei_printf("OK\n");

        ret_val = true;
    }

    pei_device->save_config();

    return ret_val;
}

/**
 *
 * @return
 */
static bool at_list_sensors(void)
{
    bool ret_val = false;
    const ei_device_sensor_t *list;
    size_t list_size;

    if (pei_device != nullptr)
    {
        int r = pei_device->get_sensor_list((const ei_device_sensor_t **)&list, &list_size);
        if (r != 0) {
            ei_printf("Failed to get sensor list (%d)\n", r);
            return true;
        }
        ret_val = true;

        for (size_t ix = 0; ix < list_size; ix++) {
            ei_printf(
                "Name: %s, Max sample length: %hus, Frequencies: [",
                list[ix].name,
                list[ix].max_sample_length_s);
            for (size_t fx = 0; fx < EI_MAX_FREQUENCIES; fx++) {
                if (list[ix].frequencies[fx] != 0.0f) {
                    if (fx != 0) {
                        ei_printf(", ");
                    }
                    ei_printf_float(list[ix].frequencies[fx]);
                    ei_printf("Hz");
                }
            }
            ei_printf("]\n");
        }
    }
    else
    {

    }

    return ret_val;
}

/**
 *
 * @return
 */
static bool at_list_fusion_sensors(void)
{
    ei_built_sensor_fusion_list();

    return true;
}

/**
 *
 * @return
 */
static bool at_get_upload_settings(void)
{
    bool ret_val = false;

    if (pei_device != nullptr)
    {
        ei_printf("Api Key:   %s\n", pei_device->get_upload_api_key().c_str());
        ei_printf("Host:      %s\n", pei_device->get_upload_host().c_str());
        ei_printf("Path:      %s\n", pei_device->get_upload_path().c_str());

        ret_val = true;
    }
    else
    {

    }

    return ret_val;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_set_upload_settings(const char **argv, const int argc)
{
    bool ret_val = false;

    if (check_args_num(2, argc) == false) {
        return false;
    }
    if (pei_device != nullptr)
    {
        pei_device->set_upload_api_key(argv[0], false);
        pei_device->set_upload_host(argv[1], false);

        ret_val = true;
    }

    pei_device->save_config();
    ei_printf("OK\n");

    return ret_val;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_set_upload_host(const char **argv, const int argc)
{
    bool ret_val = false;

    if (check_args_num(1, argc) == false) {
        return false;
    }

    if (pei_device != nullptr)
    {
        pei_device->set_upload_host(argv[0], false);
        ret_val = true;
    }

    pei_device->save_config();
    ei_printf("OK\n");

    return ret_val;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_read_raw(const char **argv, const int argc)
{
    EiDeviceMemory* mem = pei_device->get_device_memory_in_use();

    if(argc < 2) {
        ei_printf("Missing argument! Required: " AT_READBUFFER_ARGS "\n");
        return true;
    }

    volatile uint32_t start = (uint32_t)atoi(argv[0]);
    volatile uint32_t length = (uint32_t)atoi(argv[1]);

    unsigned char buffer[32];

    pei_device->set_state(eiBrickMLStateUploading);

    for(; (start < length); start += 32)
    {
        mem->read_sample_data(buffer, start, 32);

        int n_display_bytes = (length - start) < 32 ? (length - start) : 32;
        for(int i=0; i < n_display_bytes; i++)
        {
            ei_printf("%.2X ", (unsigned char)buffer[i]);
        }
        ei_printf("\b\r\n");

        if (start > length)
            return true;
    }

    pei_device->set_state(eiBrickMLStateIdle);

    return true;
}


/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_unlink_file(const char **argv, const int argc)
{
    ei_printf("\n");

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
static bool at_update_firmware(void)
{
    unsigned char retval = 0;
    ei_printf("Ready to update firmware\n");

    pei_device->set_state(eiBrickMLStateUploading);
    flash_handler_init();
    retval = XmodemDownloadAndProgramFlash(SECONDARY_IMAGE_START_ADDRESS);
    flash_handler_deinit();
    pei_device->set_state(eiBrickMLStateIdle);
    ei_printf("\r\nUpdate procedure exit with ret code: %d\r\n", retval);

    ei_printf("\n");

    return true;
}

/**
 *
 * @param argv
 * @param argc
 * @return
 */
static bool at_set_sd_storage(const char **argv, const int argc)
{
    bool to_set = false;
    bool ret_val = false;

    if (check_args_num(1, argc) == false) {
        return false;
    }
    if (pei_device != nullptr) {
        if (strcmp(argv[0], "SET_SD") == 0) {
            to_set = true;
        }
        pei_device->set_sd_storage(to_set);

        ret_val = true;
    }

    ei_printf("OK\n");

    return ret_val;
}

/**
 * @brief 
 * 
 * @param argv 
 * @param argc 
 * @return true 
 * @return false 
 */
static bool at_start_ingestion(void)
{
    if (pei_device->is_sd_present() == true) {
        ingestion_thread_start();

        ei_printf("OK\n");
    }
    else {
        ei_printf("SD not found\r\n");
    }

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
static bool at_get_ingestion_settings(void)
{
    bool ret_val = false;

    if (pei_device != nullptr) {
        ei_printf("Sensor:         %s\n", pei_device->get_sensor_label().c_str());
        ei_printf("Cycle length:   %lu ms\n", pei_device->get_long_recording_length_ms());
        ei_printf("Cycle interval: %lu ms.\n", pei_device->get_long_recording_interval_ms());
        ret_val = true;
    }
    else {

    }

    return ret_val;
}

/**
 * @brief 
 * 
 * @param argv 
 * @param argc 
 * @return true 
 * @return false 
 */
static bool at_set_ingestion_settings(const char **argv, const int argc)
{
    uint32_t sampling_time_ms = (uint32_t)atoi(argv[1]);
    uint32_t interval_ms = (uint32_t)atoi(argv[2]);

    if (check_args_num(3, argc) == false) {
        return false;
    }

    if (interval_ms > sampling_time_ms ) {
        ei_printf("Error: interval can't be larger than the total sampling time/r/n");
        return false;
    }

    pei_device->set_sensor_label(argv[0], false);

    pei_device->set_long_recording_length_ms(sampling_time_ms, false);
    pei_device->set_long_recording_interval_ms(interval_ms, false);

    pei_device->save_config();
    ei_printf("OK\n");

    return true;
}

static inline bool check_args_num(const int &required, const int &received)
{
    if (received < required) {
        ei_printf("Too few arguments! Required: %d\n", required);
        return false;
    }

    return true;
}

/**
 * @brief 
 * 
 * @param address 
 * @param length 
 * @return true 
 * @return false 
 */
static bool local_read_encode_send_sample_buffer(size_t address, size_t length)
{
    EiDeviceMemory* mem = pei_device->get_device_memory_in_use();

    if (pei_device->is_sd_in_use()) {
        //
        EiSDMemory* sd = static_cast<EiSDMemory*>(mem);
        sd->open_latest_file(false);
    }

    // we are encoiding data into base64, so it needs to be divisible by 3
    const int buffer_size = 513;
    uint8_t* buffer = (uint8_t*)ei_malloc(buffer_size);

    size_t output_size_check = floor(buffer_size / 3 * 4);
    size_t mod = buffer_size % 3;
    uint32_t size_out = 0;

    output_size_check += mod;
    uint8_t* buffer_out = (uint8_t*)ei_malloc(output_size_check);

    while (1) {
        size_t bytes_to_read = buffer_size;

        if (bytes_to_read > length) {
            bytes_to_read = length;
        }

        if (bytes_to_read == 0) {
            ei_free(buffer);
            ei_free(buffer_out);
            if (pei_device->is_sd_in_use()) {
                //
                EiSDMemory* sd = static_cast<EiSDMemory*>(mem);
                sd->close_sample_file();
            }
            return true;
        }

        if (mem->read_sample_data(buffer, address, bytes_to_read) != bytes_to_read) {
            ei_free(buffer);
            ei_free(buffer_out);
            if (pei_device->is_sd_in_use()) {
                //
                EiSDMemory* sd = static_cast<EiSDMemory*>(mem);
                sd->close_sample_file();
            }
            return false;
        }

        size_out = base64_encode_buffer((char *)buffer, bytes_to_read, (char *)buffer_out, output_size_check);
        comms_send((uint8_t*)buffer_out, size_out, 100);   // direct write to speed up

        address += bytes_to_read;
        length -= bytes_to_read;
    }

    return true;
}
