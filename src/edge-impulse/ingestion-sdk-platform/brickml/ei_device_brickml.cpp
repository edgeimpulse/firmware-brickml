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


#include <string>

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"
#include "ei_device_brickml.h"
#include "ei_flash_memory.h"
#include "ei_qspi_memory.h"
#include "comms.h"
#include "peripheral/timer_handler.h"
#include "ingestion-sdk-platform/sensors/ei_microphone.h"
#include "board_leds.h"

#define MAX_BAUD_RATE (36846400u)
#define MIC_ENABLED (   1)

/******
 *
 * @brief EdgeImpulse Device structure and information
 *
 ******/

EiBrickml::EiBrickml(EiDeviceMemory* code_flash, EiDeviceMemory* data_flash_to_set)
{
    EiDeviceInfo::memory = code_flash;
    EiBrickml::data_flash = data_flash_to_set;

    init_device_id();

    load_config();

    device_type = "BRICKML";
    state = eiBrickMLStateIdle;
    sensors[MICROPHONE].name = "Microphone";
    
    sensors[MICROPHONE].frequencies[0] = 16000.0f;
    sensors[MICROPHONE].frequencies[1] = 150000.0f;
    sensors[MICROPHONE].start_sampling_cb = &ei_microphone_sample_start;

    sensors[MICROPHONE].max_sample_length_s = (uint16_t)(code_flash->get_available_sample_bytes() / (sensors[MICROPHONE].frequencies[1] * sizeof(microphone_sample_t)));
}

EiBrickml::~EiBrickml()
{

}

EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    /* Initializing EdgeImpulse classes here in order for
     * Flash memory to be initialized before mainloop start
     */
    static EiQspiMemory code_memory;
    static EiFlashMemory data_memory(e_flash_data, sizeof(EiConfig));                  /* code flash doesn't store config !*/
    static EiBrickml dev(&code_memory, &data_memory);

    return &dev;
}

void EiBrickml::init_device_id(void)
{
    const bsp_unique_id_t *pdev_id;
    char temp[20];

    pdev_id = R_BSP_UniqueIdGet();

    snprintf(temp, sizeof(temp), "%02x:%02x:%02x:%02x:%02x:%02x",
            pdev_id->unique_id_bytes[5],
            pdev_id->unique_id_bytes[4],
            pdev_id->unique_id_bytes[3],
            pdev_id->unique_id_bytes[2],
            pdev_id->unique_id_bytes[1],
            pdev_id->unique_id_bytes[0]);

    device_id = std::string(temp);
}

void EiBrickml::load_config(void)
{
    EiConfig buf;

    memset(&buf, 0, sizeof(EiConfig));
    data_flash->load_config((uint8_t *)&buf, sizeof(EiConfig)); /* load from data flash */

    if (buf.magic == 0xdeadbeef)
    {
        wifi_ssid = std::string(buf.wifi_ssid, 128);
        wifi_password = std::string(buf.wifi_password, 128);
        wifi_security = buf.wifi_security;
        sample_interval_ms = buf.sample_interval_ms;
        sample_length_ms = buf.sample_length_ms;
        sample_label = std::string(buf.sample_label, 128);
        sample_hmac_key = std::string(buf.sample_hmac_key, 33);
        upload_host = std::string(buf.upload_host, 128);
        upload_path = std::string(buf.upload_path, 128);
        upload_api_key = std::string(buf.upload_api_key, 128);
        management_url = std::string(buf.mgmt_url, 128);
    }
}

bool EiBrickml::save_config(void)
{
    EiConfig buf;

    memset(&buf, 0, sizeof(EiConfig));

    strncpy(buf.wifi_ssid, wifi_ssid.c_str(), 128);
    strncpy(buf.wifi_password, wifi_password.c_str(), 128);
    buf.wifi_security = wifi_security;
    buf.sample_interval_ms = sample_interval_ms;
    buf.sample_length_ms = sample_length_ms;
    strncpy(buf.sample_label, sample_label.c_str(), 128);
    strncpy(buf.sample_hmac_key, sample_hmac_key.c_str(), 33);
    strncpy(buf.upload_host, upload_host.c_str(), 128);
    strncpy(buf.upload_path, upload_path.c_str(), 128);
    strncpy(buf.upload_api_key, upload_api_key.c_str(), 128);
    strncpy(buf.mgmt_url, management_url.c_str(), 128);
    buf.magic = 0xdeadbeef;

    return data_flash->save_config((uint8_t *)&buf, sizeof(EiConfig)); /* save config in data flash memory */
}

void EiBrickml::clear_config(void)
{
    EiDeviceInfo::clear_config();

    init_device_id();
    save_config();
}

uint32_t EiBrickml::get_data_output_baudrate(void)
{
    return MAX_BAUD_RATE;
}

bool EiBrickml::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    this->is_sampling = true;
    this->sample_read_callback = sample_read_cb;
    this->sample_interval_ms = sample_interval_ms;
#if MULTI_FREQ_ENABLED == 1
    this->actual_timer = 0;
    this->fusioning = 1;        //
#endif

    ei_timer1_start((uint32_t)this->sample_interval_ms);

    return true;
}

#if MULTI_FREQ_ENABLED == 1
/**
 *
 * @param sample_read_cb
 * @param multi_sample_interval_ms
 * @param num_fusioned
 * @return
 */
bool EiBrickml::start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned)
{
    uint8_t i;
    uint8_t flag = 0;

    this->is_sampling = true;
    this->sample_multi_read_callback = sample_multi_read_cb;
    this->fusioning = num_fusioned;

    this->multi_sample_interval.clear();

    for (i = 0; i < num_fusioned; i++){
        this->multi_sample_interval.push_back(1.f/multi_sample_interval_ms[i]*1000.f);
    }

    this->sample_interval = ei_fusion_calc_multi_gcd(this->multi_sample_interval.data(), this->fusioning);

    /* force first reading */
    for (i = 0; i < this->fusioning; i++){
            flag |= (1<<i);
    }
    this->sample_multi_read_callback(flag);

    this->actual_timer = 0;
    ei_timer1_start((uint32_t)this->sample_interval);

    return true;
}
#endif

bool EiBrickml::stop_sample_thread(void)
{
    this->is_sampling = false;

    ei_timer1_stop();

    return true;
}

void EiBrickml::sample_thread(void)
{
#if MULTI_FREQ_ENABLED == 1
    if (this->fusioning == 1){
        if (_timer_1_set == true)
        {
            ei_timer1_stop();
            if (this->sample_read_callback != nullptr)
            {
                this->sample_read_callback();

                if (this->is_sampling == true)
                {
                    ei_timer1_start((uint32_t)this->sample_interval_ms);
                }
            }
        }
    }
    else{
        uint8_t flag = 0;
        uint8_t i = 0;

        this->actual_timer += (uint32_t)this->sample_interval;

        if (_timer_1_set == true)
        {
            ei_timer1_stop();

            for (i = 0; i < this->fusioning; i++){
                if (((uint32_t)(this->actual_timer % (uint32_t)this->multi_sample_interval[i])) == 0) {
                    flag |= (1<<i);
                }
            }

            if (this->sample_multi_read_callback != nullptr)
            {
                this->sample_multi_read_callback(flag);

                if (this->is_sampling == true)
                {
                    ei_timer1_start((uint32_t)this->sample_interval);
                }
            }
        }
    }
#else
    if (_timer_1_set == true)
    {
        ei_timer1_stop();
        if (this->sample_read_callback != nullptr)
        {
            this->sample_read_callback();

            if (this->is_sampling == true)
            {
                ei_timer1_start((uint32_t)this->sample_interval_ms);
            }
        }
    }

#endif
}



void EiBrickml::set_state(EiBrickMlState state)
{
    static EiBrickMlState local_state;

    this->state = (EiBrickMlState)state;
    local_state = state;
    xQueueSend(g_new_state_queue, &local_state, 0);
}

EiBrickMlState EiBrickml::get_state(void)
{
    return this->state;
}

/**
 *
 * @param sensor_list
 * @param sensor_list_size
 * @return
 */
bool EiBrickml::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
#if MIC_ENABLED == 1
    *sensor_list      = sensors;
    *sensor_list_size = EI_DEVICE_N_SENSORS;
#else
    *sensor_list_size = 0;
#endif

    return false;   // ?
}


bool EiBrickml::test_flash(void)
{
    static const uint32_t buffer_size = 10000;
    EiQspiMemory* mem_flash = static_cast<EiQspiMemory*>(this->get_memory());

    uint8_t pippo[buffer_size];
    uint8_t leggi_pippo[buffer_size];

    for (uint16_t i = 0; i< buffer_size; i++)
    {
        pippo[i] = (uint8_t)i;
    }

    if (mem_flash->erase_sample_data(0, buffer_size) != buffer_size) {
        return false;
    }

    if (mem_flash->write_sample_data(pippo, 0, buffer_size) != buffer_size) {
        return false;
    }
    mem_flash->write_residual();

    if (mem_flash->read_sample_data(&leggi_pippo[0], 0, buffer_size) != buffer_size) {
        return false;
    }

    for (uint16_t i = 0; i < buffer_size; i++)
    {
        if (pippo[i] != leggi_pippo[i]) {
            ei_printf("Error at %d expecting %d but found %d\n", i, pippo[i], leggi_pippo[i]);
            return false;
        }
    }

    return true;
}


void EiBrickml::set_default_data_output_baudrate(void)
{
    //usb_set_high_speed
}

void EiBrickml::set_max_data_output_baudrate(void)
{
    //usb_set_high_speed();
}

bool EiBrickml::is_max_baudrate(void)
{
    uint32_t actual_baud;
    actual_baud = usb_get_speed();
    return (actual_baud == MAX_BAUD_RATE);
}
