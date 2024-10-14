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
#include "task.h"
#include "ingestion_thread.h"
#include "ingestion-sdk-platform/brickml/ei_device_brickml.h"
#include "ingestion-sdk-platform/brickml/ei_sd_memory.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ei_fusion.h"

#define INGESTION_TASK_PRIORITY             (configMAX_PRIORITIES - 4)
#define INGESTION_TASK_STACK_SIZE_BYTE      8192

/* FreeRTOS module */
static TaskHandle_t ingestion_thread_handle;

static void ingestion_thread_entry(void *pvParameters);

/**
 * @brief 
 * 
 * @param thread_param 
 * @return true 
 * @return false 
 */
bool ingestion_thread_start(void)
{
    BaseType_t retval;

    if (ingestion_thread_handle != NULL) {
        ei_printf("ERR: Ingestion task already running.\r\n");
        return false;
    }

    /* create a task to send data via usb */
    retval = xTaskCreate(ingestion_thread_entry,
        (const char*) "Ingestion Thread",
        INGESTION_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        INGESTION_TASK_PRIORITY, //uxPriority
        &ingestion_thread_handle);

    if (retval != pdTRUE) {
        ei_printf("ERR: Can't create ingestion task\r\n");
        while(1){};
    }

    return true;
}

/**
 * @brief 
 * 
 * @param pvParameters 
 */
static void ingestion_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED (pvParameters);
    EiBrickml*  pdev = static_cast<EiBrickml*>(EiDeviceInfo::get_device());
    bool was_sd;

    //TickType_t xLastWakeTime;
    TickType_t xFrequency = (TickType_t)(pdev->get_long_recording_interval_ms() * portTICK_PERIOD_MS);
    uint32_t total_samples = (pdev->get_long_recording_length_ms()/ pdev->get_long_recording_interval_ms());
    uint32_t actual_sample = 0;
    const char* sensor_name = pdev->get_sensor_label().c_str();

    was_sd = pdev->is_sd_in_use();
    pdev->set_sd_storage(true);     // set SD

    ei_printf("Start Ingestion thread\r\n");
    ei_printf("\tSensor: %s\r\n", sensor_name);
    ei_printf("\tLabel: %s\r\n", pdev->get_sample_label().c_str());
    ei_printf("\tTotal_samples: %lu\r\n", total_samples);
    ei_printf("\tTotal time: %lu\r\n", pdev->get_long_recording_length_ms());
    ei_printf("\tInterval: %lu\r\n", pdev->get_long_recording_interval_ms());

    if (total_samples == 0) {
        ei_printf("ERR: wrong sample length\n");
    }
    else {
        do{
            if (ei_connect_fusion_list(sensor_name, SENSOR_FORMAT)) {
                if (!ei_fusion_setup_data_sampling()) {
                    ei_printf("ERR: Failed to start sensor fusion sampling\n");
                    break;
                }
            }
            else {
                ei_printf("ERR: Failed to find sensor '%s' in the sensor list\n", sensor_name);
                break;  // stop. maybe check before...
            }

            vTaskDelay(xFrequency);
        } while(++actual_sample < total_samples);
    }

    ei_printf("End sampling, collected %ld\r\n", actual_sample);
    pdev->set_sd_storage(was_sd);   // back to old setting

    if (ingestion_thread_handle != NULL) {
        ingestion_thread_handle = NULL;
        vTaskDelete(NULL);
    }

    while (1)
    {
        /* we should never end here */
        vTaskDelay (portMAX_DELAY);
    }
}

