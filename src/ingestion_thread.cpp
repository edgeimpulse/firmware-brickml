/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

