/*
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "inference_thread_interface.h"
#include "inference/ei_run_impulse.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#define INFERENCE_TASK_STACK_SIZE_BYTE        (4096u)
#define INFERENCE_TASK_PRIORITY               (4u)

/* FreeRTOS module */
static TaskHandle_t inference_thread_handle = NULL;
static void inference_thread_entry(void *pvParameters);

/**
 * @brief start inference thread
 * @return
 */
bool start_inference_thread(void)
{
    BaseType_t retval;

    if (inference_thread_handle != NULL) {
        ei_printf("Inference Thread already running!\r\n");
        return false;
    }

    /* create a task to send data via usb */
    retval = xTaskCreate(inference_thread_entry,
        (const char*) "Inference Thread",
        INFERENCE_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        INFERENCE_TASK_PRIORITY, //uxPriority
        &inference_thread_handle);

    return (pdPASS == retval);
}

/**
 *
 * @param pvParameters
 */
static void inference_thread_entry(void *pvParameters)
{
    (void)pvParameters;

    do {
        ei_run_impulse();
    }while(is_inference_running() == true);

    if (inference_thread_handle != NULL) {
        inference_thread_handle = NULL;
        vTaskDelete(NULL);
    }

    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
