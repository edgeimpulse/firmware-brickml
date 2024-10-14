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

#include "FreeRTOS.h"
#include "task.h"
#include "usb_thread_interface.h"
#include "comms.h"
#include "portmacro.h"
#include <string.h>
#include "brickml_events.h"

#define USB_TASK_PRIORITY           (configMAX_PRIORITIES - 2)
#define USB_TASK_STACK_SIZE_BYTE    (4096u)

#define UART_RX_BUFFER_SIZE         2048
static uint8_t g_temp_buffer[UART_RX_BUFFER_SIZE] = {0};

/* Counter to update g_temp_buffer index */
static uint32_t g_rx_index = 0;
/* Index of data sent to at hanlder */
static uint32_t g_uart_rx_read_index = 0;

/* FreeRTOS module */
TaskHandle_t usb_thread;

static void usb_read(void);
static void usb_thread_entry(void *pvParameters);

void start_usb_thread(void)
{
    /* create a task to send data via usb */
    xTaskCreate(usb_thread_entry,
        (const char*) "USB PCDC Thread",
        USB_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        USB_TASK_PRIORITY, //uxPriority
        &usb_thread);
}

/* UsbThread entry function */
/* pvParameters contains TaskHandle_t */
static void usb_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED (pvParameters);
    fsp_err_t err = FSP_SUCCESS;

    /* Open the comms driver */
    err = comms_open((uint8_t)true);
    if (FSP_SUCCESS != err) {
        /* Stop as comms close failure */
        __BKPT(1);
    }

    while (1) {
        usb_read();
        vTaskDelay (1);
    }
}

/**
 * @brief Returns char from uart rx buffer
 *
 * @param is_inference_running If inference is running, we need to check for a single 'b'
 * @return
 */
char ei_get_serial_byte(uint8_t is_inference_running)
{
    FSP_PARAMETER_NOT_USED(is_inference_running);
    char to_send = -1;

    if (g_rx_index != 0) {
        to_send = g_temp_buffer[g_uart_rx_read_index++];    // get one
    }

    if ((g_uart_rx_read_index == g_rx_index) && (g_uart_rx_read_index != 0)) {  // when equal and different from zero
        g_uart_rx_read_index = 0;   // reset
        g_rx_index = 0;
        memset(g_temp_buffer, 0, sizeof(g_temp_buffer));
    }

    return to_send;
}

/**
 *
 * @return
 */
uint32_t ei_get_serial_buffer(uint8_t* pbuffer, uint32_t max_len)
{
    uint32_t sent = 0;

    if (g_rx_index > 0) {
        if (max_len < g_rx_index) {
            sent = max_len;
        }
        else {
            sent = g_rx_index;
        }
        memcpy(pbuffer, g_temp_buffer, sent);
        g_uart_rx_read_index = 0;   // reset
        g_rx_index = 0;
        memset(g_temp_buffer, 0, sizeof(g_temp_buffer));
    }

    return sent;
}

/**
 *
 */
void usb_clean_buffer(void)
{
    g_uart_rx_read_index = 0;   // reset
    g_rx_index = 0;
    memset(g_temp_buffer, 0, sizeof(g_temp_buffer));
}

/**
 * @brief Handles blockin read
 */
static void usb_read(void)
{
    uint32_t read = 0;
    if (comms_read(&g_temp_buffer[0], &read, portMAX_DELAY) == FSP_SUCCESS) {
        g_rx_index += read;
        xEventGroupSetBits(g_brickml_event_group, EVENT_RX_READY);
    }
}

