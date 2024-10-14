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
#include <stdio.h>
#include <string.h>
#include <ei_main_thread.h>
#include "common/common_assert.h"
#include "trace_own.h"
#include "trace_use.h"
#include "version.h"
#include "comms/comms.h"
#include "brickml_events.h"
#include "ingestion_thread.h"
//#include "config.h"   // for BLE
#include "ingestion-sdk-platform/brickml/ei_at_handlers.h"
#include "ingestion-sdk-platform/brickml/ei_device_brickml.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "inference/ei_run_impulse.h"
#include "board_leds.h"
#include "usb_thread_interface.h"
#include "peripheral/flash_handler.h"
#include "peripheral/qspi.h"
#include "peripheral/i2c.h"
#include "peripheral/i2s.h"
#include "peripheral/timer_handler.h"
#include "peripheral/user_button.h"

#if (BRICKML_SOM == 0)
#include "peripheral/spi_drv.h"
#include "peripheral/sd/sd_handler.h"
#endif

// Sensors, probably will move
#include "sensors/ei_inertial.h"
#include "sensors/ei_environmental.h"
#include "sensors/ei_adc.h"
#include "sensors/ei_microphone.h"


/***********************************************************************************************************************
 * Macros
 **********************************************************************************************************************/
/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * Private variables
 **********************************************************************************************************************/
/* Private variables -------------------------------------------------------------------- */
static ATServer *at;
EiBrickml* p_dev;

/***********************************************************************************************************************
 * Public API Functions Definitions
 **********************************************************************************************************************/

/* Main Thread entry function */
/* pvParameters contains TaskHandle_t */
void ei_main_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED (pvParameters);
    EventBits_t events;
    bool in_rx_loop = false;
    
    p_dev =  static_cast<EiBrickml*>(EiDeviceInfo::get_device());

    led_off();
    button_init();
    qspi_init();
    flash_handler_init();
    ei_i2c_init();
    ei_i2s_driver_init();
    ei_timer_init();

    p_dev->init_sd();
    start_usb_thread();

    at = ei_at_init(p_dev);

    ei_printf("Hello from Edge Impulse\n");
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    ei_printf("Starting main loop\r\n");

    ei_inertial_init();
    ei_environmental_sensor_init();
    ei_adc_init();

    at->print_prompt();
    led_on();

    button_init();// ok enable the button

    while(1) {
        events = xEventGroupWaitBits(g_brickml_event_group, EVENT_RX_READY | BRICKML_EVENT_BUTTON, pdTRUE, pdFALSE , portMAX_DELAY);

        // rx console event
        if (events & EVENT_RX_READY) {
            char data = ei_get_serial_byte(is_inference_running());

            in_rx_loop = false;

            while ((uint8_t)data != 0xFF) {

                if ((is_inference_running() == true) && (data == 'b') && (in_rx_loop == false)) {
                    ei_stop_impulse();
                    at->print_prompt();
                    continue;
                }

                at->handle(data);
                data = ei_get_serial_byte((uint8_t)is_inference_running());
            }
        }
        else if (events & BRICKML_EVENT_BUTTON) {
            if (p_dev->is_sd_in_use() == true) {
                if (ingestion_thread_start() == false) {
                    ei_printf("Failed to run ingestion thread\r\n");
                }
            }
            else {
                ei_printf("SD not found or not selected as main memory\r\n");
            }
        }
        /* handle command comming from uart */
        char data = ei_get_serial_byte((uint8_t)is_inference_running());

        // how to ?
        if (is_inference_running() == true) {
            ei_run_impulse();
        }
    }

    while (1)
    {
        /* we should never end here */
        vTaskDelay (portMAX_DELAY);
    }
}
