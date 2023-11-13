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
    uint32_t retval = 0;
    FSP_PARAMETER_NOT_USED (pvParameters);
    
    p_dev =  static_cast<EiBrickml*>(EiDeviceInfo::get_device());

    led_off();
    button_init();
    qspi_init();
    flash_handler_init();
    ei_i2c_init();
    ei_i2s_driver_init();
    ei_timer_init();

    if ( xSemaphoreTake( g_usb_ready, portMAX_DELAY ) == pdFALSE ) {
        // error ! possible ?
        while(1) {

        }
    }

    do {
        vTaskDelay(100);  // improve!
    }while (comms_get_is_open() == false);

    Trace_build( &g_uart_trace );

    at = ei_at_init(p_dev);

    ei_printf("Hello from Edge Impulse\n");
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    ei_printf("Starting main loop\r\n");

    ei_inertial_init();
    ei_environmental_sensor_init();
    ei_adc_init();

    at->print_prompt();
    led_on();

    while(1)
    {
        /* handle command comming from uart */
        char data = ei_get_serial_byte((uint8_t)is_inference_running());

        while ((uint8_t)data != 0xFF) {
            //led_blue_on();
            if(is_inference_running() && data == 'b') {
            ei_stop_impulse();
            at->print_prompt();
            continue;
            }

            at->handle(data);
            data = ei_get_serial_byte((uint8_t)is_inference_running());
        }

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

