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
#include "led_thread.h"
#include "board/brickml/board_leds.h"
#include "ingestion-sdk-platform/brickml/ei_device_brickml.h"

#include <FreeRTOS.h>
#include <queue.h>

/* LedThread entry function */
/* pvParameters contains TaskHandle_t */
void led_thread_entry(void *pvParameters)
{
    EiBrickml       *dev = static_cast<EiBrickml*>(EiDeviceInfo::get_device());
    EiBrickMlState  old_status = eiBrickMLStateIdle;
    TickType_t      delay = 250;
    BaseType_t      fr_err;
    uint32_t        len = 0;
    bool            toogle_finished = false;

    FSP_PARAMETER_NOT_USED (pvParameters);

    while (1)
    {
        fr_err = xQueueReceive(g_new_state_queue, &len, delay); // check if new state received ?

        if (pdPASS == fr_err) {
            old_status = dev->get_state();
        }

        switch(old_status)
        {
            case eiBrickMLStateIdle:
            {
                led_green_on();
                led_blue_off();
                led_red_off();

                delay = 500;
            }
            break;
            case eiBrickMLStateErasingFlash:
            {
                led_green_toggle();
                led_blue_off();
                led_red_off();

                delay = 500;
            }
            break;
            case eiBrickMLStateSampling:
            {
                led_green_off();
                led_blue_toggle();
                led_red_off();

                delay = 500;
            }
            break;
            case eiBrickMLStateUploading:
            {
                led_green_off();
                led_blue_toggle();
                led_red_off();

                delay = 250;
            }
            break;
            case eiBrickMLStateFinished:
            {
                if (toogle_finished) {
                    led_green_off();
                    led_blue_off();
                    led_red_off();
                    toogle_finished = false;
                }
                else {
                    led_green_on();
                    led_blue_on();
                    led_red_on();
                    toogle_finished = true;
                }

                delay = 250;
            }
            break;
            case eiBrickMLStateCombi_1:
            {
                led_green_on();
                led_blue_on();
                led_red_off();

                delay = 250;
            }
            break;
            case eiBrickMLStateCombi_2:
            {
                led_green_on();
                led_blue_off();
                led_red_on();

                delay = 250;
            }
            break;
            case eiBrickMLStateCombi_3:
            {
                led_green_off();
                led_blue_on();
                led_red_on();

                delay = 250;
            }
            break;
            case eiBrickMLStateCombi_4:
            {
                led_green_on();
                led_blue_on();
                led_red_on();

                delay = 250;
            }
            break;
            default:
            {

            }
            break;
        }
    }
}
