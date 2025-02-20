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
