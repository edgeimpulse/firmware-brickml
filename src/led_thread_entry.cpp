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
