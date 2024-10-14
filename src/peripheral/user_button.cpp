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
/* Include ----------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "event_groups.h"

#include "user_button.h"
#include "common_data.h"
#include "brickml_events.h"

/**
 * @brief Init button
 */
void button_init(void)
{
#if (BRICKML_SOM == 0)
    g_external_irq8.p_api->open(g_external_irq8.p_ctrl, g_external_irq8.p_cfg);
    g_external_irq8.p_api->enable(g_external_irq8.p_ctrl);
#endif
}

/**
 *
 * @return
 */
uint8_t ui_button_status_get(void)
{
    uint8_t status = 0;

    g_ioport.p_api->pinRead(g_ioport.p_ctrl, BSP_IO_PORT_02_PIN_10, (bsp_io_level_t*)&status);

    return status;
}

/**
 *
 * @return
 */
uint8_t ui_button_toggle_get(void)
{
    static uint8_t button_old_status = 1;
    static uint8_t button_status = 0;
    uint8_t result = 0;

    button_status = ui_button_status_get();

    if ( (button_old_status == 0) && (button_status == 1) ) {
        result = 1;
    }

    button_old_status = button_status;

    return result;
}

/**
 * @brief callback function for the S1 push button
 * @param p_args
 */
void external_irq_button_callback(external_irq_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (p_args->channel == 8) {
#if (BRICKML_SOM == 0)
        xEventGroupSetBitsFromISR(g_brickml_event_group, BRICKML_EVENT_BUTTON, &xHigherPriorityTaskWoken);


#endif
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

