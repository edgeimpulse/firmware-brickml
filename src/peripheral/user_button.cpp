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

