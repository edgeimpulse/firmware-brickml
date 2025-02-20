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

#include "ei_environmental.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "drivers/hs300x/hs300x.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "ei_main_thread.h"
#include "peripheral/i2c.h"

Hs300x_t Hs300x_instance = {0};
static float env_data[ENVIRONMENTAL_SENSOR_NUMBER];

extern EventGroupHandle_t i2c_riic0_event_handle;

/**
 *
 * @return
 */
bool ei_environmental_sensor_init(void)
{
    fsp_err_t ret = FSP_SUCCESS;

    ei_i2c_set_slave_address(0x44);
    if (Hs300x_Open(&Hs300x_instance, &g_i2c_master0, i2c_riic0_event_handle) == FSP_SUCCESS) {
        if(ei_add_sensor_to_fusion_list(environment_sensor) == false) {
            ei_printf("ERR: failed to register Environmental sensor!\r\n");
            return false;
        }
    }
    else {
        ei_printf("ERR: failed to initialize Environmental sensor!\r\n");
        return false;
    }

    return true;
}

/**
 *
 * @param n_samples
 * @return
 */
float* ei_fusion_environment_sensor_read_data(int n_samples)
{
    fsp_err_t ret = FSP_SUCCESS;
    (void)n_samples;

    ei_i2c_set_slave_address(0x44);
    ret = Hs300x_GetMeasure( &Hs300x_instance, &env_data[0], &env_data[1] );

    if (ret != FSP_SUCCESS) {
        ei_printf("Error in env reading\n");
    }

    return env_data;
}
