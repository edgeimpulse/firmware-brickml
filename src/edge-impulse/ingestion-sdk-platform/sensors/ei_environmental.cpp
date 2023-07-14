/* Edge Impulse ingestion SDK
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
