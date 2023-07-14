/* Edge Impulse ingestion SDK
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
#include "ei_adc.h"
#include "peripheral/adc.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

static float adc_fusion_data;

/**
 *
 * @return
 */
bool ei_adc_init(void)
{
    adc_init();

    if(ei_add_sensor_to_fusion_list(adc_sensor) == false) {
        ei_printf("ERR: failed to register ADC sensor!\n");
        return false;
    }

    return true;
}

/**
 *
 * @param n_samples
 * @return
 */
float *ei_fusion_adc_read_data(int n_samples)
{
    uint16_t reading = 0;
    (void)n_samples;

    adc_start_scan();

    adc_read(&reading);
    adc_fusion_data = ((float)(reading/4096.0f))*3.3f;

    return &adc_fusion_data;
}

/**
 * @brief test function
 */
void ei_adc_test(void)
{
    uint16_t reading = 0;
    uint32_t err = 0;

    while(1)
    {
        err = adc_start_scan();
        ei_printf("adc_start_scan: %d\n", err);

        err = adc_read(&reading);
        ei_printf("adc_read: %d\n", err);

        ei_printf("ADC: %d\n", reading);
        ei_sleep(500);
    }
}
