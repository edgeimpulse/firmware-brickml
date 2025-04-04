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

#ifndef EDGE_IMPULSE_INGESTION_SDK_PLATFORM_SENSORS_EI_ADC_H_
#define EDGE_IMPULSE_INGESTION_SDK_PLATFORM_SENSORS_EI_ADC_H_

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_fusion.h"

/** Number of axis used and sample data format */
#define ADC_AXIS_SAMPLED       1
#define SIZEOF_ADC_AXIS_SAMPLED   (sizeof(float) * INERTIAL_AXIS_SAMPLED)

/* Function prototypes ----------------------------------------------------- */
bool ei_adc_init(void);
void ei_adc_test(void);

float *ei_fusion_adc_read_data(int n_samples);

static const ei_device_fusion_sensor_t adc_sensor = {
    "ADC",
    // number of sensor module axis
    ADC_AXIS_SAMPLED,
    // sampling frequencies
    { 100.0f, 500.0f, 1000.0f },
    // axis name and units payload (must be same order as read in)
    { {"adc", "mV"}},
    // reference to read data function
    &ei_fusion_adc_read_data,
    0
};

#endif /* EDGE_IMPULSE_INGESTION_SDK_PLATFORM_SENSORS_EI_ADC_H_ */
