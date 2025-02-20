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
#include "ei_inertial.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "drivers/bno055/sf_bno055.h"
#include "drivers/bno055/sf_bno055_api.h"
#include "peripheral/i2c.h"
#include "hal_data.h"
#include "ei_main_thread.h"

/* Motion Sensor */
static sf_bno055_ctrl_t bno055_ctrl = {0};
static const sf_bno055_cfg_t bno055_cfg = { .device = &g_i2c_master0,
                                            .reset_pin = IOPORT_PORT_00_PIN_05, //
                                            .ioport = &g_ioport,
                                            .acc_cfg = { BNO055_ACC_RANGE_2G, BNO055_ACC_BANDWIDTH_250_HZ, BNO055_ACC_UNIT_M_S2 },
                                            .gyro_cfg = { BNO055_GYRO_RANGE_250dps, BNO055_GYRO_BANDWIDTH_230_HZ, BNO055_GYRO_UNIT_DPS },
                                            .mag_cfg = { BNO055_MAG_OUT_RATE_30_HZ, BNO055_MAG_OP_MODE_REGULAR },
                                            .temp_cfg = { BNO055_TEMP_SOURCE_ACC, BNO055_TEMP_UNIT_C } };
static sf_bno055_instance_t bno055 = { .p_ctrl = &bno055_ctrl, .p_cfg = &bno055_cfg, .p_api = &g_sf_bno055_api };

/* */
static float inertial_fusion_data[INERTIAL_AXIS_SAMPLED];
static float mag_fusion_data[MAG_AXIS_SAMPLED];

/* Public functions ------------------------------------------------ */
/**
 *
 * @return
 */
bool ei_inertial_init(void)
{
    fsp_err_t ret = FSP_SUCCESS;

    ret = g_i2c_master0.p_api->slaveAddressSet( g_i2c_master0.p_ctrl, 0x28, I2C_MASTER_ADDR_MODE_7BIT );
    ret = bno055.p_api->open( bno055.p_ctrl, bno055.p_cfg );

    if (FSP_SUCCESS == ret) {
        if(ei_add_sensor_to_fusion_list(inertial_sensor) == false) {
            ei_printf("ERR: failed to register Inertial sensor!\n");
            return false;
        }
        if(ei_add_sensor_to_fusion_list(mag_sensor) == false) {
            ei_printf("ERR: failed to register Magnetometer sensor!\n");
            return false;
        }
    }
    else {
        ei_printf("ERR: failed to initialize Inertial and Magnetometer sensor!\n");
    }

    return (FSP_SUCCESS == ret);
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* 
 */
float *ei_fusion_inertial_read_data(int n_samples)
{
    fsp_err_t ret = FSP_SUCCESS;

    memset(inertial_fusion_data, 0, sizeof(float) * 6);
    ret = g_i2c_master0.p_api->slaveAddressSet(g_i2c_master0.p_ctrl, 0x28, I2C_MASTER_ADDR_MODE_7BIT);
    if (n_samples >= 3)
    {
        ret  = bno055.p_api->readAccelerometer(bno055.p_ctrl, &inertial_fusion_data[0], &inertial_fusion_data[1], &inertial_fusion_data[2]);

        if (ret != FSP_SUCCESS) {
            memset(inertial_fusion_data, 0xff, sizeof(float) * 3);
            ei_printf("Error reading accelerometer\n");
        }
        else {
            for (uint8_t i = 0; i < 3; i++) {
                inertial_fusion_data[i] /= 100.0f;
            }
        }
    }

    if (n_samples > 3) {
        ret = bno055.p_api->readGyroscope(bno055.p_ctrl, &inertial_fusion_data[3], &inertial_fusion_data[4], &inertial_fusion_data[5]);

        if (ret != FSP_SUCCESS) {
            memset(&inertial_fusion_data[3], 0xff, sizeof(float) * 3);
            ei_printf("Error reading gyroscope\n");
        }
        else {
            for (uint8_t i = 3; i < 6; i++) {
                inertial_fusion_data[i] /= 100.0f;
            }
        }
    }

    return inertial_fusion_data;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* 
 */
float *ei_fusion_mag_read_data(int n_samples)
{
    fsp_err_t ret = FSP_SUCCESS;
    (void)n_samples;
    memset(mag_fusion_data, 0, sizeof(float)*3);

    ret = g_i2c_master0.p_api->slaveAddressSet(g_i2c_master0.p_ctrl, 0x28, I2C_MASTER_ADDR_MODE_7BIT );
    ret  = bno055.p_api->readMagnetometer(bno055.p_ctrl, &mag_fusion_data[0], &mag_fusion_data[1], &mag_fusion_data[2]);

    if (ret != FSP_SUCCESS) {
        ei_printf("Error reading magnetometer\n");
    }
    else {
        for (uint8_t i = 0; i < 3; i++) {
            mag_fusion_data[i] /= 100.0f;
        }
    }

    return mag_fusion_data;
}
