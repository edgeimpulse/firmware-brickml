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
                                            .acc_cfg = { BNO055_ACC_RANGE_2G, BNO055_ACC_BANDWIDTH_1000_HZ, BNO055_ACC_UNIT_M_S2 },
                                            .gyro_cfg = { BNO055_GYRO_RANGE_250dps, BNO055_GYRO_BANDWIDTH_523_HZ, BNO055_GYRO_UNIT_DPS },
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

    ret = g_i2c_master0.p_api->slaveAddressSet(g_i2c_master0.p_ctrl, 0x28, I2C_MASTER_ADDR_MODE_7BIT);
    if (n_samples >= 3)
    {
        ret  = bno055.p_api->readAccelerometer(bno055.p_ctrl, &inertial_fusion_data[0], &inertial_fusion_data[1], &inertial_fusion_data[2]);

        if (ret != FSP_SUCCESS)
        {
            memset(inertial_fusion_data, '0', sizeof(float)*3);
            ei_printf("Error reading accelerometer\n");
        }
        else
        {
            for (uint8_t i = 0; i < 3; i++) {
                inertial_fusion_data[i] /= 100.0f;
            }
        }
    }

    if (n_samples > 3)
    {
        ret = bno055.p_api->readGyroscope(bno055.p_ctrl, &inertial_fusion_data[3], &inertial_fusion_data[4], &inertial_fusion_data[5]);

        if (ret != FSP_SUCCESS)
        {
            memset(&inertial_fusion_data[3], '0', sizeof(float)*3);
            ei_printf("Error reading gyroscope\n");
        }
        else
        {
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

    ret = g_i2c_master0.p_api->slaveAddressSet(g_i2c_master0.p_ctrl, 0x28, I2C_MASTER_ADDR_MODE_7BIT );
    ret  = bno055.p_api->readMagnetometer(bno055.p_ctrl, &mag_fusion_data[0], &mag_fusion_data[1], &mag_fusion_data[2]);

    if (ret != FSP_SUCCESS)
    {
        memset(mag_fusion_data, '0', sizeof(float)*3);
        ei_printf("Error reading magnetometer\n");
    }
    else
    {
        for (uint8_t i = 0; i < 3; i++) {
            mag_fusion_data[i] /= 100.0f;
        }
    }

    return mag_fusion_data;
}
