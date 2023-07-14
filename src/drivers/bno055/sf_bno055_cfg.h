/***********************************************************************************************************************
 * Copyright [2017] RELOC s.r.l. - All rights strictly reserved
 * This Software is provided for evaluation purposes; any other use — including reproduction, modification, use for
 * a commercial product, distribution, or republication — without the prior written permission of the Copyright holder
 * is strictly prohibited. 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
 * OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * File Name    : sf_bno055_cfg.h
 * Description  : Build time configurations for 9-Axis Absolute Orientation Sensor BNO055.
 **********************************************************************************************************************/

#ifndef SF_BNO055_CFG_H
#define SF_BNO055_CFG_H

/*******************************************************************************************************************//**
 * @ingroup SF_BNO055
 * @defgroup SF_BNO055_CFG Build Time Configurations
 * @{
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/** Specify whether to include code for API parameter checking. Valid settings include:
 *    - BSP_CFG_PARAM_CHECKING_ENABLE : Utilizes the system default setting from bsp_cfg.h
 *    - 1 : Includes parameter checking
 *    - 0 : Compiles out parameter checking
 */
#define SF_BNO055_CFG_PARAM_CHECKING_ENABLE (BSP_CFG_PARAM_CHECKING_ENABLE)

/** Specify RTOS tick rate, used for delays computation. */
#define THREADX_TICK_RATE               (TX_TIMER_TICKS_PER_SECOND)

/** Accelerometer Configurations */
#define BNO055_ACC_RANGE_2G             (0x00)
#define BNO055_ACC_RANGE_4G             (0x01)
#define BNO055_ACC_RANGE_8G             (0x02)
#define BNO055_ACC_RANGE_16G            (0x03)

#define BNO055_ACC_BANDWIDTH_7_81_HZ    (0x00)
#define BNO055_ACC_BANDWIDTH_15_63_HZ   (0x04)
#define BNO055_ACC_BANDWIDTH_31_25_HZ   (0x08)
#define BNO055_ACC_BANDWIDTH_62_5_HZ    (0x0C)
#define BNO055_ACC_BANDWIDTH_125_HZ     (0x10)
#define BNO055_ACC_BANDWIDTH_250_HZ     (0x14)
#define BNO055_ACC_BANDWIDTH_500_HZ     (0x18)
#define BNO055_ACC_BANDWIDTH_1000_HZ    (0x1C)

#define BNO055_ACC_UNIT_M_S2            (0x00)
#define BNO055_ACC_UNIT_MG              (0x01)

/** Gyroscope Configurations */
#define BNO055_GYRO_RANGE_2000dps       (0x00)
#define BNO055_GYRO_RANGE_1000dps       (0x01)
#define BNO055_GYRO_RANGE_500dps        (0x02)
#define BNO055_GYRO_RANGE_250dps        (0x03)
#define BNO055_GYRO_RANGE_125dps        (0x04)

#define BNO055_GYRO_BANDWIDTH_523_HZ    (0x00)
#define BNO055_GYRO_BANDWIDTH_230_HZ    (0x08)
#define BNO055_GYRO_BANDWIDTH_116_HZ    (0x10)
#define BNO055_GYRO_BANDWIDTH_47_HZ     (0x18)
#define BNO055_GYRO_BANDWIDTH_23_HZ     (0x20)
#define BNO055_GYRO_BANDWIDTH_12_HZ     (0x28)
#define BNO055_GYRO_BANDWIDTH_64_HZ     (0x30)
#define BNO055_GYRO_BANDWIDTH_32_HZ     (0x38)

#define BNO055_GYRO_UNIT_DPS            (0x00)
#define BNO055_GYRO_UNIT_RPS            (0x02)

/** Magnetometer Configurations */
#define BNO055_MAG_OUT_RATE_2_HZ        (0x00)
#define BNO055_MAG_OUT_RATE_6_HZ        (0x01)
#define BNO055_MAG_OUT_RATE_8_HZ        (0x02)
#define BNO055_MAG_OUT_RATE_10_HZ       (0x03)
#define BNO055_MAG_OUT_RATE_15_HZ       (0x04)
#define BNO055_MAG_OUT_RATE_20_HZ       (0x05)
#define BNO055_MAG_OUT_RATE_25_HZ       (0x06)
#define BNO055_MAG_OUT_RATE_30_HZ       (0x07)

#define BNO055_MAG_OP_MODE_LOW_P        (0x00)
#define BNO055_MAG_OP_MODE_REGULAR      (0x08)
#define BNO055_MAG_OP_MODE_ENH_REGULAR  (0x10)
#define BNO055_MAG_OP_MODE_HIGH_ACCUR   (0x18)

#define BNO055_MAG_UNIT_D               (0x00)
#define BNO055_MAG_UNIT_R               (0x04)

/** TERMOMETER Configurations */
#define BNO055_TEMP_SOURCE_ACC          (0x00)
#define BNO055_TEMP_SOURCE_GYRO         (0x01)

#define BNO055_TEMP_UNIT_C              (0x00)
#define BNO055_TEMP_UNIT_F              (0x10)

/** Fusion Data Format */
#define BNO055_PG0_UNIT_SEL_ORI_WIN     (0x00)
#define BNO055_PG0_UNIT_SEL_ORI_AND     (0x80)

/** Euler Data Unit */
#define BNO055_PG0_UNIT_SEL_EUL_DEG     (0x00)
#define BNO055_PG0_UNIT_SEL_EUL_RAD     (0x04)

/*******************************************************************************************************************//**
 * @} (end defgroup SF_BNO055_CFG)
 **********************************************************************************************************************/
#endif /* SF_BNO055_CFG_H */
