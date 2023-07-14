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
 * File Name        : sf_bno055.c
 * Description      : BNO055 9-Axis Absolute Orientation Sensor Framework Layer API Driver.
 *********************************************************************************************************************/


/**********************************************************************************************************************
 * Includes
 *********************************************************************************************************************/
#include "bsp_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sf_bno055.h"
#include "sf_bno055_cfg.h"
#include "sf_bno055_private_api.h"
#include "r_i2c_master_api.h"
#include "r_ioport_api.h"

//#include "sensors_thread_entry.h"

/**********************************************************************************************************************
 * Macro definitions
 *********************************************************************************************************************/
/** Macro for error logger. */
#ifndef SF_BNO055_ERROR_RETURN
#define SF_BNO055_ERROR_RETURN(a, err) FSP_ERROR_RETURN((a), (err))
#endif

//#define SF_BNO055_ERROR_UNLOCK_AND_RETURN(a, err) \
//    {                                             \
//        if (!(a))                                 \
//        {                                         \
//            p_ctrl->device->p_api->unlock( p_ctrl->device->p_ctrl ); \
//            FSP_ERROR_LOG((err));  \
//            return (err);                         \
//        }                                         \
//    }
#define SF_BNO055_ERROR_UNLOCK_AND_RETURN(a, err) \
    {                                             \
        if (!(a))                                 \
        {                                         \
            FSP_ERROR_LOG((err));  \
            return (err);                         \
        }                                         \
    }

#define BNO055_INSTANCE_OPEN                    (0x76138495UL)
#define BNO055_INSTANCE_CLOSED                  (0x00000000UL)

#define MAX( a, b )                             ( ((a) < (b)) ? (b) : (a) )
#define CONV_MS_TO_TICK(a)                      ((a>0) ? MAX( ((a)*1000/1000), 1 ) : 0) // ((a>0) ? MAX( ((a)*TX_TIMER_TICKS_PER_SECOND/1000), 1 ) : 0)
/* This macro is used in order to convert a time from "ms" to "tick" unit. Moreover it makes sure to have   */
/* a '1-tick' result even if the requested time is less than a single tick, except for 0.                   */


#define BNO055_I2C_TIMEOUT_MS                   (1000)
#define BNO055_RESET_TIME_MS                    (850)
#define BNO055_I2C_TIMEOUT_TICK                 CONV_MS_TO_TICK( BNO055_I2C_TIMEOUT_MS )
#define BNO055_RESET_TIME_TICK                  CONV_MS_TO_TICK( BNO055_RESET_TIME_MS )

/* PAGE0 REGISTER DEFINITION START */
#define BNO055_PG0_CHIP_ID_ADDR                 (0x0000)
#define BNO055_PG0_ACCEL_REV_ID_ADDR            (0x0001)
#define BNO055_PG0_MAG_REV_ID_ADDR              (0x0002)
#define BNO055_PG0_GYRO_REV_ID_ADDR             (0x0003)
#define BNO055_PG0_SW_REV_ID_LSB_ADDR           (0x0004)
#define BNO055_PG0_SW_REV_ID_MSB_ADDR           (0x0005)
#define BNO055_PG0_BL_REV_ID_ADDR               (0x0006)
#define BNO055_PG0_PAGE_ID_ADDR                 (0x0007) /* Page id register definition*/
#define BNO055_PG0_ACCEL_DATA_X_LSB_ADDR        (0x0008) /* Accel data register */
#define BNO055_PG0_ACCEL_DATA_X_MSB_ADDR        (0x0009) /* Accel data register */
#define BNO055_PG0_ACCEL_DATA_Y_LSB_ADDR        (0x000A) /* Accel data register */
#define BNO055_PG0_ACCEL_DATA_Y_MSB_ADDR        (0x000B) /* Accel data register */
#define BNO055_PG0_ACCEL_DATA_Z_LSB_ADDR        (0x000C) /* Accel data register */
#define BNO055_PG0_ACCEL_DATA_Z_MSB_ADDR        (0x000D) /* Accel data register */
#define BNO055_PG0_MAG_DATA_X_LSB_ADDR          (0x000E) /* Mag data register */
#define BNO055_PG0_MAG_DATA_X_MSB_ADDR          (0x000F) /* Mag data register */
#define BNO055_PG0_MAG_DATA_Y_LSB_ADDR          (0x0010) /* Mag data register */
#define BNO055_PG0_MAG_DATA_Y_MSB_ADDR          (0x0011) /* Mag data register */
#define BNO055_PG0_MAG_DATA_Z_LSB_ADDR          (0x0012) /* Mag data register */
#define BNO055_PG0_MAG_DATA_Z_MSB_ADDR          (0x0013) /* Mag data register */
#define BNO055_PG0_GYRO_DATA_X_LSB_ADDR         (0x0014) /* Gyro data registers */
#define BNO055_PG0_GYRO_DATA_X_MSB_ADDR         (0x0015) /* Gyro data registers */
#define BNO055_PG0_GYRO_DATA_Y_LSB_ADDR         (0x0016) /* Gyro data registers */
#define BNO055_PG0_GYRO_DATA_Y_MSB_ADDR         (0x0017) /* Gyro data registers */
#define BNO055_PG0_GYRO_DATA_Z_LSB_ADDR         (0x0018) /* Gyro data registers */
#define BNO055_PG0_GYRO_DATA_Z_MSB_ADDR         (0x0019) /* Gyro data registers */
#define BNO055_PG0_EULER_H_LSB_ADDR             (0x001A) /* Euler data registers */
#define BNO055_PG0_EULER_H_MSB_ADDR             (0x001B) /* Euler data registers */
#define BNO055_PG0_EULER_R_LSB_ADDR             (0x001C) /* Euler data registers */
#define BNO055_PG0_EULER_R_MSB_ADDR             (0x001D) /* Euler data registers */
#define BNO055_PG0_EULER_P_LSB_ADDR             (0x001E) /* Euler data registers */
#define BNO055_PG0_EULER_P_MSB_ADDR             (0x001F) /* Euler data registers */
#define BNO055_PG0_QUATERNION_DATA_W_LSB_ADDR   (0x0020) /* Quaternion data registers */
#define BNO055_PG0_QUATERNION_DATA_W_MSB_ADDR   (0x0021) /* Quaternion data registers */
#define BNO055_PG0_QUATERNION_DATA_X_LSB_ADDR   (0x0022) /* Quaternion data registers */
#define BNO055_PG0_QUATERNION_DATA_X_MSB_ADDR   (0x0023) /* Quaternion data registers */
#define BNO055_PG0_QUATERNION_DATA_Y_LSB_ADDR   (0x0024) /* Quaternion data registers */
#define BNO055_PG0_QUATERNION_DATA_Y_MSB_ADDR   (0x0025) /* Quaternion data registers */
#define BNO055_PG0_QUATERNION_DATA_Z_LSB_ADDR   (0x0026) /* Quaternion data registers */
#define BNO055_PG0_QUATERNION_DATA_Z_MSB_ADDR   (0x0027) /* Quaternion data registers */
#define BNO055_PG0_LINEAR_ACCEL_DATA_X_LSB_ADDR (0x0028) /* Linear acceleration data registers */
#define BNO055_PG0_LINEAR_ACCEL_DATA_X_MSB_ADDR (0x0029) /* Linear acceleration data registers */
#define BNO055_PG0_LINEAR_ACCEL_DATA_Y_LSB_ADDR (0x002A) /* Linear acceleration data registers */
#define BNO055_PG0_LINEAR_ACCEL_DATA_Y_MSB_ADDR (0x002B) /* Linear acceleration data registers */
#define BNO055_PG0_LINEAR_ACCEL_DATA_Z_LSB_ADDR (0x002C) /* Linear acceleration data registers */
#define BNO055_PG0_LINEAR_ACCEL_DATA_Z_MSB_ADDR (0x002D) /* Linear acceleration data registers */
#define BNO055_PG0_GRAVITY_DATA_X_LSB_ADDR      (0x002E) /* Gravity data registers */
#define BNO055_PG0_GRAVITY_DATA_X_MSB_ADDR      (0x002F) /* Gravity data registers */
#define BNO055_PG0_GRAVITY_DATA_Y_LSB_ADDR      (0x0030) /* Gravity data registers */
#define BNO055_PG0_GRAVITY_DATA_Y_MSB_ADDR      (0x0031) /* Gravity data registers */
#define BNO055_PG0_GRAVITY_DATA_Z_LSB_ADDR      (0x0032) /* Gravity data registers */
#define BNO055_PG0_GRAVITY_DATA_Z_MSB_ADDR      (0x0033) /* Gravity data registers */
#define BNO055_PG0_TEMP_ADDR                    (0x0034) /* Temperature data register */
#define BNO055_PG0_CALIB_STAT_ADDR              (0x0035)
#define BNO055_PG0_SELFTEST_RESULT_ADDR         (0x0036)
#define BNO055_PG0_INTR_STAT_ADDR               (0x0037)
#define BNO055_PG0_SYS_CLK_STAT_ADDR            (0x0038)
#define BNO055_PG0_SYS_STAT_ADDR                (0x0039)
#define BNO055_PG0_SYS_ERR_ADDR                 (0x003A)
#define BNO055_PG0_UNIT_SEL_ADDR                (0x003B) /* Unit selection register */
#define BNO055_PG0_DATA_SELECT_ADDR             (0x003C) /* Unit selection register */
#define BNO055_PG0_OPR_MODE_ADDR                (0x003D) /* Operating Mode register */
#define BNO055_PG0_PWR_MODE_ADDR                (0x003E) /* Power Mode register */
#define BNO055_PG0_SYS_TRIGGER_ADDR             (0x003F)
#define BNO055_PG0_TEMP_SOURCE_ADDR             (0x0040)
#define BNO055_PG0_AXIS_MAP_CONFIG_ADDR         (0x0041) /* Axis remap registers */
#define BNO055_PG0_AXIS_MAP_SIGN_ADDR           (0x0042) /* Axis remap registers */
#define BNO055_PG0_ACCEL_OFFSET_X_LSB_ADDR      (0x0055) /* Accelerometer Offset registers */
#define BNO055_PG0_ACCEL_OFFSET_X_MSB_ADDR      (0x0056) /* Accelerometer Offset registers */
#define BNO055_PG0_ACCEL_OFFSET_Y_LSB_ADDR      (0x0057) /* Accelerometer Offset registers */
#define BNO055_PG0_ACCEL_OFFSET_Y_MSB_ADDR      (0x0058) /* Accelerometer Offset registers */
#define BNO055_PG0_ACCEL_OFFSET_Z_LSB_ADDR      (0x0059) /* Accelerometer Offset registers */
#define BNO055_PG0_ACCEL_OFFSET_Z_MSB_ADDR      (0x005A) /* Accelerometer Offset registers */
#define BNO055_PG0_MAG_OFFSET_X_LSB_ADDR        (0x005B) /* Magnetometer Offset registers */
#define BNO055_PG0_MAG_OFFSET_X_MSB_ADDR        (0x005C) /* Magnetometer Offset registers */
#define BNO055_PG0_MAG_OFFSET_Y_LSB_ADDR        (0x005D) /* Magnetometer Offset registers */
#define BNO055_PG0_MAG_OFFSET_Y_MSB_ADDR        (0x005E) /* Magnetometer Offset registers */
#define BNO055_PG0_MAG_OFFSET_Z_LSB_ADDR        (0x005F) /* Magnetometer Offset registers */
#define BNO055_PG0_MAG_OFFSET_Z_MSB_ADDR        (0x0060) /* Magnetometer Offset registers */
#define BNO055_PG0_GYRO_OFFSET_X_LSB_ADDR       (0x0061) /* Gyroscope Offset registers */
#define BNO055_PG0_GYRO_OFFSET_X_MSB_ADDR       (0x0062) /* Gyroscope Offset registers */
#define BNO055_PG0_GYRO_OFFSET_Y_LSB_ADDR       (0x0063) /* Gyroscope Offset registers */
#define BNO055_PG0_GYRO_OFFSET_Y_MSB_ADDR       (0x0064) /* Gyroscope Offset registers */
#define BNO055_PG0_GYRO_OFFSET_Z_LSB_ADDR       (0x0065) /* Gyroscope Offset registers */
#define BNO055_PG0_GYRO_OFFSET_Z_MSB_ADDR       (0x0066) /* Gyroscope Offset registers */
#define BNO055_PG0_ACCEL_RADIUS_LSB_ADDR        (0x0067) /* Radius registers */
#define BNO055_PG0_ACCEL_RADIUS_MSB_ADDR        (0x0068) /* Radius registers */
#define BNO055_PG0_MAG_RADIUS_LSB_ADDR          (0x0069) /* Radius registers */
#define BNO055_PG0_MAG_RADIUS_MSB_ADDR          (0x006A) /* Radius registers */


/* PAGE1 REGISTERS DEFINITION START */
/* 0x1000 has been added to the addresses in order to identify Page 1 registers */
#define BNO055_PG1_OFFSET_ADDR                  (0x1000) /* Not a real register! */

#define BNO055_PG1_PAGE_ID_ADDR                 (0x1007) /* Page id register definition*/
#define BNO055_PG1_ACCEL_CONFIG_ADDR            (0x1008)
#define BNO055_PG1_MAG_CONFIG_ADDR              (0x1009)
#define BNO055_PG1_GYRO_CONFIG_ADDR             (0x100A)
#define BNO055_PG1_GYRO_MODE_CONFIG_ADDR        (0x100B)
#define BNO055_PG1_ACCEL_SLEEP_CONFIG_ADDR      (0x100C)
#define BNO055_PG1_GYRO_SLEEP_CONFIG_ADDR       (0x100D)
#define BNO055_PG1_MAG_SLEEP_CONFIG_ADDR        (0x100E)
#define BNO055_PG1_INT_MASK_ADDR                (0x100F)
#define BNO055_PG1_INT_ADDR                     (0x1010)
#define BNO055_PG1_ACCEL_ANY_MOTION_THRES_ADDR  (0x1011)
#define BNO055_PG1_ACCEL_INTR_SETTINGS_ADDR     (0x1012)
#define BNO055_PG1_ACCEL_HIGH_G_DURN_ADDR       (0x1013)
#define BNO055_PG1_ACCEL_HIGH_G_THRES_ADDR      (0x1014)
#define BNO055_PG1_ACCEL_NO_MOTION_THRES_ADDR   (0x1015)
#define BNO055_PG1_ACCEL_NO_MOTION_SET_ADDR     (0x1016)
#define BNO055_PG1_GYRO_INTR_SETING_ADDR        (0x1017)
#define BNO055_PG1_GYRO_HIGHRATE_X_SET_ADDR     (0x1018)
#define BNO055_PG1_GYRO_DURN_X_ADDR             (0x1019)
#define BNO055_PG1_GYRO_HIGHRATE_Y_SET_ADDR     (0x101A)
#define BNO055_PG1_GYRO_DURN_Y_ADDR             (0x101B)
#define BNO055_PG1_GYRO_HIGHRATE_Z_SET_ADDR     (0x101C)
#define BNO055_PG1_GYRO_DURN_Z_ADDR             (0x101D)
#define BNO055_PG1_GYRO_ANY_MOTION_THRES_ADDR   (0x101E)
#define BNO055_PG1_GYRO_ANY_MOTION_SET_ADDR     (0x101F)


/* REGISTER VALUES */
#define BNO055_PG0_CHIP_ID_VALUE                (0xA0) /* Factory value */
#define BNO055_PG0_ACCEL_REV_ID_VALUE           (0xFB) /* Factory value */
#define BNO055_PG0_MAG_REV_ID_VALUE             (0x32) /* Factory value */
#define BNO055_PG0_GYRO_REV_ID_VALUE            (0x0F) /* Factory value */

#define BNO055_PG0_PAGE_ID_0                    (0x00)
#define BNO055_PG0_PAGE_ID_1                    (0x01)

#define BNO055_PG0_SYS_TRIGGER_CLK_EXT          (0x80) /* Switch to external oscillator */
#define BNO055_PG0_SYS_TRIGGER_RST_INT          (0x40) /* Reset Interrupts */
#define BNO055_PG0_SYS_TRIGGER_RST_SYS          (0x20) /* Reset System */
#define BNO055_PG0_SYS_TRIGGER_SELF_TEST        (0x01) /* Self Test */

#define BNO055_PG0_PWR_MODE_NORMAL              (0x00)
#define BNO055_PG0_PWR_MODE_LOW_POWER           (0x01)
#define BNO055_PG0_PWR_MODE_SUSPEND             (0x02)

#define BNO055_PG0_OPR_MODE_CONFIG              (0x00)
#define BNO055_PG0_OPR_MODE_ACC_ONLY            (0x01)
#define BNO055_PG0_OPR_MODE_MAG_ONLY            (0x02)
#define BNO055_PG0_OPR_MODE_GYRO_ONLY           (0x03)
#define BNO055_PG0_OPR_MODE_ACC_MAG             (0x04)
#define BNO055_PG0_OPR_MODE_ACC_GYRO            (0x05)
#define BNO055_PG0_OPR_MODE_MAG_GYRO            (0x06)
#define BNO055_PG0_OPR_MODE_AMG                 (0x07)
#define BNO055_PG0_OPR_MODE_IMU                 (0x08)
#define BNO055_PG0_OPR_MODE_COMPASS             (0x09)
#define BNO055_PG0_OPR_MODE_M4G                 (0x0A)
#define BNO055_PG0_OPR_MODE_NDOF_FMS_OFF        (0x0B)
#define BNO055_PG0_OPR_MODE_NDOF                (0x0C)

#define BNO055_PG0_TEMP_SOURCE_MASK             (0x03)

#define BNO055_PG0_UNIT_SEL_ORI_MASK            (0x80)
#define BNO055_PG0_UNIT_SEL_TEMP_MASK           (0x10)
#define BNO055_PG0_UNIT_SEL_EUL_MASK            (0x04)
#define BNO055_PG0_UNIT_SEL_GYRO_MASK           (0x02)
#define BNO055_PG0_UNIT_SEL_ACC_MASK            (0x01)

#define BNO055_PG1_ACCEL_CONFIG_PWR_MODE_MASK   (0xE0)
#define BNO055_PG1_ACCEL_CONFIG_BW_MASK         (0x18)
#define BNO055_PG1_ACCEL_CONFIG_RANGE_MASK      (0x07)

#define BNO055_PG1_GYRO_CONFIG_BW_MASK          (0x38)
#define BNO055_PG1_GYRO_CONFIG_RANGE_MASK       (0x07)

#define BNO055_PG1_GYRO_MODE_CONFIG_PWR_NORMAL  (0x00)
#define BNO055_PG1_GYRO_MODE_CONFIG_PWR_FAST_UP (0x01)
#define BNO055_PG1_GYRO_MODE_CONFIG_PWR_DEEPS   (0x02)
#define BNO055_PG1_GYRO_MODE_CONFIG_PWR_SUSP    (0x03)
#define BNO055_PG1_GYRO_MODE_CONFIG_PWR_ADV_SAV (0x04)
#define BNO055_PG1_GYRO_MODE_CONFIG_PWR_MASK    (0x07)

#define BNO055_PG1_MAG_CONFIG_PWR_NORMAL        (0x00)
#define BNO055_PG1_MAG_CONFIG_PWR_SLEEP         (0x20)
#define BNO055_PG1_MAG_CONFIG_PWR_SUSP          (0x40)
#define BNO055_PG1_MAG_CONFIG_PWR_FORCE         (0x60)
#define BNO055_PG1_MAG_CONFIG_PWR_MASK          (0x60)
#define BNO055_PG1_MAG_CONFIG_OPM_MASK          (0x18)
#define BNO055_PG1_MAG_CONFIG_OUT_RATE_MASK     (0x07)

/* BUFFER LENGTHS */
#define TMP_BUF_1BYTE_LEN                       (1)
#define TMP_BUF_2BYTES_LEN                      (2)
#define TMP_BUF_3BYTES_LEN                      (3)
#define TMP_BUF_4BYTES_LEN                      (4)
#define TMP_BUF_5BYTES_LEN                      (5)
#define TMP_BUF_6BYTES_LEN                      (6)
#define TMP_BUF_8BYTES_LEN                      (8)


#define BNO055_EULER_DATA_COEFF                 ((float) 1/((float)16))
#define BNO055_QUATERNION_DATA_COEFF            ((float) 1/((float)16384))


/**********************************************************************************************************************
 * Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Private function prototypes
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Private global variables
 *********************************************************************************************************************/
#if 0
static const ssp_version_t g_version =
{
    .api_version_major  = SF_BNO055_API_VERSION_MAJOR,
    .api_version_minor  = SF_BNO055_API_VERSION_MINOR,
    .code_version_major = SF_BNO055_CODE_VERSION_MAJOR,
    .code_version_minor = SF_BNO055_CODE_VERSION_MINOR
};
#endif

/** Name of module used by error logger macro */
static const char g_module_name[] = "sf_bno055";

/**********************************************************************************************************************
 * Private global variables
 *********************************************************************************************************************/
const sf_bno055_api_t g_sf_bno055_api =
{
    .open               = SF_BNO055_Open,
    .readAccelerometer  = SF_BNO055_ReadAccelerometer,
    .readGyroscope      = SF_BNO055_ReadGyroscope,
    .readMagnetometer   = SF_BNO055_ReadMagnetometer,
    .readEulerAngles    = SF_BNO055_ReadEulerAngles,
    .readQuaternions    = SF_BNO055_ReadQuaternions,
    .readTemperature    = SF_BNO055_ReadTemperature,
    .reset              = SF_BNO055_Reset,
    .close              = SF_BNO055_Close,
    .versionGet         = SF_BNO055_VersionGet,
};

/**********************************************************************************************************************
 * Functions
 *********************************************************************************************************************/
static fsp_err_t SF_BNO055_readRegister (   sf_bno055_ctrl_t * const p_ctrl,
                                            const uint16_t reg_addr,
                                            uint8_t* buffer,
                                            const uint8_t buffer_len );

static fsp_err_t SF_BNO055_writeRegister (  sf_bno055_ctrl_t * const p_ctrl,
                                            const uint16_t reg_addr,
                                            const uint8_t* buffer,
                                            const uint8_t buffer_len );

static fsp_err_t SF_BNO055_selectPage ( sf_bno055_ctrl_t * const p_ctrl,
                                        BNO055_Page_t requested_page );

static fsp_err_t SF_BNO055_initDevice (sf_bno055_ctrl_t * const p_ctrl);

static float SF_BNO055_convertData( uint8_t lsb, uint8_t msb );

/******************************************************************************************************************//**
 * @addtogroup SF_BNO055
 * @{
 *********************************************************************************************************************/
 
/******************************************************************************************************************//**
 * @brief Implements sf_bno055_api_t::open.
 *
 * @retval FSP_SUCCESS          Lower level drivers opened successfully.
 * @retval SSP_ERR_ASSERTION    A pointer parameter was NULL, or a lower level driver reported this error.
 * @retval SSP_ERR_ABORTED      Test read of device identification register failed.
 * @return                      See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
 *********************************************************************************************************************/
fsp_err_t SF_BNO055_Open (  sf_bno055_ctrl_t    * const p_ctrl,
                            sf_bno055_cfg_t const * const p_cfg)
{
    fsp_err_t err;
    uint8_t tmp_buf[TMP_BUF_4BYTES_LEN];
    
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != p_ctrl);
    SSP_ASSERT(NULL != p_cfg);
    SSP_ASSERT(NULL != p_cfg->device);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    /** Store user configurations in control block. */
    p_ctrl->device = p_cfg->device;
    p_ctrl->reset_pin = p_cfg->reset_pin;
    p_ctrl->ioport = p_cfg->ioport;
    p_ctrl->current_page = BNO055_PageInvalid;
    p_ctrl->reg_ACCEL_CONFIG =  (BNO055_PG1_ACCEL_CONFIG_BW_MASK & p_cfg->acc_cfg.bandwidth) |
                                (BNO055_PG1_ACCEL_CONFIG_RANGE_MASK & p_cfg->acc_cfg.range);
    
    p_ctrl->reg_GYRO_CONFIG =   (BNO055_PG1_GYRO_CONFIG_BW_MASK & p_cfg->gyro_cfg.bandwidth) |
                                (BNO055_PG1_GYRO_CONFIG_RANGE_MASK & p_cfg->gyro_cfg.range);
    
    p_ctrl->reg_GYRO_MODE_CONFIG = (BNO055_PG1_GYRO_MODE_CONFIG_PWR_MASK & BNO055_PG1_GYRO_MODE_CONFIG_PWR_NORMAL);
    
    p_ctrl->reg_MAG_CONFIG =    (BNO055_PG1_MAG_CONFIG_OPM_MASK & p_cfg->mag_cfg.mode) |
                                (BNO055_PG1_MAG_CONFIG_OUT_RATE_MASK & p_cfg->mag_cfg.rate);
    
    p_ctrl->reg_TEMP_SOURCE = (BNO055_PG0_TEMP_SOURCE_MASK & p_cfg->temp_cfg.source);
    
    p_ctrl->reg_UNIT_SEL =  (BNO055_PG0_UNIT_SEL_ORI_MASK & p_cfg->data_format) |
                            (BNO055_PG0_UNIT_SEL_TEMP_MASK & p_cfg->temp_cfg.unit) |
                            (BNO055_PG0_UNIT_SEL_EUL_MASK & p_cfg->euler_unit) |
                            (BNO055_PG0_UNIT_SEL_GYRO_MASK & p_cfg->gyro_cfg.unit) |
                            (BNO055_PG0_UNIT_SEL_ACC_MASK & p_cfg->acc_cfg.unit);
#if 1
    /** Reset Cycle */
    p_ctrl->ioport->p_api->pinWrite( p_ctrl->ioport->p_ctrl, p_ctrl->reset_pin, BSP_IO_LEVEL_LOW );
    //tx_thread_sleep(10);
#if (BSP_CFG_RTOS == 0)
    R_BSP_SoftwareDelay((uint32_t) 10, BSP_DELAY_UNITS_MILLISECONDS);
#elif (BSP_CFG_RTOS == 1)
    tx_thread_sleep(10);
#elif (BSP_CFG_RTOS == 2)
    vTaskDelay((TickType_t) (10 / portTICK_PERIOD_MS));
#endif
    p_ctrl->ioport->p_api->pinWrite( p_ctrl->ioport->p_ctrl, p_ctrl->reset_pin, BSP_IO_LEVEL_HIGH );
    //tx_thread_sleep(BNO055_RESET_TIME_TICK);
#if (BSP_CFG_RTOS == 0)
    R_BSP_SoftwareDelay((uint32_t) BNO055_RESET_TIME_TICK, BSP_DELAY_UNITS_MILLISECONDS);
#elif (BSP_CFG_RTOS == 1)
    tx_thread_sleep(BNO055_RESET_TIME_TICK);
#elif (BSP_CFG_RTOS == 2)
    vTaskDelay((TickType_t) (BNO055_RESET_TIME_TICK / portTICK_PERIOD_MS));
#endif
    
    /** Open I2C Framework Device. */
    //err = p_ctrl->device->p_api->open(  p_ctrl->device->p_ctrl,
    //                                    p_ctrl->device->p_cfg );
    //SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /** Reset I2C bus */
    //err = p_ctrl->device->p_api->reset( p_ctrl->device->p_ctrl, TX_WAIT_FOREVER );
    //SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    //err = p_ctrl->device->p_api->lock( p_ctrl->device->p_ctrl );
    //SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
#if 1
    /** Read the ID codes and check them */
    err = SF_BNO055_readRegister (  p_ctrl,
                                    BNO055_PG0_CHIP_ID_ADDR,
                                    tmp_buf,
                                    TMP_BUF_4BYTES_LEN );
    SF_BNO055_ERROR_UNLOCK_AND_RETURN(FSP_SUCCESS == err, err);
    SF_BNO055_ERROR_UNLOCK_AND_RETURN(BNO055_PG0_CHIP_ID_VALUE      == tmp_buf[0], FSP_ERR_ABORTED);
    SF_BNO055_ERROR_UNLOCK_AND_RETURN(BNO055_PG0_ACCEL_REV_ID_VALUE == tmp_buf[1], FSP_ERR_ABORTED);
    SF_BNO055_ERROR_UNLOCK_AND_RETURN(BNO055_PG0_MAG_REV_ID_VALUE   == tmp_buf[2], FSP_ERR_ABORTED);
    SF_BNO055_ERROR_UNLOCK_AND_RETURN(BNO055_PG0_GYRO_REV_ID_VALUE  == tmp_buf[3], FSP_ERR_ABORTED);
#endif
#endif
    /* Initialize the device */
    err = SF_BNO055_initDevice( p_ctrl );
    
    //p_ctrl->device->p_api->unlock( p_ctrl->device->p_ctrl );
    
    if( err == FSP_SUCCESS )
    {
        /** Mark control block open so other tasks know it is valid. */
        p_ctrl->open = BNO055_INSTANCE_OPEN;
    }
    
    return err;
}

/******************************************************************************************************************//**
 * @brief Implements sf_bno055_api_t::readAccelerometer.
 *
 * @retval FSP_SUCCESS          XYZ-axes accelerations read successfully.
 * @retval SSP_ERR_ASSERTION    A pointer parameter was NULL, or a lower level driver reported this error.
 * @retval SSP_ERR_NOT_OPEN     Driver is not configured. Call SF_BNO055_Open.
 * @return                      See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
 *********************************************************************************************************************/
fsp_err_t SF_BNO055_ReadAccelerometer ( sf_bno055_ctrl_t * const p_ctrl,
                                        float * x_value,
                                        float * y_value,
                                        float * z_value)
{
    fsp_err_t err;
    uint8_t tmp_buf[TMP_BUF_6BYTES_LEN];
    
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != p_ctrl);
    SSP_ASSERT(NULL != x_value);
    SSP_ASSERT(NULL != y_value);
    SSP_ASSERT(NULL != z_value);
    SF_BNO055_ERROR_RETURN(BNO055_INSTANCE_OPEN == p_ctrl->open, SSP_ERR_NOT_OPEN);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    //err = p_ctrl->device->p_api->lock( p_ctrl->device->p_ctrl );
    //SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /* Read the 6 registers of the X-Y-Z Accelerometer data in a single time */
    err = SF_BNO055_readRegister (  p_ctrl,
                                    BNO055_PG0_ACCEL_DATA_X_LSB_ADDR,
                                    tmp_buf,
                                    TMP_BUF_6BYTES_LEN );
    
    //p_ctrl->device->p_api->unlock( p_ctrl->device->p_ctrl );
    
    if( err == FSP_SUCCESS )
    {
        *x_value = SF_BNO055_convertData( tmp_buf[0], tmp_buf[1] );
        *y_value = SF_BNO055_convertData( tmp_buf[2], tmp_buf[3] );
        *z_value = SF_BNO055_convertData( tmp_buf[4], tmp_buf[5] );
    }
    
    return err;
}

/******************************************************************************************************************//**
 * @brief Implements sf_bno055_api_t::readGyroscope.
 *
 * @retval FSP_SUCCESS          XYZ-axes angular rates read successfully.
 * @retval SSP_ERR_ASSERTION    A pointer parameter was NULL, or a lower level driver reported this error.
 * @retval SSP_ERR_NOT_OPEN     Driver is not configured. Call SF_BNO055_Open.
 * @return                      See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
 *********************************************************************************************************************/
fsp_err_t SF_BNO055_ReadGyroscope ( sf_bno055_ctrl_t * const p_ctrl,
                                    float * x_value,
                                    float * y_value,
                                    float * z_value)
{
    fsp_err_t err;
    uint8_t tmp_buf[TMP_BUF_6BYTES_LEN];
    
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != p_ctrl);
    SSP_ASSERT(NULL != x_value);
    SSP_ASSERT(NULL != y_value);
    SSP_ASSERT(NULL != z_value);
    SF_BNO055_ERROR_RETURN(BNO055_INSTANCE_OPEN == p_ctrl->open, SSP_ERR_NOT_OPEN);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    //err = p_ctrl->device->p_api->lock( p_ctrl->device->p_ctrl );
    //SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /* Read the 6 registers of the X-Y-Z Gyroscope data in a single time */
    err = SF_BNO055_readRegister (  p_ctrl,
                                    BNO055_PG0_GYRO_DATA_X_LSB_ADDR,
                                    tmp_buf,
                                    TMP_BUF_6BYTES_LEN );
    
    //p_ctrl->device->p_api->unlock( p_ctrl->device->p_ctrl );
    
    if( err == FSP_SUCCESS )
    {
        *x_value = SF_BNO055_convertData( tmp_buf[0], tmp_buf[1] );
        *y_value = SF_BNO055_convertData( tmp_buf[2], tmp_buf[3] );
        *z_value = SF_BNO055_convertData( tmp_buf[4], tmp_buf[5] );
    }
    
    return err;
}

/******************************************************************************************************************//**
 * @brief Implements sf_bno055_api_t::readMagnetometer.
 *
 * @retval FSP_SUCCESS          XYZ-axes magnetic field strength read successfully.
 * @retval SSP_ERR_ASSERTION    A pointer parameter was NULL, or a lower level driver reported this error.
 * @retval SSP_ERR_NOT_OPEN     Driver is not configured. Call SF_BNO055_Open.
 * @return                      See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
 *********************************************************************************************************************/
fsp_err_t SF_BNO055_ReadMagnetometer (  sf_bno055_ctrl_t * const p_ctrl,
                                        float * x_value,
                                        float * y_value,
                                        float * z_value)
{
    fsp_err_t err;
    uint8_t tmp_buf[TMP_BUF_6BYTES_LEN];
    
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    FSP_ASSERT(NULL != p_ctrl);
    FSP_ASSERT(NULL != x_value);
    FSP_ASSERT(NULL != y_value);
    FSP_ASSERT(NULL != z_value);
    SF_BNO055_ERROR_RETURN(BNO055_INSTANCE_OPEN == p_ctrl->open, FSP_ERR_NOT_OPEN);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    //err = p_ctrl->device->p_api->lock( p_ctrl->device->p_ctrl );
    //SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /* Read the 6 registers of the X-Y-Z Magnetometer data in a single time */
    err = SF_BNO055_readRegister (  p_ctrl,
                                    BNO055_PG0_MAG_DATA_X_LSB_ADDR,
                                    tmp_buf,
                                    TMP_BUF_6BYTES_LEN );
    
    //p_ctrl->device->p_api->unlock( p_ctrl->device->p_ctrl );
    
    if( err == FSP_SUCCESS )
    {
        *x_value = SF_BNO055_convertData( tmp_buf[0], tmp_buf[1] );
        *y_value = SF_BNO055_convertData( tmp_buf[2], tmp_buf[3] );
        *z_value = SF_BNO055_convertData( tmp_buf[4], tmp_buf[5] );
    }
    
    return err;
}

/******************************************************************************************************************//**
 * @brief Implements sf_bno055_api_t::readEulerAngles.
 *
 * @retval FSP_SUCCESS          Pitch, Roll and Heading read successfully.
 * @retval SSP_ERR_ASSERTION    A pointer parameter was NULL, or a lower level driver reported this error.
 * @retval SSP_ERR_NOT_OPEN     Driver is not configured. Call SF_BNO055_Open.
 * @return                      See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
 *********************************************************************************************************************/
fsp_err_t SF_BNO055_ReadEulerAngles (   sf_bno055_ctrl_t * const p_ctrl,
                                        float * pitch_value,
                                        float * roll_value,
                                        float * heading_value)
{
    fsp_err_t err;
    uint8_t tmp_buf[TMP_BUF_6BYTES_LEN];
    
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != p_ctrl);
    SSP_ASSERT(NULL != pitch_value);
    SSP_ASSERT(NULL != roll_value);
    SSP_ASSERT(NULL != heading_value);
    SF_BNO055_ERROR_RETURN(BNO055_INSTANCE_OPEN == p_ctrl->open, SSP_ERR_NOT_OPEN);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    //err = p_ctrl->device->p_api->lock( p_ctrl->device->p_ctrl );
    //SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /* Read the 6 registers of the Pitch, Roll and Heading data in a single time */
    err = SF_BNO055_readRegister (  p_ctrl,
                                    BNO055_PG0_EULER_H_LSB_ADDR,
                                    tmp_buf,
                                    TMP_BUF_6BYTES_LEN );
    
    //p_ctrl->device->p_api->unlock( p_ctrl->device->p_ctrl );
    
    if( err == FSP_SUCCESS )
    {
        *heading_value = SF_BNO055_convertData( tmp_buf[0], tmp_buf[1] ) * BNO055_EULER_DATA_COEFF;
        *roll_value = SF_BNO055_convertData( tmp_buf[2], tmp_buf[3] ) * BNO055_EULER_DATA_COEFF;
        *pitch_value = SF_BNO055_convertData( tmp_buf[4], tmp_buf[5] ) * BNO055_EULER_DATA_COEFF;
    }
    
    return err;
}
/******************************************************************************************************************//**
 * @brief Implements sf_bno055_api_t::readQuaternions.
 *
 * @retval FSP_SUCCESS          Quaternions read successfully.
 * @retval SSP_ERR_ASSERTION    A pointer parameter was NULL, or a lower level driver reported this error.
 * @retval SSP_ERR_NOT_OPEN     Driver is not configured. Call SF_BNO055_Open.
 * @return                      See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
 *********************************************************************************************************************/
fsp_err_t SF_BNO055_ReadQuaternions (   sf_bno055_ctrl_t * const p_ctrl,
                                        float * w_value,
                                        float * x_value,
                                        float * y_value,
                                        float * z_value )
{
    fsp_err_t err;
    uint8_t tmp_buf[TMP_BUF_8BYTES_LEN];
    
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != p_ctrl);
    SSP_ASSERT(NULL != w_value);
    SSP_ASSERT(NULL != x_value);
    SSP_ASSERT(NULL != y_value);
    SSP_ASSERT(NULL != z_value);
    SF_BNO055_ERROR_RETURN(BNO055_INSTANCE_OPEN == p_ctrl->open, SSP_ERR_NOT_OPEN);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    //err = p_ctrl->device->p_api->lock( p_ctrl->device->p_ctrl );
    //SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /* Read the 8 registers of the quaternion data in a single time */
    err = SF_BNO055_readRegister (  p_ctrl,
                                    BNO055_PG0_QUATERNION_DATA_W_LSB_ADDR,
                                    tmp_buf,
                                    TMP_BUF_8BYTES_LEN );
    
    //p_ctrl->device->p_api->unlock( p_ctrl->device->p_ctrl );
    
    if( err == FSP_SUCCESS )
    {
        *w_value = SF_BNO055_convertData( tmp_buf[0], tmp_buf[1] ) * BNO055_QUATERNION_DATA_COEFF;
        *x_value = SF_BNO055_convertData( tmp_buf[2], tmp_buf[3] ) * BNO055_QUATERNION_DATA_COEFF;
        *y_value = SF_BNO055_convertData( tmp_buf[4], tmp_buf[5] ) * BNO055_QUATERNION_DATA_COEFF;
        *z_value = SF_BNO055_convertData( tmp_buf[6], tmp_buf[7] ) * BNO055_QUATERNION_DATA_COEFF;
    }
    
    return err;
}


/******************************************************************************************************************//**
 * @brief Implements sf_bno055_api_t::readTemperature.
 *
 * @retval FSP_SUCCESS          Temperature read successfully.
 * @retval SSP_ERR_ASSERTION    A pointer parameter was NULL, or a lower level driver reported this error.
 * @retval SSP_ERR_NOT_OPEN     Driver is not configured. Call SF_BNO055_Open.
 * @return                      See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
 *********************************************************************************************************************/
fsp_err_t SF_BNO055_ReadTemperature (  sf_bno055_ctrl_t * const p_ctrl,
                                       float * temperature)
{
    fsp_err_t err;
    uint8_t tmp_buf[TMP_BUF_1BYTE_LEN];
    
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != p_ctrl);
    SSP_ASSERT(NULL != temperature);
    SF_BNO055_ERROR_RETURN(BNO055_INSTANCE_OPEN == p_ctrl->open, SSP_ERR_NOT_OPEN);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    //err = p_ctrl->device->p_api->lock( p_ctrl->device->p_ctrl );
    //SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /* Read the temperature register */
    err = SF_BNO055_readRegister (  p_ctrl,
                                    BNO055_PG0_TEMP_ADDR,
                                    tmp_buf,
                                    TMP_BUF_1BYTE_LEN );
    
    //p_ctrl->device->p_api->unlock( p_ctrl->device->p_ctrl );
    
    if( err == FSP_SUCCESS )
    {
        *temperature = (float) *((int8_t*) &tmp_buf[0]);
    }
    
    return err;
}

/******************************************************************************************************************//**
 * @brief Implements sf_dummy_api_t::reset.
 *
 * @retval FSP_SUCCESS          BNO055 were reset successfully.
 * @retval SSP_ERR_ASSERTION    Parameter p_ctrl was NULL, or a lower level driver reported this error.
 * @retval SSP_ERR_NOT_OPEN     Driver is not configured. Call SF_BNO055_Open.
 * @return                      See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
 *********************************************************************************************************************/
fsp_err_t SF_BNO055_Reset (sf_bno055_ctrl_t * const p_ctrl)
{
    fsp_err_t err;
    
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != p_ctrl);
    SF_BNO055_ERROR_RETURN(BNO055_INSTANCE_OPEN == p_ctrl->open, SSP_ERR_NOT_OPEN);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    /** Reset device in order to stop running measurements and reset register values */
    p_ctrl->ioport->p_api->pinWrite( p_ctrl->ioport->p_ctrl, p_ctrl->reset_pin, BSP_IO_LEVEL_LOW );
    //tx_thread_sleep ( 10 );
#if (BSP_CFG_RTOS == 0)
    R_BSP_SoftwareDelay((uint32_t) 10, BSP_DELAY_UNITS_MILLISECONDS);
#elif (BSP_CFG_RTOS == 1)
    tx_thread_sleep(10);
#elif (BSP_CFG_RTOS == 2)
    vTaskDelay((TickType_t) (10 / portTICK_PERIOD_MS));
#endif
    p_ctrl->ioport->p_api->pinWrite( p_ctrl->ioport->p_ctrl, p_ctrl->reset_pin, BSP_IO_LEVEL_HIGH );
    //tx_thread_sleep ( BNO055_RESET_TIME_TICK );
#if (BSP_CFG_RTOS == 0)
    R_BSP_SoftwareDelay((uint32_t) BNO055_RESET_TIME_TICK, BSP_DELAY_UNITS_MILLISECONDS);
#elif (BSP_CFG_RTOS == 1)
    tx_thread_sleep(BNO055_RESET_TIME_TICK);
#elif (BSP_CFG_RTOS == 2)
    vTaskDelay((TickType_t) (BNO055_RESET_TIME_TICK / portTICK_PERIOD_MS));
#endif
    /* Reset I2C bus */
    //err = p_ctrl->device->p_api->reset( p_ctrl->device->p_ctrl, TX_WAIT_FOREVER );
    //SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    //err = p_ctrl->device->p_api->lock( p_ctrl->device->p_ctrl );
    //SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /* Reconfigure device */
    err = SF_BNO055_initDevice( p_ctrl );
    
    //p_ctrl->device->p_api->unlock( p_ctrl->device->p_ctrl );
    
    return err;
}

/******************************************************************************************************************//**
 * @brief Implements sf_bno055_api_t::close.
 *
 * @retval FSP_SUCCESS          BNO055 instance successfully closed and device resetted.
 * @retval SSP_ERR_ASSERTION    Parameter p_ctrl was NULL, or a lower level driver reported this error.
 * @retval SSP_ERR_NOT_OPEN     Driver is not configured. Call SF_BNO055_Open.
 * @return                      See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
 *********************************************************************************************************************/
fsp_err_t SF_BNO055_Close (sf_bno055_ctrl_t * const p_ctrl)
{
    fsp_err_t err;
    uint8_t tmp_buf[TMP_BUF_1BYTE_LEN];
    
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != p_ctrl);
    SF_BNO055_ERROR_RETURN(BNO055_INSTANCE_OPEN == p_ctrl->open, SSP_ERR_NOT_OPEN);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    //err = p_ctrl->device->p_api->lock( p_ctrl->device->p_ctrl );
    //SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /** Change device mode to suspended */
    /** Configure Device power control register */
    tmp_buf[0] = BNO055_PG0_PWR_MODE_SUSPEND;
    SF_BNO055_writeRegister (   p_ctrl,
                                BNO055_PG0_PWR_MODE_ADDR,
                                tmp_buf,
                                TMP_BUF_1BYTE_LEN);
    
    //p_ctrl->device->p_api->unlock( p_ctrl->device->p_ctrl );
    
    /** Close I2C Framework Device. */
    err = p_ctrl->device->p_api->close(p_ctrl->device->p_ctrl);

    /** Mark control block open so other tasks know it is NOT valid. */
    p_ctrl->open = BNO055_INSTANCE_CLOSED;
    
    return err;
}

/******************************************************************************************************************//**
 * @brief Implements sf_bno055_api_t::versionGet.
 *
 * @retval FSP_SUCCESS          Version returned successfully.
 * @retval SSP_ERR_ASSERTION    Parameter p_version was NULL.
 * @return                      See @ref Common_Error_Codes.
  *********************************************************************************************************************/
fsp_err_t SF_BNO055_VersionGet (void * const p_version)
{
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != p_version);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    //*p_version = g_version;
    
    return FSP_SUCCESS;
}

/******************************************************************************************************************//**
 * @brief Implements static SF_BNO055_initDevice
 *
 * @retval FSP_SUCCESS          Device initialization successfully.
 * @return                      See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
  *********************************************************************************************************************/
static fsp_err_t SF_BNO055_initDevice (sf_bno055_ctrl_t * const p_ctrl)
{
    fsp_err_t err;
    uint8_t tmp_buf[TMP_BUF_1BYTE_LEN];
    
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != p_ctrl);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    
    /** Change operating mode! Configuration mode! */
    tmp_buf[0] = BNO055_PG0_OPR_MODE_CONFIG;
    err = SF_BNO055_writeRegister ( p_ctrl,
                                    BNO055_PG0_OPR_MODE_ADDR,
                                    tmp_buf,
                                    TMP_BUF_1BYTE_LEN);
    SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /** Use the external oscillator */
    tmp_buf[0] = BNO055_PG0_SYS_TRIGGER_CLK_EXT;
    err = SF_BNO055_writeRegister ( p_ctrl,
                                    BNO055_PG0_SYS_TRIGGER_ADDR,
                                    tmp_buf,
                                    TMP_BUF_1BYTE_LEN);
    SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /** Configure Device power control register... */
    tmp_buf[0] = BNO055_PG0_PWR_MODE_NORMAL;
    err = SF_BNO055_writeRegister ( p_ctrl,
                                    BNO055_PG0_PWR_MODE_ADDR,
                                    tmp_buf,
                                    TMP_BUF_1BYTE_LEN);
    SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    
    /** Accelerometer configuration */
    err = SF_BNO055_writeRegister ( p_ctrl,
                                    BNO055_PG1_ACCEL_CONFIG_ADDR,
                                    &p_ctrl->reg_ACCEL_CONFIG,
                                    1);
    SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /** Gyroscope configuration */
    err = SF_BNO055_writeRegister ( p_ctrl,
                                    BNO055_PG1_GYRO_MODE_CONFIG_ADDR,
                                    &p_ctrl->reg_GYRO_MODE_CONFIG,
                                    1);
    SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /** Magnetometer configuration */
    err = SF_BNO055_writeRegister ( p_ctrl,
                                    BNO055_PG1_MAG_CONFIG_ADDR,
                                    &p_ctrl->reg_MAG_CONFIG,
                                    1);
    SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /** Thermometer configuration */
    err = SF_BNO055_writeRegister ( p_ctrl,
                                    BNO055_PG0_TEMP_SOURCE_ADDR,
                                    &p_ctrl->reg_TEMP_SOURCE,
                                    1);
    SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /** Sensors' unit configuration */
    err = SF_BNO055_writeRegister ( p_ctrl,
                                    BNO055_PG0_UNIT_SEL_ADDR,
                                    &p_ctrl->reg_UNIT_SEL,
                                    1);
    SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    /** Change operating mode! Sensors start reading! */
    /** After this command no new configuration can be applied */
    tmp_buf[0] = BNO055_PG0_OPR_MODE_NDOF;
    err = SF_BNO055_writeRegister ( p_ctrl,
                                    BNO055_PG0_OPR_MODE_ADDR,
                                    tmp_buf,
                                    TMP_BUF_1BYTE_LEN);
    SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
    
    return err;
}

/******************************************************************************************************************//**
 * @brief Implements static SF_BNO055_readRegister
 *
 * @retval FSP_SUCCESS          Read procedure successfully.
 * @return                      See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
  *********************************************************************************************************************/
static fsp_err_t SF_BNO055_readRegister (   sf_bno055_ctrl_t * const p_ctrl,
                                            const uint16_t reg_addr,
                                            uint8_t* buffer,
                                            const uint8_t buffer_len )
{
    fsp_err_t err;
    
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != buffer);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    if( reg_addr >= BNO055_PG1_OFFSET_ADDR )
    {
        err = SF_BNO055_selectPage( p_ctrl, BNO055_Page1 );
    }
    else
    {
        err = SF_BNO055_selectPage( p_ctrl, BNO055_Page0 );
    }
    
    if( err == FSP_SUCCESS )
    {
        uint8_t tmp_buf[TMP_BUF_1BYTE_LEN];
        
        tmp_buf[0] = (uint8_t) (reg_addr & 0x00FF);
        
        //err = p_ctrl->device->p_api->write (p_ctrl->device->p_ctrl,
        //                                    tmp_buf,
        //                                    TMP_BUF_1BYTE_LEN,
        //                                    true,       /* Restart request! */
        //                                    BNO055_I2C_TIMEOUT_TICK );
        //err = p_ctrl->device->p_api->write (p_ctrl->device->p_ctrl,
        //                                    tmp_buf,
        //                                    TMP_BUF_1BYTE_LEN,
        //                                    true);
        err = i2c_mst_write(tmp_buf, TMP_BUF_1BYTE_LEN, true);

        SF_BNO055_ERROR_RETURN(FSP_SUCCESS == err, err);
        //vTaskDelay((TickType_t) (10));
        //err = p_ctrl->device->p_api->read ( p_ctrl->device->p_ctrl,
        //                                    buffer,
        //                                    buffer_len,
        //                                    false,
        //                                    BNO055_I2C_TIMEOUT_TICK );
        //err = p_ctrl->device->p_api->read ( p_ctrl->device->p_ctrl,
        //                                    buffer,
        //                                    buffer_len,
        //                                    false );
        err = i2c_mst_read(buffer, buffer_len, false);
    }
    return err;
}

/******************************************************************************************************************//**
 * @brief Implements static SF_BNO055_writeRegister
 *
 * @retval FSP_SUCCESS          Write procedure successfully.
 * @return                      See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
  *********************************************************************************************************************/
static fsp_err_t SF_BNO055_writeRegister (  sf_bno055_ctrl_t * const p_ctrl,
                                            const uint16_t reg_addr,
                                            const uint8_t* buffer,
                                            const uint8_t buffer_len )
{
    fsp_err_t err;
    
#if SF_BNO055_CFG_PARAM_CHECKING_ENABLE
    SSP_ASSERT(NULL != buffer);
#endif /* if SF_BNO055_CFG_PARAM_CHECKING_ENABLE */
    
    if( reg_addr >= BNO055_PG1_OFFSET_ADDR )
    {
        err = SF_BNO055_selectPage( p_ctrl, BNO055_Page1 );
    }
    else
    {
        err = SF_BNO055_selectPage( p_ctrl, BNO055_Page0 );
    }
    
    if( err == FSP_SUCCESS )
    {
        uint8_t tmp_buf[TMP_BUF_3BYTES_LEN];
        uint8_t i;
        
        FSP_ASSERT( buffer_len <= (TMP_BUF_3BYTES_LEN-1) );
        
        tmp_buf[0] = (uint8_t) (reg_addr & 0x00FF);
        for( i=0 ; i<buffer_len; i++ )
        {
            tmp_buf[i+1] = buffer[i];
        }
        
        //err = p_ctrl->device->p_api->write (p_ctrl->device->p_ctrl,
        //                                    tmp_buf,
        //                                    ((uint32_t)buffer_len)+1,   /* +1 for the register address */
        //                                    false,
        //                                    BNO055_I2C_TIMEOUT_TICK );
        //err = p_ctrl->device->p_api->write (p_ctrl->device->p_ctrl,
        //                                    tmp_buf,
        //                                    ((uint32_t)buffer_len)+1,   /* +1 for the register address */
        //                                    false );
        err = i2c_mst_write(tmp_buf, ((uint32_t)buffer_len)+1, false);
    }
    return err;
}

/******************************************************************************************************************//**
 * @brief Implements static SF_BNO055_selectPage
 *
 * @retval FSP_SUCCESS                  Page selected successfully.
 * @retval FSP_ERR_INVALID_ARGUMENT     Requested page is not valid.
 * @return                              See @ref Common_Error_Codes or lower level drivers for other possible return codes.
 * @note This function is reentrant for any panel.
  *********************************************************************************************************************/
static fsp_err_t SF_BNO055_selectPage ( sf_bno055_ctrl_t * const p_ctrl,
                                        BNO055_Page_t requested_page )
{
    fsp_err_t err;
    uint8_t tmp_buf[TMP_BUF_2BYTES_LEN];
    
    if( requested_page < BNO055_PageMaxNum )
    {
        if( p_ctrl->current_page != requested_page )
        {
            tmp_buf[0] = BNO055_PG0_PAGE_ID_ADDR;   /* Page ID reg is in the same address in both pages */
            tmp_buf[1] = (requested_page == BNO055_Page0) ? BNO055_PG0_PAGE_ID_0 : BNO055_PG0_PAGE_ID_1;
            //err = p_ctrl->device->p_api->write (p_ctrl->device->p_ctrl,
            //                                    tmp_buf,
            //                                    TMP_BUF_2BYTES_LEN,
            //                                    false,
            //                                    BNO055_I2C_TIMEOUT_TICK );
            //err = p_ctrl->device->p_api->write (p_ctrl->device->p_ctrl,
            //                                    tmp_buf,
            //                                    TMP_BUF_2BYTES_LEN,
            //                                    false );
            err = i2c_mst_write(tmp_buf, TMP_BUF_2BYTES_LEN, false);
            if( err == FSP_SUCCESS )
            {
                p_ctrl->current_page = requested_page;
            }
        }
        else
        {
            /* Page already selected */
            err = FSP_SUCCESS;
        }
    }
    else
    {
        /* Invalid page requested */
        err = FSP_ERR_INVALID_ARGUMENT;
    }

    return err;
}

/******************************************************************************************************************//**
 * @brief Implements static SF_BNO055_convertData
 *
 * @retval float value obtained from the signed 16 bit integer, splitter in two bytes,
 *         passed as arguments to the function.
 * @note This function is reentrant for any panel.
  *********************************************************************************************************************/
static float SF_BNO055_convertData( uint8_t lsb, uint8_t msb )
{
    float ret;
    uint16_t tmp = (uint16_t)( (((uint16_t)msb) << 8) + ((uint16_t) lsb) );
    ret = (float) *((int16_t*) &tmp);
    return ret;
}
/******************************************************************************************************************//**
 * @}
 *********************************************************************************************************************/
