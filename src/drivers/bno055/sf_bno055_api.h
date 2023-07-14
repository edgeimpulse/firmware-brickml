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
 * File Name    : sf_bno055_api.h
 * Description  : BNO055 Framework Layer Driver API.
 **********************************************************************************************************************/

#ifndef SF_BNO055_API_H
#define SF_BNO055_API_H

/*******************************************************************************************************************//**
 * @ingroup SF_Interface_Library
 * @defgroup SF_BNO055_API 9-Axis Absolute Orientation Sensor Driver Interface
 * @brief 9-Axis MEMS Accelerometer BNO055, Framework Layer, Driver Interface.
 *
 * @section SF_BNO055_API_SUMMARY Summary
 * This module is the Framework layer driver for the 9-Axis MEMS Accelerometer BNO055.
 * This Interface is implemented by @ref SF_BNO055.
 *
 * Interfaces used: @see SPI_API
 *
 * Related SSP architecture topics:
 *  - What is an SSP Interface? @ref ssp-interfaces
 *  - What is an SSP Layer? @ref ssp-predefined-layers
 *  - How to use SSP Interfaces and Modules? @ref using-ssp-modules
 *
 * BNO055 Driver Interface user manual:
 *
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "bsp_api.h"
#include "r_i2c_master_api.h"
#include "r_ioport.h"
#include "sf_bno055_cfg.h"

/**********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define SF_BNO055_API_VERSION_MAJOR (0)
#define SF_BNO055_API_VERSION_MINOR (1)

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
typedef enum BNO055_Page_tag {
    BNO055_Page0=0,
    BNO055_Page1,
    
    BNO055_PageMaxNum,
    BNO055_PageInvalid
} BNO055_Page_t;

typedef struct st_bno055_acc_cfg
{
    uint8_t             range;
    uint8_t             bandwidth;
    uint8_t             unit;
} bno055_acc_cfg_t;

typedef struct st_bno055_gyro_cfg
{
    uint8_t             range;
    uint8_t             bandwidth;
    uint8_t             unit;
} bno055_gyro_cfg_t;

typedef struct st_bno055_mag_cfg
{
    uint8_t             rate;
    uint8_t             mode;
} bno055_mag_cfg_t;

typedef struct st_bno055_temp_cfg
{
    uint8_t             source;
    uint8_t             unit;
} bno055_temp_cfg_t;

/** Channel control block. DO NOT INITIALIZE.  Initialization occurs when SF_BNO055_Open is called */
typedef struct st_sf_bno055_ctrl
{
    uint32_t                    open;                   ///< Used by driver to check if control block is valid
    BNO055_Page_t               current_page;           ///< Used to remember the current selected page
    uint8_t                     reg_GYRO_CONFIG;        ///< Value of a configuration register (used when the device must be init)
    uint8_t                     reg_GYRO_MODE_CONFIG;   ///< Value of a configuration register (used when the device must be init)
    uint8_t                     reg_MAG_CONFIG;         ///< Value of a configuration register (used when the device must be init)
    uint8_t                     reg_ACCEL_CONFIG;       ///< Value of a configuration register (used when the device must be init)
    uint8_t                     reg_UNIT_SEL;           ///< Value of a configuration register (used when the device must be init)
    uint8_t                     reg_TEMP_SOURCE;        ///< Value of a configuration register (used when the device must be init)
    const i2c_master_instance_t *   device;                 ///< I2C Framework Device
    ioport_port_pin_t           reset_pin;              ///< Reset Pin
    const ioport_instance_t*    ioport;                 ///< Point to ioport instance
} sf_bno055_ctrl_t;

/** Configuration for RTOS integrated BNO055 Driver. */
typedef struct st_sf_bno055_cfg
{
    const i2c_master_instance_t *   device;             ///< I2C Framework Device
    const ioport_port_pin_t     reset_pin;              ///< Reset Pin
    const ioport_instance_t*    ioport;                 ///< Point to ioport instance
    const bno055_acc_cfg_t      acc_cfg;                ///< Accelerometer configuration
    const bno055_gyro_cfg_t     gyro_cfg;               ///< Gyroscope configuration
    const bno055_mag_cfg_t      mag_cfg;                ///< Magnetometer configuration
    const bno055_temp_cfg_t     temp_cfg;               ///< Thermometer configuration
    const uint8_t               euler_unit;             ///< Euler data unit
    const uint8_t               data_format;            ///< Funsion data format (euler data only)
} sf_bno055_cfg_t;


/** BNO055 Driver API structure. */
typedef struct st_sf_bno055_api
{
    /** Open SPI Framework Driver and initialize internal constant values.
     * @par Implemented as
     *  - SF_BNO055_Open()
     *
     * @param[in,out] p_ctrl   Pointer to a structure allocated by user. This control structure is initialized in
     *                         this function.
     * @param[in]     p_cfg    Pointer to configuration structure. All elements of the structure must be set by user.
     */
    fsp_err_t (* open)(sf_bno055_ctrl_t      * const p_ctrl,
                       sf_bno055_cfg_t const * const p_cfg);
    
    /** Sample XYZ-axes accelerations. Measured value will be expressed in 'g' or 'm/s2' unit, depending on configuration.
     * @par Implemented as
     *  - SF_BNO055_ReadAccelerometer()
     *
     * @param[in]   p_ctrl      Handle set in bno055_api_t::open.
     * @param[in]   x_value     Pointer in which the X-axis acceleration value will be stored ['g' or 'm/s2'].
     * @param[in]   y_value     Pointer in which the Y-axis acceleration value will be stored ['g' or 'm/s2'].
     * @param[in]   z_value     Pointer in which the Z-axis acceleration value will be stored ['g' or 'm/s2'].
     */
    fsp_err_t (* readAccelerometer)(sf_bno055_ctrl_t * const p_ctrl,
                                    float * x_value,
                                    float * y_value,
                                    float * z_value );
    
    /** Sample XYZ-axes angular rates. Measured value will be expressed in 'dps' or 'rps' unit, depending on configuration.
     * @par Implemented as
     *  - SF_BNO055_ReadGyroscope()
     *
     * @param[in]   p_ctrl      Handle set in bno055_api_t::open.
     * @param[in]   x_value     Pointer in which the X-axis angular rate value will be stored ['dps' or 'rps'].
     * @param[in]   y_value     Pointer in which the Y-axis angular rate value will be stored ['dps' or 'rps'].
     * @param[in]   z_value     Pointer in which the Z-axis angular rate value will be stored ['dps' or 'rps'].
     */
    fsp_err_t (* readGyroscope)(sf_bno055_ctrl_t * const p_ctrl,
                                float * x_value,
                                float * y_value,
                                float * z_value );
    
    /** Sample XYZ-axes magnetic field strength. Measured value will be expressed in 'uT' unit.
     * @par Implemented as
     *  - SF_BNO055_ReadMagnetometer()
     *
     * @param[in]   p_ctrl      Handle set in bno055_api_t::open.
     * @param[in]   x_value     Pointer in which the X-axis magnetic field strength value will be stored [uT].
     * @param[in]   y_value     Pointer in which the Y-axis magnetic field strength value will be stored [uT].
     * @param[in]   z_value     Pointer in which the Z-axis magnetic field strength value will be stored [uT].
     */
    fsp_err_t (* readMagnetometer)( sf_bno055_ctrl_t * const p_ctrl,
                                    float * x_value,
                                    float * y_value,
                                    float * z_value );
    
    /** Sample Pitch, Roll and Heading. Measured value will be expressed in 'degree' or 'radiant' unit, depending on configuration.
     * @par Implemented as
     *  - SF_BNO055_ReadEulerAngles()
     *
     * @param[in]   p_ctrl          Handle set in bno055_api_t::open.
     * @param[in]   pitch_value     Pointer in which the Pitch value will be stored ['degree' or 'radiant'].
     * @param[in]   roll_value      Pointer in which the Roll value will be stored ['degree' or 'radiant'].
     * @param[in]   heading_value   Pointer in which the Heading value will be stored ['degree' or 'radiant'].
     */
    fsp_err_t (* readEulerAngles) ( sf_bno055_ctrl_t * const p_ctrl,
                                    float * pitch_value,
                                    float * roll_value,
                                    float * heading_value );
    
    /** Sample Quaternions.
     * @par Implemented as
     *  - SF_BNO055_ReadQuaternions()
     *
     * @param[in]   p_ctrl          Handle set in bno055_api_t::open.
     * @param[in]   w_value         Pointer in which the W value will be stored.
     * @param[in]   x_value         Pointer in which the X value will be stored.
     * @param[in]   y_value         Pointer in which the Y value will be stored.
     * @param[in]   z_value         Pointer in which the Z value will be stored.
     */
    fsp_err_t (* readQuaternions) ( sf_bno055_ctrl_t * const p_ctrl,
                                    float * w_value,
                                    float * x_value,
                                    float * y_value,
                                    float * z_value );
    
    /** Sample temperature. Measured value will be expressed in Celsius or Fahrenheit degree, depending on configuration.
     * @par Implemented as
     *  - SF_BNO055_ReadTemperature()
     *
     * @param[in]   p_ctrl       Pointer to control block set in temperature_sensor_api_t::open.
     * @param[in]   temperature  Pointer in which the temperature value will be stored ['°C' or '°F'].
     */
    fsp_err_t (* readTemperature)(  sf_bno055_ctrl_t * const p_ctrl,
                                    float * temperature );
    
    /** Reset BNO055 device.
     * @par Implemented as
     *  - SF_BNO055_Reset()
     *
     * @param[in]   p_ctrl      Handle set in bno055_api_t::open.
     */
    fsp_err_t (* reset)(sf_bno055_ctrl_t * const p_ctrl);
    
    /** Close SPI channel at Framework layer.
     * @par Implemented as
     *  - SF_BNO055_Close()
     *
     * @param[in]   p_ctrl       Handle set in bno055_api_t::open.
     */
    fsp_err_t (* close)(sf_bno055_ctrl_t      * const p_ctrl);

    /** Gets version and stores it in provided pointer p_version.
     * @par Implemented as
     *  - SF_BNO055_VersionGet()
     *
     * @param[out]  p_version  Code and API version used stored here.
     */
    fsp_err_t (* versionGet)(void     * const p_version);
} sf_bno055_api_t;

/** This structure encompasses everything that is needed to use an instance of this interface. */
typedef struct st_sf_bno055_instance
{
    sf_bno055_ctrl_t      * p_ctrl;    ///< Pointer to the control structure for this instance
    sf_bno055_cfg_t const * p_cfg;     ///< Pointer to the configuration structure for this instance
    sf_bno055_api_t const * p_api;     ///< Pointer to the API structure for this instance
} sf_bno055_instance_t;


/*******************************************************************************************************************//**
 * @} (end defgroup SF_BNO055_API)
 **********************************************************************************************************************/
#endif /* SF_BNO055_API_H */
