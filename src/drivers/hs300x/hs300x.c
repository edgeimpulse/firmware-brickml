/***********************************************************************************************************************
 * Copyright [2020] RELOC s.r.l. - All rights strictly reserved
 * This Software is provided for evaluation purposes; any other use - including reproduction, modification, use for
 * a commercial product, distribution, or republication - without the prior written permission of the Copyright holder
 * is strictly prohibited. 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
 * OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * File Name    : hs300x.c
 * Description  : This module contains the implementation of the driver for the High Performance Relative Humidity and
 *                Temperature Sensor HS300X.
 * SW Platform  : Renesas RA Flexible Software Package (FSP).
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include "FreeRTOS.h"
#include "event_groups.h"
#include "r_i2c_master_api.h"
#include "hs300x.h"

/***********************************************************************************************************************
 * Macros
 **********************************************************************************************************************/
#define HS300X_IC2_ADDR                 0x44

#define HS300X_MEAS_REQUEST             0x00
#define HS300X_DATA_LEN_BYTE            4

#define HS300X_WAKEUP_TIME_MS           1
#define HS300X_MEASURE_TIME_MS          100
#define HS300X_EVENT_WAIT_TIMEOUT_MS    1000

#define HS300X_WAKEUP_TIME_TICKS        pdMS_TO_TICKS(HS300X_WAKEUP_TIME_MS)
#define HS300X_MEASURE_TIME_TICKS       pdMS_TO_TICKS(HS300X_MEASURE_TIME_MS)
#define HS300X_EVENT_WAIT_TIMEOUT_TICKS pdMS_TO_TICKS(HS300X_EVENT_WAIT_TIMEOUT_MS)


#define HS300X_ASSERT_RET( cond, ret )  FSP_ERROR_RETURN( cond, ret )

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private variables
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Public API Functions Definitions
 **********************************************************************************************************************/

/******************************************************************************************************************//***
 * Opens the Hs300x driver.
 *
 * @param[in] this_                     Pointer to an instance of the Hs300x module.
 * @param[in] p_i2c                     Pointer to the i2c bus connected to the sensor.
 * @param[in] i2c_event_handle          Event handler where the i2c events will be received.
 *
 * @retval  FSP_SUCCESS                 Module successfully opened.
 * @retval  FSP_ERR_INVALID_ARGUMENT    One or more arguments are NULL.
***********************************************************************************************************************/
fsp_err_t Hs300x_Open( Hs300x_t* this_, const i2c_master_instance_t* p_i2c, EventGroupHandle_t i2c_event_handle )
{
    fsp_err_t ret = FSP_SUCCESS;
    HS300X_ASSERT_RET( this_ != NULL, FSP_ERR_INVALID_ARGUMENT );
    HS300X_ASSERT_RET( p_i2c != NULL, FSP_ERR_INVALID_ARGUMENT );
    HS300X_ASSERT_RET( i2c_event_handle != NULL, FSP_ERR_INVALID_ARGUMENT );
    
    this_->p_i2c = p_i2c;
    this_->i2c_event_handle = i2c_event_handle;
    
    return ret;
}

/******************************************************************************************************************//***
 * Closes the Hs300x driver.
 *
 * @param[in] this_                     Pointer to an opened instance of the Hs300x module.
 *
 * @retval  FSP_SUCCESS                 Module successfully closed.
 * @retval  FSP_ERR_INVALID_ARGUMENT    Module instance is NULL.
***********************************************************************************************************************/
fsp_err_t Hs300x_Close( Hs300x_t* this_ )
{
    HS300X_ASSERT_RET( this_ != NULL, FSP_ERR_INVALID_ARGUMENT );
    
    this_->p_i2c = NULL;
    this_->i2c_event_handle = NULL;
    
    return FSP_SUCCESS;
}

/******************************************************************************************************************//***
 * Read temperature and humidity from the sensor.
 *
 * @param[in] this_                     Pointer to an opened instance of the Hs300x module.
 * @param[in] p_temperature             Pointer to a float variable where the temperature value [°C] will be written to.
 * @param[in] p_humidity                Pointer to a float variable where the humidity value [%] will be written to.
 *
 * @retval  FSP_SUCCESS                 Temperature and humidity read successfully.
 * @retval  FSP_ERR_INVALID_ARGUMENT    One or more arguments are NULL.
 * @retval  FSP_ERR_NOT_OPEN            Module instance not opened.
 * @retval  FSP_ERR_ABORTED             I2C communication failed, Hs300x is not responding?
***********************************************************************************************************************/
fsp_err_t Hs300x_GetMeasure( Hs300x_t* this_, float* p_temperature, float* p_humidity )
{
    EventBits_t events;
    uint8_t buf[HS300X_DATA_LEN_BYTE];
    fsp_err_t ret = FSP_SUCCESS;
    
    HS300X_ASSERT_RET( this_ != NULL, FSP_ERR_INVALID_ARGUMENT );
    HS300X_ASSERT_RET( p_temperature != NULL, FSP_ERR_INVALID_ARGUMENT );
    HS300X_ASSERT_RET( p_humidity != NULL, FSP_ERR_INVALID_ARGUMENT );
    HS300X_ASSERT_RET( this_->p_i2c != NULL, FSP_ERR_NOT_OPEN );
    HS300X_ASSERT_RET( this_->i2c_event_handle != NULL, FSP_ERR_NOT_OPEN );
    
    ret = this_->p_i2c->p_api->slaveAddressSet( this_->p_i2c->p_ctrl, HS300X_IC2_ADDR, I2C_MASTER_ADDR_MODE_7BIT );
    HS300X_ASSERT_RET( (ret == FSP_SUCCESS), ret );
    
    /* Make a dummy write request in order to trigger a measurement request */
    /* Actually no data in required but the lower level I2C driver requires at least 1 byte */
    buf[0] = HS300X_MEAS_REQUEST;
    ret = this_->p_i2c->p_api->write( this_->p_i2c->p_ctrl, buf, 1, false );
    HS300X_ASSERT_RET( (ret == FSP_SUCCESS), ret );
    /* Wait for TX end */
    events = xEventGroupWaitBits( this_->i2c_event_handle,
                                  (1 << I2C_MASTER_EVENT_TX_COMPLETE),
                                  pdTRUE,
                                  pdFALSE,
                                  HS300X_EVENT_WAIT_TIMEOUT_TICKS );
    HS300X_ASSERT_RET( ((events & (1 << I2C_MASTER_EVENT_TX_COMPLETE)) != 0), FSP_ERR_ABORTED );
    
    /* Wait for measurement */
    vTaskDelay( HS300X_MEASURE_TIME_TICKS + HS300X_WAKEUP_TIME_TICKS );
    
    
    /* Read results */
    if (this_->p_i2c->p_api->read( this_->p_i2c->p_ctrl, buf, HS300X_DATA_LEN_BYTE, false ) == FSP_SUCCESS)
    {
        /* Wait for RX end */
        events = xEventGroupWaitBits( this_->i2c_event_handle,
                                      (1 << I2C_MASTER_EVENT_RX_COMPLETE),
                                      pdTRUE,
                                      pdFALSE,
                                      HS300X_EVENT_WAIT_TIMEOUT_TICKS );
        HS300X_ASSERT_RET( ((events & (1 << I2C_MASTER_EVENT_RX_COMPLETE)) != 0), FSP_ERR_ABORTED );
        
        /* Data conversion, see Section 7 of HS300X Datasheet */
        *p_humidity = (float)((((float)(((buf[0] & 0x3f) * 0x100) + buf[1])) / 16383.0) * 100.0);
        *p_temperature = (float)((((float)((unsigned short)((buf[2]) * 0x100 + buf[3]) >> 2)) / 16383.0) * 165.0 - 40.0);
    }
    
    return ret;
}
