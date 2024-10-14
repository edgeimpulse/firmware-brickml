/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/* Include ----------------------------------------------------------------- */
#include "i2c.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "common_assert.h"
#include "r_i2c_master_api.h"
#include "ei_main_thread.h"

/* Global Variables -------------------------------------------------------- */
#define RESET_VALUE 0
#define I2C_TIMEOUT_TICKS                           (1000)  /*Timeout for the I"C transmission and Reception*/

EventGroupHandle_t i2c_riic0_event_handle = {0};
StaticEventGroup_t i2c_riic1_event_buf = {0};


/* Public functions -------------------------------------------------------- */
/**
 * @brief Init I2C peripheral
 * @return
 *
 * @note default speed is 100kHz
 */
int ei_i2c_init(void)
{
    fsp_err_t ret = FSP_SUCCESS;
    /*Try to Open the connection...*/
    ret = g_i2c_master0.p_api->open(g_i2c_master0.p_ctrl, g_i2c_master0.p_cfg);

    if(ret == FSP_SUCCESS)
    {
        ret = g_i2c_master0.p_api->slaveAddressSet( g_i2c_master0.p_ctrl, 0x44, I2C_MASTER_ADDR_MODE_7BIT );
    }

    i2c_riic0_event_handle = xEventGroupCreateStatic(&i2c_riic1_event_buf);

    return (int)ret;
}

/**
 *
 * @return
 */
int ei_i2c_deinit(void)
{
    fsp_err_t err     = FSP_SUCCESS;

    err = g_i2c_master0.p_api->close(g_i2c_master0.p_ctrl);


    return (int)err;
}
/**
 * @brief Wrapper for reloc driver
 * @param buf
 * @param buf_len
 * @param restart
 * @return
 */
fsp_err_t i2c_mst_write( uint8_t* buf, uint32_t buf_len, bool restart )
{
    fsp_err_t ret = FSP_SUCCESS;
    EventBits_t events = {0};

    ret = g_i2c_master0.p_api->write( g_i2c_master0.p_ctrl, buf, buf_len, restart);
    FSP_ERROR_RETURN( (ret == FSP_SUCCESS), ret );

    /* Wait for TX end */
    events = xEventGroupWaitBits( i2c_riic0_event_handle,
                                  (1 << I2C_MASTER_EVENT_TX_COMPLETE),
                                  pdTRUE,
                                  pdFALSE,
                                  I2C_TIMEOUT_TICKS );

    FSP_ERROR_RETURN( ((events & (1 << I2C_MASTER_EVENT_TX_COMPLETE)) != 0), FSP_ERR_ABORTED );

    return ret;
}

/**
 * @brief wrapper for reloc driver
 * @param buf
 * @param buf_len
 * @param restart
 * @return
 */
fsp_err_t i2c_mst_read( uint8_t* buf, uint32_t buf_len, bool restart )
{
    fsp_err_t ret = FSP_SUCCESS;
    EventBits_t events = {0};

    ret = g_i2c_master0.p_api->read( g_i2c_master0.p_ctrl, buf, buf_len, restart);

    FSP_ERROR_RETURN( (ret == FSP_SUCCESS), ret );
    /* Wait for RX end */
    events = xEventGroupWaitBits( i2c_riic0_event_handle,
                                  (1 << I2C_MASTER_EVENT_RX_COMPLETE),
                                  pdTRUE,
                                  pdFALSE,
                                  I2C_TIMEOUT_TICKS );
    FSP_ERROR_RETURN( ((events & (1 << I2C_MASTER_EVENT_RX_COMPLETE)) != 0), FSP_ERR_ABORTED );

    ret = FSP_SUCCESS;

    return ret;
}

void ei_i2c_set_slave_address(uint8_t slave_address)
{
    g_i2c_master0.p_api->slaveAddressSet( g_i2c_master0.p_ctrl, slave_address, I2C_MASTER_ADDR_MODE_7BIT );
}


/**
 *
 * @param p_args
 */
void i2c_master_callback(i2c_master_callback_args_t *p_args)
{
    if (NULL != p_args)
    {
        BaseType_t pxHigherPriorityTaskWoken;

        xEventGroupSetBitsFromISR( i2c_riic0_event_handle,
                                   (1 << p_args->event),
                                   &pxHigherPriorityTaskWoken );
    }
}
/* Private functions -------------------------------------------------------- */
