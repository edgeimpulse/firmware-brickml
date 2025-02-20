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
