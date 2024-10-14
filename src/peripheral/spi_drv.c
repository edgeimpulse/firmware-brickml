/*
 * Copyright (c) 2022 EdgeImpulse Inc.
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
#include "FreeRTOS.h"
#include "event_groups.h"
#include "spi_drv.h"
#include <stdio.h>
#if (BRICKML_SOM == 0)
#include "r_spi_api.h"
#endif
#include "hal_data.h"

/* Global Variables -------------------------------------------------------- */
#define RESET_VALUE         0x00
#define SPI_TIMEOUT_TICKS                           (1000)  /*Timeout for the I"C transmission and Reception*/

EventGroupHandle_t spi_event_handle = {0};
StaticEventGroup_t spi_event_buf = {0};

/* Public functions -------------------------------------------------------- */
/**
 *
 * @return
 */
int spi_init(void)
{
#if (BRICKML_SOM == 0)
    fsp_err_t err = FSP_SUCCESS;

    err = R_SPI_Open (&g_spi0_ctrl, &g_spi0_cfg);
    if (FSP_SUCCESS != err) {
        return err;
    }

    spi_event_handle = xEventGroupCreateStatic(&spi_event_buf);
#else
    fsp_err_t err = FSP_ERR_UNSUPPORTED;
#endif
    return err;
}

#if (BRICKML_SOM == 0)
/**
 *
 * @param p_args
 */
void spi_callback(spi_callback_args_t * p_args)
{
    if (NULL != p_args)
    {
        BaseType_t pxHigherPriorityTaskWoken;

        xEventGroupSetBitsFromISR( spi_event_handle,
                                   (1 << p_args->event),
                                   &pxHigherPriorityTaskWoken );
    }
}
#endif

/**
 *
 * @param data
 * @param bytes
 * @return
 */
int spi_write(uint8_t *data, uint32_t bytes)
{
#if (BRICKML_SOM == 0)
    fsp_err_t err = FSP_SUCCESS;
    EventBits_t events = {0};

    err = R_SPI_Write(&g_spi0_ctrl, data, bytes, SPI_BIT_WIDTH_8_BITS);
    if(FSP_SUCCESS != err) {
        return err;
    }

    /* Wait for RX end */
    events = xEventGroupWaitBits( spi_event_handle,
                                  (1 << SPI_EVENT_TRANSFER_COMPLETE),
                                  pdTRUE,
                                  pdFALSE,
                                  SPI_TIMEOUT_TICKS );

    FSP_ERROR_RETURN( ((events & (1 << SPI_EVENT_TRANSFER_COMPLETE)) != 0), FSP_ERR_ABORTED );
    return FSP_SUCCESS;
#else
    return FSP_ERR_UNSUPPORTED;
#endif
}

/**
 *
 * @param buf
 * @param size
 * @return
 */
int spi_read(uint8_t *buf, uint32_t size)
{
#if (BRICKML_SOM == 0)
    fsp_err_t err = FSP_SUCCESS;
    EventBits_t events = {0};

    err = R_SPI_Read(&g_spi0_ctrl, buf, size, SPI_BIT_WIDTH_8_BITS);
    if(FSP_SUCCESS != err) {
        return err;
    }

    /* Wait for RX end */
    events = xEventGroupWaitBits( spi_event_handle,
                                  (1 << SPI_EVENT_TRANSFER_COMPLETE),
                                  pdTRUE,
                                  pdFALSE,
                                  SPI_TIMEOUT_TICKS );

    FSP_ERROR_RETURN( ((events & (1 << SPI_EVENT_TRANSFER_COMPLETE)) != 0), FSP_ERR_ABORTED );

    return FSP_SUCCESS;
#else
    return FSP_ERR_UNSUPPORTED;
#endif
}

/**
 *
 * @param out
 * @param in
 * @param size
 * @return
 */
int spi_write_read(const uint8_t *out, uint8_t *in, uint32_t size)
{
#if (BRICKML_SOM == 0)
    fsp_err_t err = FSP_SUCCESS;
    EventBits_t events = {0};

    err = R_SPI_WriteRead(&g_spi0_ctrl, out, in, size, SPI_BIT_WIDTH_8_BITS);
    if(FSP_SUCCESS != err) {
        return err;
    }

    /* Wait for RX end */
    events = xEventGroupWaitBits( spi_event_handle,
                                  (1 << SPI_EVENT_TRANSFER_COMPLETE),
                                  pdTRUE,
                                  pdFALSE,
                                  SPI_TIMEOUT_TICKS );

    FSP_ERROR_RETURN( ((events & (1 << SPI_EVENT_TRANSFER_COMPLETE)) != 0), FSP_ERR_ABORTED );

    return FSP_SUCCESS;
#else
    return FSP_ERR_UNSUPPORTED;
#endif
}
