#include "FreeRTOS.h"
#include "common_data.h"
#include "../comms.h"
#include "r_usb_pcdc_api.h"
#include "r_usb_basic.h"
#include "task.h"
#include <stdbool.h>


#ifdef COMMS_USB

/* pvParameters contains TaskHandle_t */

#define USB_DATA_LEN              30000 // 1000 // 50000 // 10000 //  // 1024 //
#define MAX_PACKET_SIZE           64
#define LINE_CODING_LENGTH        7
#define CONTROL_LINE_STATE_LENGTH 1

//uint8_t g_buf[DATA_LEN];

uint32_t usb_start_timestamp = 0;
uint32_t usb_stop_timestamp = 0;
uint32_t usb_tx_msg_len = 0;
uint32_t usb_tx_msg_len_max = 0;
uint32_t usb_tx_period = 0;
uint32_t usb_tx_period_total = 0;
uint32_t usb_tx_bytes_counter = 0;
uint32_t usb_tx_period_max = 0;
uint32_t usb_tx_error_counter = 0;
uint32_t usb_tx_timeout_counter = 0;
uint32_t usb_tx_period_avg = 0;
float usb_baudrate = 1;
float usb_baudrate_max = 1;

typedef enum data_bits
{
    DATA_BITS_5 = 5,
    DATA_BITS_6,
    DATA_BITS_7,
    DATA_BITS_8,
    DATA_BITS_16 = 16,
}data_bits_t;

typedef enum parity_type
{
    PARITY_NONE = 0,
    PARITY_ODD,
    PARITY_EVEN,
    PARITY_MARK,
    PARITY_SPACE,
}parity_type_t;

typedef enum stop_bits
{
    STOP_BITS_1 = 0,
    STOP_BITS_1_5,
    STOP_BITS_2,
}stop_bits_t;

volatile usb_pcdc_linecoding_t g_line_coding_in_use = {0};
/* 115200 8n1 by default */
usb_pcdc_linecoding_t g_line_coding = {0};

volatile usb_pcdc_ctrllinestate_t g_control_line_state = {
    .bdtr = 0,
    .brts = 0,
};

usb_pcdc_linecoding_t g_line_coding_fs = {
    .dw_dte_rate    = (1843200), // 115200, //
    .b_char_format = STOP_BITS_1,
    .b_parity_type = PARITY_NONE,
    .b_data_bits   = DATA_BITS_8,
};

uint8_t g_comms_opened_flag = 0;

extern usb_instance_ctrl_t g_basic_ctrl;
extern const usb_cfg_t g_basic_cfg;

fsp_err_t comms_send_low(uint8_t * p_src, uint32_t len, uint32_t period);

void usb_cdc_rtos_callback(usb_event_info_t * event, usb_hdl_t handle, usb_onoff_t onoff)
{    
    //FSP_PARAMETER_NOT_USED(event);
    FSP_PARAMETER_NOT_USED(handle);
    FSP_PARAMETER_NOT_USED(onoff);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    usb_setup_t             setup;

    switch (event->event)
    {
        case USB_STATUS_CONFIGURED :
        break;
        case USB_STATUS_WRITE_COMPLETE :
            if (pdTRUE == xSemaphoreGiveFromISR(g_usb_write_complete_binary_semaphore, &xHigherPriorityTaskWoken))
            {
                __NOP();
            }
        break;
        case USB_STATUS_READ_COMPLETE :
            if (pdTRUE == xQueueSendFromISR(g_usb_read_queue, &event->data_size, &xHigherPriorityTaskWoken ) ) {
                __NOP();
            }

        break;
        case USB_STATUS_REQUEST : /* Receive Class Request */
            g_usb_on_usb.setupGet(event, &setup);
            if (USB_PCDC_GET_LINE_CODING == (setup.request_type & USB_BREQUEST))
            {
                g_usb_on_usb.periControlDataSet(event, (uint8_t *) &g_line_coding, LINE_CODING_LENGTH);
            }
            else if (USB_PCDC_SET_LINE_CODING == (setup.request_type & USB_BREQUEST))
            {
                /* Configure virtual UART settings */
                g_usb_on_usb.periControlDataGet(&g_basic_ctrl, (uint8_t *) &g_line_coding, LINE_CODING_LENGTH);
                if (g_line_coding.dw_dte_rate != g_line_coding_in_use.dw_dte_rate) {
                    g_line_coding_in_use.dw_dte_rate = g_line_coding.dw_dte_rate ;
                }
            }
            else if (USB_PCDC_SET_CONTROL_LINE_STATE == (event->setup.request_type & USB_BREQUEST))
            {
                //g_usb_on_usb.periControlStatusSet(&g_basic_ctrl, USB_SETUP_STATUS_ACK);
                fsp_err_t err = g_usb_on_usb.periControlDataGet(event, (uint8_t *) &g_control_line_state, sizeof(g_control_line_state));
                if (FSP_SUCCESS == err)
                {
                    g_control_line_state.bdtr = (unsigned char)((event->setup.request_value >> 0) & 0x01);
                    g_control_line_state.brts = (unsigned char)((event->setup.request_value >> 1) & 0x01);
                    g_comms_opened_flag = 1;
                    g_usb_on_usb.periControlStatusSet(&g_basic_ctrl, USB_SETUP_STATUS_ACK);
                    xSemaphoreGiveFromISR(g_usb_ready, &xHigherPriorityTaskWoken);
                }

            }
            else
            {
                /* none */
            }
        break;
        case USB_STATUS_REQUEST_COMPLETE :
            __NOP();
        break;
        case USB_STATUS_SUSPEND :
        case USB_STATUS_DETACH :
        case USB_STATUS_DEFAULT :
            __NOP();
        break;
        default :
            __NOP();
        break;
    }

    /* We can switch context if necessary. */
    /* Actual macro used here is port specific. */
    portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
}

/**
 * @brief 
 * 
 * @param wait 
 * @return fsp_err_t 
 */
fsp_err_t comms_open(uint8_t wait)
{
    fsp_err_t err;

    err = g_usb_on_usb.open(&g_basic_ctrl, &g_basic_cfg);
    if (FSP_SUCCESS != err)
    {
        return (err);
    }

    if (wait)
    {
        if ( xSemaphoreTake( g_usb_ready, portMAX_DELAY ) == pdFALSE ) {
            return FSP_ERR_ABORTED;
        }

        g_comms_opened_flag = 1;
        vTaskDelay (500);
    }
    return FSP_SUCCESS;
}

/**
 * @brief 
 * 
 * @param p_src 
 * @param len 
 * @param period 
 * @return fsp_err_t 
 */
fsp_err_t comms_send(uint8_t * p_src, uint32_t len, uint32_t period)
{
    fsp_err_t       err = FSP_SUCCESS;
    uint32_t i = 0;
    uint32_t offset = 0;
    uint32_t len1 = 0;

    if (0 == g_comms_opened_flag)
        return FSP_ERR_BLE_INIT_FAILED;

    usb_start_timestamp = xTaskGetTickCount();
#if 1
    //if (len <= USB_DATA_LEN)
    //{
    //    comms_send_low(p_src, len, period);
    //}
    //else
    {
        while(1)
        {
            i++;
            if (len - offset >= USB_DATA_LEN)
            {
                len1 = USB_DATA_LEN;
            }
            else
            {
                len1 = len - offset;
            }
            //while(1)
            {
                err = comms_send_low(&(p_src[offset]), len1, period);
                if (err == FSP_SUCCESS)
                {
                    //break;
                    offset += len1;
                }
            }

            if (offset >= len)
                break;
        }
    }
#elif 1
    err = comms_send_low(p_src, len, period);
#else
#endif
    usb_stop_timestamp = xTaskGetTickCount();

    if (err == FSP_SUCCESS)
    {
        usb_tx_msg_len = len;
        if (usb_tx_msg_len > usb_tx_msg_len_max)
            usb_tx_msg_len_max = usb_tx_msg_len;

        usb_tx_period = usb_stop_timestamp - usb_start_timestamp;
        if (usb_tx_period > usb_tx_period_max)
            usb_tx_period_max = usb_tx_period;

        if (usb_tx_bytes_counter > 1*1000*1000)
        {
            usb_tx_bytes_counter = 0;
            usb_tx_period_total = 0;
        }
        usb_tx_period_total += usb_tx_period;
        usb_tx_bytes_counter += len;
#if 0
        if (usb_tx_period != 0)
        {
            usb_baudrate = (len*1000);
            usb_baudrate /= usb_tx_period;
        }
#else
        usb_baudrate = (usb_tx_bytes_counter*1000);
        usb_baudrate /= usb_tx_period_total;
#endif
        if (usb_baudrate > usb_baudrate_max)
            usb_baudrate_max = usb_baudrate;
    }
    else
    {
        usb_baudrate = 0;
    }
    return err;
}

fsp_err_t comms_send_low(uint8_t * p_src, uint32_t len, uint32_t period)
{
    fsp_err_t       err;
    uint8_t timeout = 0;

    timeout = 2;
    timeout += len/1024;
#if 1
    err = g_usb_on_usb.write(&g_basic_ctrl, p_src, len,  USB_CLASS_PCDC);
    if (FSP_SUCCESS != err)
    {
        usb_tx_error_counter++;
        return  err;
    }

    /* Wait for the USB Write to complete */
#if 0
    if( xSemaphoreTake( g_usb_write_complete_binary_semaphore, portMAX_DELAY ) == pdTRUE )
    {
        __NOP();
    }
#else
    if (period != 0)
    {
        if (period == 0xFFFFFFFF)
        {
            if( xSemaphoreTake( g_usb_write_complete_binary_semaphore, portMAX_DELAY ) == pdTRUE )
            {
                __NOP();
            }
            else
            {
                usb_tx_timeout_counter++;
                err = FSP_ERR_ABORTED;
            }
        }
        else
        {
            //vTaskDelay (period);
            if( xSemaphoreTake( g_usb_write_complete_binary_semaphore, period ) == pdTRUE )
            {
                __NOP();
            }
            else
            {
                usb_tx_timeout_counter++;
                err = FSP_ERR_ABORTED;
            }
        }
    }
    else
    {
        vTaskDelay (1);
    }
#endif
#else
    uint8_t pipe_num = 0;
    err = g_usb_on_usb.pipeNumberGet(&g_basic_ctrl, &pipe_num);
    if (FSP_SUCCESS != err)
    {
        return;
    }

    err = g_usb_on_usb.pipeWrite(&g_basic_ctrl, p_src, len, 2);
    if (FSP_SUCCESS != err)
    {
        return;
    }

#endif
    return  err;
}

fsp_err_t comms_read(uint8_t * p_dest, uint32_t * len, uint32_t timeout_milliseconds)
{
    fsp_err_t       err             = FSP_SUCCESS;
    BaseType_t      fr_err;
    TickType_t      timeout;

    if (0 == g_comms_opened_flag)
        return FSP_ERR_BLE_INIT_FAILED;

    //err = g_usb_on_usb.read(&g_basic_ctrl, p_dest, MAX_PACKET_SIZE,  USB_CLASS_PCDC);
    err = g_usb_on_usb.read(&g_basic_ctrl, p_dest, 512,  USB_CLASS_PCDC);
    if (FSP_SUCCESS != err)
    {
        return FSP_ERR_USB_FAILED;
    }

    if(timeout_milliseconds == portMAX_DELAY)
    {
        timeout = portMAX_DELAY;
    }
    else
    {
        timeout = timeout_milliseconds / portTICK_PERIOD_MS;
    }

    /* Wait for the USB Read to complete */
    *len = 0;
    fr_err = xQueueReceive(g_usb_read_queue, len, timeout);
    if (pdTRUE == fr_err)
    {
        return FSP_SUCCESS;
    }
    else
    {
        /* If there was a timeout, we need to terminate the USB transfer */
        //trans.type            = USB_CLASS_PCDC;
        //trans.module_number   = USB_CFG_USE_USBIP;
        //err = g_usb_on_usb.stop(&g_ctrl, USB_TRANSFER_READ, &trans);
        err = g_usb_on_usb.stop(&g_basic_ctrl, USB_TRANSFER_READ, USB_CLASS_PCDC);

        *len = 0;

        return FSP_ERR_TIMEOUT;
    }
}

bool comms_get_is_open(void)
{
    return (g_comms_opened_flag != 0);
}

fsp_err_t comms_close(void)
{
    fsp_err_t err;

    err = g_usb_on_usb.close(&g_basic_ctrl);
    if (FSP_SUCCESS != err)
    {
        return (err);
    }

    return FSP_SUCCESS;
}

uint32_t usb_get_speed(void)
{
    return g_line_coding.dw_dte_rate;
}

#endif /* #ifdef COMMS_USB */
