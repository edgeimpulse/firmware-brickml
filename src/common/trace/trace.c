/***********************************************************************************************************************
 * Copyright [2019] RELOC s.r.l. - All rights strictly reserved
 * This Software is provided for evaluation purposes; any other use - including reproduction, modification, use for
 * a commercial product, distribution, or republication - without the prior written permission of the Copyright holder
 * is strictly prohibited. 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
 * OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * File Name    : trace.c
 * Description  : This module contains a class used to provide a trace system.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <stdarg.h>
#include "common_integrity.h"
#include "common_assert.h"
#include "common_macro.h"
#include "trace_own.h"
#include "trace_use.h"
#include "r_uart_api.h"
#if defined(TRACE_ON_USB)
#include "comms/comms.h"
#endif

/***********************************************************************************************************************
 * Macros
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
#define TRACE_WAIT_TIMEOUT_TICKS            1000

/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
#if defined(TRACE_ON_SERIAL)
void uart_trace_cbk(uart_callback_args_t *p_args);
#endif

/***********************************************************************************************************************
 * Private variables
 **********************************************************************************************************************/
static Trace_t local_this;
static Trace_t* this_ = &local_this;

/***********************************************************************************************************************
 * Public API Function Definitions
 **********************************************************************************************************************/
void Trace_build( void const* interface )
{
#if defined(TRACE_ON_SERIAL)
    this_->interface = interface;
    if (interface != NULL)
    {
        uart_instance_t* serial = (uart_instance_t*) this_->interface;
        
        this_->uart_event_handle = xEventGroupCreateStatic( &this_->uart_event_buf );
        
        this_->mutex_handle = xSemaphoreCreateMutexStatic( &this_->mutex_buf );

        serial->p_api->open( serial->p_ctrl, serial->p_cfg );
        
    }
#else
    this_->interface = interface;
#endif
#if defined(TRACE_ON_USB)

#endif
    INSTANCE_INTEGRITY_SET( this_ )
}

ret_t Trace_destroy( void )
{
    INSTANCE_INTEGRITY_CHECK( this_ )
    INSTANCE_INTEGRITY_DESTROY( this_ )
    return RET_SUCCESS;
}


void Trace_write( char* str, uint8_t str_len )
{
    INSTANCE_INTEGRITY_CHECK( this_ )
#if 1
    if ( pdTRUE == xSemaphoreTake( this_->mutex_handle, TRACE_WAIT_TIMEOUT_TICKS ) )
    {
#if defined(TRACE_ON_SERIAL)
        fsp_err_t err = FSP_SUCCESS;
        uart_instance_t* serial = (uart_instance_t*) this_->interface;
        err = serial->p_api->write( serial->p_ctrl, (uint8_t*)str, str_len );
        if (err == FSP_SUCCESS)
        {
            xEventGroupWaitBits( this_->uart_event_handle,
                                          (1 << UART_EVENT_TX_COMPLETE),
                                          pdTRUE,
                                          pdFALSE,
                                          TRACE_WAIT_TIMEOUT_TICKS );
        }
#endif
#if defined(TRACE_ON_USB)

        err = comms_send((uint8_t*)str, str_len, portMAX_DELAY);
        if (err == FSP_SUCCESS)
        {

        }
        else
        {

        }
#endif
        xSemaphoreGive( this_->mutex_handle );
    }
#else
    UNUSED_PARAMETER( str );
    UNUSED_PARAMETER( str_len );
#endif
}

void Trace_printf( char* str, ... )
{    
    va_list ap;
    char buf[128] = {0};
    
    va_start(ap, str);
    vsnprintf(buf, sizeof(buf), str, ap);
    va_end(ap);
    TRACE( buf );
}




void uart_trace_cbk(uart_callback_args_t *p_args)
{
#if defined(TRACE_ON_SERIAL)
    BaseType_t pxHigherPriorityTaskWoken;
    
    xEventGroupSetBitsFromISR( this_->uart_event_handle,
                               (1 << p_args->event),
                               &pxHigherPriorityTaskWoken );
#endif
}

