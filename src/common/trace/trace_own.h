/***********************************************************************************************************************
 * Copyright [2019] RELOC s.r.l. - All rights strictly reserved
 * This Software is provided for evaluation purposes; any other use — including reproduction, modification, use for
 * a commercial product, distribution, or republication — without the prior written permission of the Copyright holder
 * is strictly prohibited.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
 * OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 **********************************************************************************************************************/
/**********************************************************************************************************************
 * File Name    : trace_own.h
 * Description  : Trace module owner header file.
 **********************************************************************************************************************/
#ifndef TRACE_OWN_H
#define TRACE_OWN_H

FSP_HEADER

/**********************************************************************************************************************
 * Includes
 *********************************************************************************************************************/
#include "common_return.h"
#include "common_integrity.h"
#include "event_groups.h"
#include "semphr.h"
#include "trace.h"


/**********************************************************************************************************************
 * Macro definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Typedef definitions
 *********************************************************************************************************************/
struct Trace_
{
    void const* interface;
    EventGroupHandle_t uart_event_handle;
    StaticEventGroup_t uart_event_buf;
    StaticSemaphore_t mutex_buf;
    SemaphoreHandle_t mutex_handle;
#ifndef SKIP_INTEGRITY_CHECK
    Trace_t * self;
#endif
};

/**********************************************************************************************************************
 * Public functions
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * @brief   Create a new trace system
 *
 * This function create and configure a trace system module that can be used to print trace strings in a configured
 * port. Using global defines configure the trace outputs to a specific port:
 *      TRACE_ON_SERIAL     = passed interface will be used as a serial port.
 *      TRACE_ON_USB        = passed interface will be used as a usb port.
 *      TRACE_ON_VIRTUAL    = a virtual debug port is used.
 *
 * @param[in] interface                 Pointer to a generic port handle.
  *********************************************************************************************************************/
void Trace_build( void const* interface );

/***********************************************************************************************************************
 * @brief   Delete a trace system
 *
 * This function delete a previously created trace module.
  *********************************************************************************************************************/
ret_t Trace_destroy( void );

FSP_FOOTER

#endif

