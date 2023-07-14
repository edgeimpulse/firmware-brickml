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
 * File Name    : trace_use.h
 * Description  : This module contains the usage API declaration of the class trace.
 **********************************************************************************************************************/
#ifndef TRACE_USE_H
#define TRACE_USE_H

#if defined(__cplusplus)
extern "C" {
#endif

/***********************************************************************************************************************
 * Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include <stdint.h>
#include <string.h>
#include "trace.h"

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Public API Function Declarations
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * @brief   Print a trace string
 *
 * This function prints a trace string.
 * 
 * @param[in] str                       String to print.
 * @param[in] str_len                   String length.
  *********************************************************************************************************************/
void Trace_write( char* str, uint8_t str_len );
void Trace_printf( char* str, ... );

/***********************************************************************************************************************
 * Macros
 **********************************************************************************************************************/
#define TRACE( str )            Trace_write( str, (uint8_t)strlen(str) );
#define TRACE2( str, ... )      Trace_printf( str, ##__VA_ARGS__ );

#if defined(__cplusplus)
}
#endif

#endif

