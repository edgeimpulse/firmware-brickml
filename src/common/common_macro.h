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
 * File Name    : common_macro.h
 * Description  : This module contains some general purpose macros.
 **********************************************************************************************************************/
#ifndef COMMON_MACRO_H
#define COMMON_MACRO_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "FreeRTOS.h"

/***********************************************************************************************************************
 * Macros
 **********************************************************************************************************************/
#ifdef __cplusplus
#define GLOBAL_FUNCTION     extern "C"
#else
    #define GLOBAL_FUNCTION
#endif

#define IS_SET_BIT( reg, bitNum )   ( ( reg & ( 1 << bitNum ) ) != 0 )
#define SET_BIT( reg, bitNum )      ( reg | ( 1 << bitNum ) )
#define RESET_BIT( reg, bitNum )    ( reg & (~( 1 << bitNum )) )

#define MAX( a, b )                 ( ((a) < b) ? (b) : (a) )
#define MIN( a, b )                 ( ((a) < b) ? (a) : (b) )

#define UNUSED_PARAMETER(a)         (a=a)

#define CONV_MS_TO_TICK(a)          (uint32_t)((configTICK_RATE_HZ*(a))/1000)
#define CONV_TICK_TO_MS(a)          (uint32_t)((1000*(a))/configTICK_RATE_HZ)

#if defined(__cplusplus)
}
#endif

#endif
