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
 * File Name    : hs300x.h
 * Description  : This module contains the type definition and the interface of the driver for the High Performance
 *                Relative Humidity and Temperature Sensor HS300X.
 * SW Platform  : Renesas RA Flexible Software Package (FSP).
 **********************************************************************************************************************/
#ifndef HS300X_H
#define HS300X_H

#if defined(__cplusplus)
extern "C" {
#endif

/***********************************************************************************************************************
 * Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include "r_i2c_master_api.h"
#include "FreeRTOS.h"
#include "event_groups.h"

/***********************************************************************************************************************
 * Macros
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
typedef struct Hs300x_
{
    const i2c_master_instance_t* p_i2c;
    EventGroupHandle_t i2c_event_handle;
} Hs300x_t;

/***********************************************************************************************************************
 * Public API Function Declarations
 **********************************************************************************************************************/
fsp_err_t Hs300x_Open( Hs300x_t* this_, const i2c_master_instance_t* i2c, EventGroupHandle_t i2c_event_handle );
fsp_err_t Hs300x_Close( Hs300x_t* this_ );
fsp_err_t Hs300x_GetMeasure( Hs300x_t* this_, float* temperature, float* humidity );


#if defined(__cplusplus)
}
#endif

#endif /* HS300X_H */
