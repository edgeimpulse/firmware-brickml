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
 * File Name    : panic.c
 * Description  : This function is used to print a debug trace if a mandatory condition is not met. (used by ASSERT)
 **********************************************************************************************************************/
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "trace_use.h"
//#include "bsp_mcu_api.h"
#include "panic.h"

#define PANIC_STR_LEN_MAX   100

void panic( bool condition,
            char* filename,
            int line )
{
    if( condition != true )
    {
        char panicStr[ PANIC_STR_LEN_MAX ];
        
        snprintf(   panicStr,
                    PANIC_STR_LEN_MAX,
                    "PANIC in %s - line %d\r\n",
                    filename,
                    line);
        
        //TRACE (panicStr);
        
        //R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_SECONDS);
        
        //__NVIC_SystemReset();
    }
}
