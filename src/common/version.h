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
 * File Name    : version.h
 * Description  : Project versioning header file
 **********************************************************************************************************************/
#ifndef VERSION_H
#define VERSION_H

#if defined(__cplusplus)
extern "C" {
#endif

/***********************************************************************************************************************
 * Includes   <System Includes> , "Project Includes"
 **********************************************************************************************************************/
#include "stdint.h"
#include "common_macro.h"

/***********************************************************************************************************************
 * Macros
 **********************************************************************************************************************/
#define xstr(s) str(s)
#define str(s) #s
#define PROJECT_NAME                "BRICKML - RA6M5"
#define BOARD_NAME                  "EK-RA6M5 R1D0"

#define FIRMWARE_VERSION            0
#define FIRMWARE_REVISION           0
#define FIRMWARE_PATCH              1

#define FIRMWARE_STRING             xstr(FIRMWARE_VERSION) "." xstr(FIRMWARE_REVISION) "." xstr(FIRMWARE_PATCH)
#define FIRMWARE_NUMBER             ( (FIRMWARE_VERSION << 16) + (FIRMWARE_REVISION << 8) + (FIRMWARE_PATCH << 0) )

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Public API Function Declarations
 **********************************************************************************************************************/
char* Version_getInfoStr( uint32_t* lenght );

#if defined(__cplusplus)
}
#endif

#endif
