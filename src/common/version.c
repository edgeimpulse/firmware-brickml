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
 * File Name    : version.c
 * Description  : Project versioning source file
 **********************************************************************************************************************/
#include "stdint.h"
#include "string.h"
#include "version.h"

/***********************************************************************************************************************
 * Macros
 **********************************************************************************************************************/
#define INFO_STRING_PART1       "\n\r     ===============================================\n\r" \
                                "\t  [ " PROJECT_NAME " - " BOARD_NAME " ]  \r\n" \
                                "\t  Firmware Version : " FIRMWARE_STRING " \r\n" \
                                "\t  Build time: " __DATE__ " "__TIME__ "\n\r" \
                                "     ===============================================\n\r"

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private variables
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Public API Functions Definitions
 **********************************************************************************************************************/
char* Version_getInfoStr( uint32_t* lenght )
{
    if( lenght != NULL )
    {
        *lenght = strlen(INFO_STRING_PART1);
    }
    return INFO_STRING_PART1;
}
