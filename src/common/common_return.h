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
 * File Name    : common_return.h
 * Description  : This module contains all the predefined return values.
 **********************************************************************************************************************/
#ifndef COMMON_RETURN_H_
#define COMMON_RETURN_H_

FSP_HEADER

/***********************************************************************************************************************
 * Macros
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
typedef enum
{
    RET_SUCCESS = 0,
    RET_FAIL = -1,
    RET_ERR_CONFIG = -2,
    RET_TIMEOUT = -3,
    RET_NOT_INITIALIZED = -4,
    RET_BUF_OVERFLOW = -5,
    RET_NOT_FREE = -6,
    RET_NOT_REGISTERED = -7,
    RET_NOT_ENABLED = -8,
    RET_ERR_FULL = -9,
    RET_ERR_BUSY = -10,
    RET_ERR_NOT_OPEN = -11,
    RET_ERR_CORRUPTED = -12,
} ret_t;

FSP_FOOTER


#endif
