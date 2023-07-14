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
 * File Name    : common_assert.h
 * Description  : This module contains the assert definition.
 **********************************************************************************************************************/
#ifndef COMMON_ASSERT_H_
#define COMMON_ASSERT_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include "panic.h"

/***********************************************************************************************************************
 * Macros
 **********************************************************************************************************************/

#define ASSERT( a )         panic( (a), __FILE__ , __LINE__ );

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

#if defined(__cplusplus)
}
#endif

#endif
