/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef PERIPHERAL_BOARD_CTRL_H_
#define PERIPHERAL_BOARD_CTRL_H_

#define FLASH_BLOCK_SIZE                (32 * 1024)

#define PRIMARY_IMAGE_START_ADDRESS     0x00020000
#define PRIMARY_IMAGE_END_ADDRESS       0x000FFFFF
#define PRIMARY_IMAGE_NUM_BLOCKS        ((PRIMARY_IMAGE_END_ADDRESS - PRIMARY_IMAGE_START_ADDRESS) / FLASH_BLOCK_SIZE)

#define SECONDARY_IMAGE_START_ADDRESS   0x00100000
#define SECONDARY_IMAGE_END_ADDRESS     0x001FFFFF
#define SECONDARY_IMAGE_NUM_BLOCKS      ((SECONDARY_IMAGE_END_ADDRESS - SECONDARY_IMAGE_START_ADDRESS) / FLASH_BLOCK_SIZE)

typedef enum e_enable_disable
{
    DISABLE,
    RE_ENABLE
}enable_disable_t;

#if defined(__cplusplus)
extern "C" {
#endif

void ThreadsAndInterrupts(enable_disable_t EnableDisable);

#if defined(__cplusplus)
}
#endif

#endif /* PERIPHERAL_BOARD_CTRL_H_ */
