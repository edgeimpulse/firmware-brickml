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
#ifndef PERIPHERAL_I2S_H_
#define PERIPHERAL_I2S_H_

#include "hal_data.h"

// --- 150 kHz
#define MIC_PCM_FREQ_150_KHZ                            (150000u)   // 150 kHz
#define MICROPHONE_PCM_DECIMATION_FACTOR_150k           (32U)
#define MICROPHONE_BUFFER_SCALE_FACTOR_150k             (50)

// PCM buffer len
#define MICROPHONE_PCM_BUFFER_SIZE_150k                 (MIC_PCM_FREQ_150_KHZ/1000)
#define MICROPHONE_PCM_REAL_BUFFER_SIZE_150k            (MICROPHONE_PCM_BUFFER_SIZE_150k * MICROPHONE_BUFFER_SCALE_FACTOR_150k)

// PDM buffer len
#define MICROPHONE_RAW_BUFFER_SIZE_150k                 (MICROPHONE_PCM_BUFFER_SIZE_150k * (MICROPHONE_PCM_DECIMATION_FACTOR_150k/8))
#define EI_I2S_READ_SAMPLES_BYTE_150k                   (MICROPHONE_RAW_BUFFER_SIZE_150k * MICROPHONE_BUFFER_SCALE_FACTOR_150k)

// ----------------------------------------------------
// --- 16 Khz ---
#define MIC_PCM_FREQ_16_KHZ                             (16000)
#define MICROPHONE_PCM_DECIMATION_FACTOR_16k            (64U)
#define MICROPHONE_BUFFER_SCALE_FACTOR_16k              (50)  //

#define MICROPHONE_PCM_BUFFER_SIZE_16k                  (MIC_PCM_FREQ_16_KHZ/1000)
#define MICROPHONE_PCM_REAL_BUFFER_SIZE_16k             (MICROPHONE_PCM_BUFFER_SIZE_16k * MICROPHONE_BUFFER_SCALE_FACTOR_16k)

// PDM buffer len
#define MICROPHONE_RAW_BUFFER_SIZE_16k                  (MICROPHONE_PCM_BUFFER_SIZE_16k * (MICROPHONE_PCM_DECIMATION_FACTOR_16k/8))
#define EI_I2S_READ_SAMPLES_BYTE_16k                    (MICROPHONE_RAW_BUFFER_SIZE_16k * MICROPHONE_BUFFER_SCALE_FACTOR_16k)

FSP_CPP_HEADER

extern int ei_i2s_driver_init(void);
extern int ei_i2s_driver_deinit(void);
extern int ei_i2s_init(uint32_t freq);
extern int ei_i2s_deinit(void);
extern bool ei_i2s_can_read(void);
extern uint32_t ei_i2s_get_buffer(int16_t* dst, uint32_t max_byte);
extern uint32_t ei_i2s_get_buf_len(uint32_t sampling_freq);

FSP_CPP_FOOTER

#endif /* PERIPHERAL_I2S_H_ */
