/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
