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
#include "i2s.h"
#include "audio/OpenPDMFilter.h"

#define DEBUG_LED                               0

#if DEBUG_LED == 1
#include "peripheral/led.h"
#endif

#define MICROPHONE_CONVERT_PCM_GAIN             1

#define SUPPORTED_FREQUENCY                     2

const uint32_t mic_pcm_frequency[SUPPORTED_FREQUENCY] = {MIC_PCM_FREQ_16_KHZ, MIC_PCM_FREQ_150_KHZ};    // 16khz and 150khz
const uint8_t mic_pcm_decimator_factor[SUPPORTED_FREQUENCY] = {MICROPHONE_PCM_DECIMATION_FACTOR_16k, MICROPHONE_PCM_DECIMATION_FACTOR_150k};
const uint32_t timer_count[SUPPORTED_FREQUENCY]  = {94, 20};    // 1.024 MHz and 4.8 MHz
const uint32_t read_size[SUPPORTED_FREQUENCY]  = {EI_I2S_READ_SAMPLES_BYTE_16k, EI_I2S_READ_SAMPLES_BYTE_150k};
int8_t decimator_in_use_index;

typedef enum
{
    e_buffer_first_one   = 0,
    e_buffer_second_one = 1,
    e_buffer_max        = 2
}t_double_buffer;

/* Global variables */
volatile bool can_read;
uint8_t _buffer[e_buffer_max][EI_I2S_READ_SAMPLES_BYTE_150k]; //  BSP_ALIGN_VARIABLE(4);  // *2 uint16 ?
volatile t_double_buffer actual_buffer;

volatile bool _close_driver = false;
volatile bool _read_enabled = false;

static uint32_t pcmSamplingF = 0;
static uint32_t pcmBufLen = 0;
TPDMFilter_InitStruct filter = {0};

#define get_read_buffer(x)  x == e_buffer_first_one ? e_buffer_second_one : e_buffer_first_one;


/**
 * @brief Initialize peripherals (ssi and timer channel 2)
 * @return
 */
int ei_i2s_driver_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    memset((void*)_buffer, 0, sizeof(_buffer));

    /* start timer */
    err = R_GPT_Open(&g_timer_i2s_ctrl, &g_timer_i2s_cfg);

    if (FSP_SUCCESS != err) {
        //
    }

    /* Open SSI module */
    err = R_SSI_Open(&g_i2s0_ctrl, &g_i2s0_cfg);
    if (FSP_SUCCESS != err) {
        //
    }
    //R_SSI0->SSICR |= R_SSI0_SSICR_BCKP_Msk; /* set bit clock polarity */
    g_i2s0_ctrl.p_reg->SSICR_b.PDTA = 0;   /* left alignment! */

    _read_enabled = false;

    actual_buffer = e_buffer_first_one;

    /* Handle error */
    if (FSP_SUCCESS != err) {
        /*
         * print error
         */
    }

    return (int)err;
}

/**
 *
 * @return
 */
int ei_i2s_init(uint32_t freq)
{
    fsp_err_t err = FSP_SUCCESS;

    can_read = false;

    decimator_in_use_index = -1;
    for (uint8_t i = 0; i < SUPPORTED_FREQUENCY; i++) {
        if (mic_pcm_frequency[i] == freq) {
            decimator_in_use_index = i;
            break;
        }
    }

    if (decimator_in_use_index == -1) {
        return 200;//?
    }

    // input parameters
    pcmSamplingF = mic_pcm_frequency[decimator_in_use_index];   // pcm
    pcmBufLen = pcmSamplingF/1000;

    /* Initialize Open PDM library */
    filter.Fs = pcmSamplingF;
    filter.nSamples = pcmBufLen;
    filter.LP_HZ = (float)(pcmSamplingF/2.0f);
    filter.HP_HZ = 10;
    filter.In_MicChannels = 1;
    filter.Out_MicChannels = 1;
    filter.Decimation = mic_pcm_decimator_factor[decimator_in_use_index];

    Open_PDM_Filter_Init(&filter);

    actual_buffer = e_buffer_first_one;

    memset((void*)_buffer, 0, sizeof(_buffer));

    /* */
    err = R_SSI_Read(&g_i2s0_ctrl, &_buffer[actual_buffer][0], read_size[decimator_in_use_index]);    /* start reading */
    if (FSP_SUCCESS != err) {
        //
    }
    else {
        _read_enabled = true;
        /* start timer */
        R_GPT_Stop(&g_timer_i2s_ctrl);  // be sure
        R_GPT_PeriodSet(&g_timer_i2s_ctrl, timer_count[decimator_in_use_index]); /* set new period */
        R_GPT_Start(&g_timer_i2s_ctrl);
    }


    return (int)err;
}

/**
 *
 * @return
 */
int ei_i2s_deinit(void)
{
    /* start timer */
    R_GPT_Stop(&g_timer_i2s_ctrl);
    //R_SSI_Stop(&g_i2s0_ctrl);

    _read_enabled = false;

    /*
    while(_close_driver == false) {
        __NOP();
    }
    */

    return 0;
}

/**
 *
 * @return
 */
int ei_i2s_driver_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;
    _close_driver = true;

    while(_close_driver) {
        __NOP();
    };

    /* Close SSI Module */
    err = R_SSI_Close(&g_i2s0_ctrl);

    /* Stop & close timer */
    R_GPT_Stop(&g_timer_i2s_ctrl);
    R_GPT_Close(&g_timer_i2s_ctrl);

    return (int)err;
}

/**
 *
 * @return
 */
bool ei_i2s_can_read(void)
{
    return can_read;
}

/**
 *
 * @param dst
 * @param max_byte
 * @return
 */
uint32_t ei_i2s_get_buffer(int16_t* dst, uint32_t samples)
{
    uint32_t return_value = 0;
    uint32_t i;
    t_double_buffer read_buffer = get_read_buffer(actual_buffer);
    uint8_t* pSrc = &_buffer[read_buffer][0];
    uint16_t buffer_scale = 0;

    if (can_read == true) {

        if (decimator_in_use_index == 0) {
            buffer_scale = MICROPHONE_BUFFER_SCALE_FACTOR_16k;
        }
        else {
            buffer_scale = MICROPHONE_BUFFER_SCALE_FACTOR_150k;
        }

        for (i = 0; i < buffer_scale; i++)
        {
            if (decimator_in_use_index == 1) {
                Open_PDM_Filter_32((uint8_t*)(pSrc + (MICROPHONE_RAW_BUFFER_SIZE_150k * i)), (int16_t*)(dst + (i * pcmBufLen)), MICROPHONE_CONVERT_PCM_GAIN, &filter);
                // return_value += MICROPHONE_RAW_BUFFER_SIZE_150k;
            }
            else if (decimator_in_use_index == 0) {
                Open_PDM_Filter_64((uint8_t*)(pSrc + (MICROPHONE_RAW_BUFFER_SIZE_16k * i)), (int16_t*)(dst + (i * pcmBufLen)), MICROPHONE_CONVERT_PCM_GAIN, &filter);
                // return_value += MICROPHONE_RAW_BUFFER_SIZE_16k;
            }
            else {
                // not supported ....
            }
        }
        return_value = samples;
        memset((void*)_buffer[read_buffer], 0, sizeof(_buffer[read_buffer]));
        can_read = false;
    }

    return (return_value);
}

/**
 * @brief 
 * 
 * @param sampling_freq 
 * @return uint32_t 
 */
uint32_t ei_i2s_get_buf_len(uint32_t sampling_freq)
{
    if (sampling_freq == MIC_PCM_FREQ_150_KHZ) {
        return MICROPHONE_PCM_REAL_BUFFER_SIZE_150k;
    }
    else if (sampling_freq == MIC_PCM_FREQ_16_KHZ) {
        return MICROPHONE_PCM_REAL_BUFFER_SIZE_16k;
    }
    else {
        return 0;
    }
}

/**
 *
 * @param p_args
 */
void i2s_callback(i2s_callback_args_t *p_args)
{
#if DEBUG_LED == 1
    static bool led_blue = false;
    static bool led_red = false;
#endif

    if( NULL != p_args)
    {
        if (_read_enabled == true)
        {
            if (p_args->event == I2S_EVENT_RX_FULL) {
                can_read = true;
                actual_buffer = get_read_buffer(actual_buffer); /* switch buffer */
                R_SSI_Read(&g_i2s0_ctrl, &_buffer[actual_buffer][0], read_size[decimator_in_use_index]);
            }
        }
        else
        {
            _close_driver = true;
        }
    }
}
