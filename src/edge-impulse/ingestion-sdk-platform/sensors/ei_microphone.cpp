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
/* Include ----------------------------------------------------------------- */
#include "ei_microphone.h"
#include "peripheral/i2s.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ingestion-sdk-platform/brickml/ei_device_brickml.h"
#include "ingestion-sdk-platform/brickml/ei_qspi_memory.h"
#include "ingestion-sdk/sensor_aq_mbedtls_hs256.h"
#include "ingestion-sdk/ei_sampler.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/sensor-aq/sensor_aq.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

/* Constant ---------------------------------------------------------------- */
#define MIC_USE_DC_BLOCKING         (0u)


/** Status and control struct for inferencing struct */
typedef struct {
    int16_t     *buffers[2];
    uint8_t     buf_select;
    uint8_t     buf_ready;
    uint32_t    buf_count;
    uint32_t    n_samples;
} inference_t;

/* Private functions ------------------------------------------------ */
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM * stream);
static int ei_seek(EI_SENSOR_AQ_STREAM * stream, long int offset, int origin);
static int insert_ref(char *buffer, int hdrLength);
static bool create_header(sensor_aq_payload_info *payload);
static void ingestion_samples_callback(const int16_t *buffer, uint32_t sample_count);


/* Private variables ------------------------------------------------------- */
static inference_t inference;
static uint32_t required_samples_size;
static uint32_t headerOffset = 0;
static uint32_t current_sample;
static bool first_sample;

static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/**
 *
 * @return
 */
bool ei_microphone_sample_start(void)
{
    EiBrickml* dev = static_cast<EiBrickml*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_device_memory_in_use();

    int ret;
    uint32_t required_samples;
    uint32_t freq = (1/dev->get_sample_interval_ms())*1000;
    uint32_t i2s_buf_len = 0;

    sensor_aq_payload_info payload = {
        dev->get_device_id().c_str(),
        dev->get_device_type().c_str(),
        dev->get_sample_interval_ms(),
        { { "audio", "wav" } }
    };

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float(dev->get_sample_interval_ms());
    ei_printf(" ms.\n");
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tSensor: %s\n", (dev->get_sensor_label().c_str()));
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    required_samples = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());

    /* Round to even number of samples for word align flash write */
    if(required_samples & 1) {
        required_samples++;
    }

    required_samples_size = required_samples * sizeof(microphone_sample_t);
    current_sample = 0;

    if (required_samples_size > mem->get_available_sample_bytes()) {
        ei_printf("ERR: Sample length is too long. Maximum allowed is %lu ms at %d Hz.\r\n",
          ((mem->get_available_sample_bytes() / (freq * sizeof(microphone_sample_t))) * 1000), freq);
      return false;
    }

    uint32_t delay_time_ms = ((required_samples_size / mem->block_size) + 1) * mem->block_erase_time;
    ei_printf("Starting in %lu ms... (or until all flash was erased)\n", delay_time_ms < 2000 ? 2000 : delay_time_ms);

    dev->set_state(eiBrickMLStateErasingFlash);
    if(mem->erase_sample_data(0, required_samples_size) != (required_samples_size)) {
       return false;
    }
    
    if (mem->setup_sampling(dev->get_sensor_label().c_str(), dev->get_sample_label().c_str()) == false) {
        ei_printf("ERR: can't setup_sampling\r\n");
        return false;
    }

    // if erasing took less than 2 seconds, wait additional time
    if(delay_time_ms < 2000) {
       R_BSP_SoftwareDelay(2000 - delay_time_ms, BSP_DELAY_UNITS_MILLISECONDS);
    }

    if (create_header(&payload) == false) {
        return false;
    }

    i2s_buf_len = ei_i2s_get_buf_len((uint32_t)freq);

    if (ei_mic_init(freq) == false) {
        ei_printf("ERR: in mic initialisation\r\n");
        return false;
    }

    first_sample = true;
    ei_printf("Sampling...\r\n");
    dev->set_state(eiBrickMLStateSampling);

    while (current_sample < required_samples_size) {
        ei_mic_thread(&ingestion_samples_callback, i2s_buf_len);
        __WFI();
    }

    ei_mic_deinit();

    mem->flush_data();  /* write any additional data */    

    ret = ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);
    if (ret != 0) {
        ei_printf("Failed to finish signature (%d)\r\n", ret);
        return false;
    }

    mem->finalize_samplig();
    dev->set_state(eiBrickMLStateIdle);

    ei_printf("Done sampling, total bytes collected: %lu\n", required_samples_size);
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.\n", required_samples_size + headerOffset);
    ei_printf("OK\n");

    return true;
}

/**
 *
 * @param cb
 */
void ei_mic_thread(mic_sampler_callback cb, uint32_t buffer_len)
{
    uint16_t read_samples = 0;
    int16_t* _readl_buffer_audio;

    if (buffer_len != 0) {
        if (ei_i2s_can_read() == true) {
            
            _readl_buffer_audio = (int16_t*)ei_malloc(buffer_len*2);
            memset(_readl_buffer_audio, 0xFF, sizeof(_readl_buffer_audio));

            if (_readl_buffer_audio != nullptr) {
                read_samples = ei_i2s_get_buffer(_readl_buffer_audio, buffer_len);    /* */
            }
            
            if ((cb != nullptr)
                    && (read_samples != 0)) {
                cb(_readl_buffer_audio, read_samples);
            }

            ei_free(_readl_buffer_audio);
        }
    }

}

/**
 *
 * @param buffer
 * @param sample_count
 */
void inference_samples_callback(const int16_t *buffer, uint32_t sample_count)
{
    for(uint32_t i = 0; i < sample_count; i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = buffer[i];

        if(inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

/**
 * @brief Write mic
 *
 * @param buffer
 * @param sample_count
 */
static void ingestion_samples_callback(const int16_t *buffer, uint32_t sample_count)
{
    EiBrickml* dev = static_cast<EiBrickml*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_device_memory_in_use();

    if (first_sample == true) {
        first_sample = false;
    }
    else {
        //write raw data into memory
        mem->write_sample_data((uint8_t*)buffer, headerOffset + current_sample, (sample_count * 2));    /* *2 because we are storing samples of 16 bit */

        //update data hash
        ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)buffer, (sample_count * 2));

        current_sample += sample_count;
    }
}

/**
 *
 * @param offset
 * @param length
 * @param out_ptr
 * @return
 */
int ei_microphone_inference_get_data(size_t offset, size_t length, float *out_ptr)
{
    ei::numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    inference.buf_ready = 0;

    return 0;
}

/**
 *
 * @param n_samples
 * @param interval_ms
 * @return
 */
bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms)
{
    uint32_t freq = (1/interval_ms)*1000;
    inference.buffers[0] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if(inference.buffers[0] == NULL) {
        ei_printf("ERR: Failed to allocate audio buffer\n");
        return false;
    }

    inference.buffers[1] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if(inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        ei_printf("ERR: Failed to allocate audio buffer\n");
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    /* start mic */
    ei_mic_init(freq);

    return true;
}



/**
 *
 * @return
 */
bool ei_microphone_inference_is_recording(void)
{
    return (inference.buf_ready == 0);
}

/**
 *
 */
void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
}

/**
 *
 * @return
 */
bool ei_microphone_inference_end(void)
{
    ei_i2s_deinit();

    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);

    return true;
}

/**
 *
 */
void ei_mic_test(void)
{
    //ei_mic_init();

    while(1) {
        __WFI();
        ei_mic_thread(nullptr, 100);
    }
}

/* Private functions ------------------------------------------------------- */
/* Dummy functions for sensor_aq_ctx type */
/**
 *
 * @param
 * @param size
 * @param count
 * @param
 * @return
 */
static size_t ei_write(const void* buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM* stream)
{
    (void)buffer;
    (void)size;
    (void)stream;

    return count;
}

/**
 *
 * @param
 * @param offset
 * @param origin
 * @return
 */
static int ei_seek(EI_SENSOR_AQ_STREAM* stream, long int offset, int origin)
{
    (void)stream;
    (void)offset;
    (void)origin;

    return 0;
}

/**
 *
 * @param buffer
 * @param hdrLength
 * @return
 */
static int insert_ref(char *buffer, int hdrLength)
{
    #define EXTRA_BYTES(a)  ((a & 0x3) ? 4 - (a & 0x3) : (a & 0x03))
    const char *ref = "Ref-BINARY-i16";
    int addLength = 0;
    int padding = EXTRA_BYTES(hdrLength);

    buffer[addLength++] = 0x60 + 14 + padding;
    for(unsigned int i = 0; i < strlen(ref); i++) {
        buffer[addLength++] = *(ref + i);
    }
    for(int i = 0; i < padding; i++) {
        buffer[addLength++] = ' ';
    }

    buffer[addLength++] = 0xFF;

    return addLength;
}

/**
 *
 * @param payload
 * @return
 */
static bool create_header(sensor_aq_payload_info *payload)
{
    int ret;
    EiBrickml* dev = static_cast<EiBrickml*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_device_memory_in_use();
    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());

    ret = sensor_aq_init(&ei_mic_ctx, payload, NULL, true);

    if (ret != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", ret);
        return false;
    }

    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix != 0; ix--) {
        if (((uint8_t *)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    int ref_size = insert_ref(((char*)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), end_of_header_ix);
    // and update the signature
    ret = ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)(ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), ref_size);
    if (ret != 0) {
        ei_printf("Failed to update signature from header (%d)\n", ret);
        return false;
    }
    end_of_header_ix += ref_size;

    // Write to blockdevice
    ret = mem->write_sample_data((uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
    if ((size_t)ret != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", ret);
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}

/**
 *
 */
bool ei_mic_init(uint32_t freq)
{
    int err_code = ei_i2s_init(freq);

    if (err_code != 0) {
        return false;
    }
    return true;
}

/**
 *
 */
void ei_mic_deinit(void)
{
    ei_sleep(1000);

    ei_i2s_deinit();
}
