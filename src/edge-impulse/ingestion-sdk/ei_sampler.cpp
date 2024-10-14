/*
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/* Include ----------------------------------------------------------------- */
#include "ei_sampler.h"
#include "sensor_aq_mbedtls_hs256.h"
#include "firmware-sdk/sensor-aq/sensor_aq.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ingestion-sdk-platform/brickml/ei_device_brickml.h"
#include "ingestion-sdk-platform/brickml/ei_qspi_memory.h"
#include "bsp_api.h"

/* Forward declarations ---------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght);
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM *);
static int ei_seek(EI_SENSOR_AQ_STREAM *, long int offset, int origin);
static time_t ei_time(time_t *t);
static bool create_header(sensor_aq_payload_info *payload);
static void ei_write_last_data(void);


/* Private variables ------------------------------------------------------- */
static uint32_t samples_required;
static uint32_t current_sample;
static uint32_t sample_counter_increase;
static uint32_t sample_buffer_size;
static uint32_t headerOffset = 0;
EI_SENSOR_AQ_STREAM stream;

static uint8_t write_word_buf[4];
static int write_addr = 0;
static unsigned char ei_sensor_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_sensor_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_sensor_hs_ctx;
static sensor_aq_ctx ei_sensor_ctx = {
    { ei_sensor_ctx_buffer, 1024 },
    &ei_sensor_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/**
 * @brief      Setup and start sampling, write CBOR header to flash
 *
 * @param      v_ptr_payload  sensor_aq_payload_info pointer hidden as void
 * @param[in]  sample_size    Number of bytes for 1 sample (include all axis)
 *
 * @return     true if successful
 */
bool ei_sampler_start_sampling(void *v_ptr_payload, starter_callback ei_sample_start, uint32_t sample_size)
{
    EiBrickml* dev = static_cast<EiBrickml*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_device_memory_in_use();
    sensor_aq_payload_info *payload = (sensor_aq_payload_info *)v_ptr_payload;

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float((float)dev->get_sample_interval_ms());
    ei_printf(" ms.\n");
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tSensor: %s\n", (dev->get_sensor_label().c_str()));
    ei_printf("\tName: %s\n", (dev->get_sample_label().c_str()));
    ei_printf("\tHMAC Key: %s\n", (dev->get_sample_hmac_key().c_str()));
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    samples_required = (uint32_t)((float)dev->get_sample_length_ms());
    sample_counter_increase = (uint32_t)dev->get_sample_interval_ms();

    sample_buffer_size = ((samples_required/sample_counter_increase) * sample_size) * 2;  // why 2?
    current_sample = 0;

    uint32_t delay_time_ms = ((sample_buffer_size / mem->block_size) + 1) * mem->block_erase_time;
    ei_printf("Starting in %lu ms... (or until all flash was erased)\n", delay_time_ms < 2000 ? 2000 : delay_time_ms);

    dev->set_state(eiBrickMLStateErasingFlash);
    if(mem->erase_sample_data(0, sample_buffer_size) != (sample_buffer_size)) {
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

    if (create_header(payload) == false) {
        return false;
    }

    ei_printf("Sampling...\n");
    dev->set_state(eiBrickMLStateSampling);

    if (ei_sample_start(&sample_data_callback, dev->get_sample_interval_ms()) == false) {
        return false;
    }

    while(current_sample < samples_required) {
        dev->sample_thread();
        __WFI();    // yeah ?
    };

    ei_write_last_data();
    mem->flush_data();

    write_addr++;
    uint8_t final_byte[] = {0xff};

    int ctx_err = ei_sensor_ctx.signature_ctx->update(ei_sensor_ctx.signature_ctx, final_byte, 1);
    if (ctx_err != 0) {
        return ctx_err;
    }

    // finish the signing
    ctx_err = ei_sensor_ctx.signature_ctx->finish(ei_sensor_ctx.signature_ctx, ei_sensor_ctx.hash_buffer.buffer);
    if (ctx_err != 0) {
        ei_printf("Failed to finish signature (%d)\r\n", ctx_err);
        return false;
    }

    mem->finalize_samplig();
    dev->set_state(eiBrickMLStateIdle);

    ei_printf("Done sampling, total bytes collected: %lu\n", samples_required);
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.\n", write_addr + headerOffset);
    ei_printf("OK\n");

    return true;
}


/**
 * @brief      Write samples to FLASH in CBOR format
 *
 * @param[in]  sample_buf  The sample buffer
 * @param[in]  byteLenght  The byte lenght
 *
 * @return     true if all required samples are received. Caller should stop sampling,
 */
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght)
{
    if (byteLenght != 0) {
        sensor_aq_add_data(&ei_sensor_ctx, (float *)sample_buf, byteLenght / sizeof(float));
    }

    current_sample += sample_counter_increase;
    if(current_sample >= samples_required) {
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief      Write sample data to FLASH
 * @details    Write size is always 4 bytes to keep alignment
 *
 * @param[in]  buffer     The buffer
 * @param[in]  size       The size
 * @param[in]  count      The count
 * @param      EI_SENSOR_AQ_STREAM file pointer (not used)
 *
 * @return     number of bytes handled
 */
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM *)
{
    EiBrickml* dev = static_cast<EiBrickml*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_device_memory_in_use();

    for (size_t i = 0; i < count; i++) {
        write_word_buf[write_addr & 0x3] = *((char *)buffer + i);

        if ((++write_addr & 0x03) == 0x00) {
            mem->write_sample_data(write_word_buf, (write_addr - 4) + headerOffset, 4);
        }
    }

    return count;
}

/**
 * @brief      File handle seed function. Not used
 */
static int ei_seek(EI_SENSOR_AQ_STREAM *, long int offset, int origin)
{
    (void)origin;
    (void)offset;

    return 0;
}

/**
 * @brief      File handle time function. Not used
 */
static time_t ei_time(time_t *t)
{
    time_t cur_time = 4564867;
    if (t) *(t) = cur_time;
    return cur_time;
}

/**
 * @brief      Create and write the CBOR header to FLASH
 *
 * @param      payload  The payload
 *
 * @return     True on success
 */
static bool create_header(sensor_aq_payload_info *payload)
{
    EiBrickml* dev = static_cast<EiBrickml*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_device_memory_in_use();

    sensor_aq_init_mbedtls_hs256_context(&ei_sensor_signing_ctx, &ei_sensor_hs_ctx, dev->get_sample_hmac_key().c_str());

    int tr = sensor_aq_init(&ei_sensor_ctx, payload, NULL, true);

    if (tr != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", tr);
        return false;
    }
    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_sensor_ctx.cbor_buffer.len - 1; ix != 0; ix--) {
        if (((uint8_t *)ei_sensor_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    // Write to blockdevice
    tr = mem->write_sample_data((uint8_t*)ei_sensor_ctx.cbor_buffer.ptr, 0, end_of_header_ix);

    if (tr != end_of_header_ix) {
//        end_of_header_ix = tr;  // we can write block of 128. so could be they are different.
        ei_printf("Failed to write to header blockdevice (%d)\n", tr);
        return false;
    }

    ei_sensor_ctx.stream = &stream;

    headerOffset = end_of_header_ix;
    write_addr = 0;

    return true;
}

/**
 * @brief      Write out remaining data in word buffer to FLASH.
 *             And append CBOR end character.
 */
static void ei_write_last_data(void)
{
    EiBrickml* dev = static_cast<EiBrickml*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_device_memory_in_use();
    uint8_t fill = ((uint8_t)write_addr & 0x03);
    uint8_t insert_end_address = 0;

    if (dev->is_sd_in_use() == true) {  // if sd, i need to appen 0xFF
        if (fill != 0x03) {
            write_word_buf[fill] = 0xFF;
            mem->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset, (fill + 1));
        }
        else {
            write_word_buf[0] = 0xFF;
            mem->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset, 1);
        }
    }
    else {
        if (fill != 0x00) {
            for (uint8_t i = fill; i < 4; i++) {
                write_word_buf[i] = 0xFF;
            }

            mem->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset, 4);
            insert_end_address = 4;
        }

        /* Write appending word for end character */
        for (uint8_t i =0 ; i < 4; i++) {
            write_word_buf[i] = 0xFF;
        }
        mem->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset + insert_end_address, 4);
    }

}
