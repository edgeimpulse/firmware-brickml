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

#ifndef EI_DEVICE_TEMPLATE_H_ /* TODO-RENAME Update define */
#define EI_DEVICE_TEMPLATE_H_

#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"

/** Sensors */
typedef enum
{
    MICROPHONE      = 0,
}used_sensors_t;

typedef enum
{
    eiBrickMLStateIdle          = eiStateIdle,
    eiBrickMLStateErasingFlash  = eiStateErasingFlash,
    eiBrickMLStateSampling      = eiStateSampling,
    eiBrickMLStateUploading     = eiStateUploading,
    eiBrickMLStateFinished      = eiStateFinished,
    eiBrickMLStateCombi_1       = eiStateFinished + 1,
    eiBrickMLStateCombi_2       = eiStateFinished + 2,
    eiBrickMLStateCombi_3       = eiStateFinished + 3,
    eiBrickMLStateCombi_4       = eiStateFinished + 4,
} EiBrickMlState;

/** Number of sensors used */
#define EI_DEVICE_N_SENSORS            1

/** Baud rates */

class EiBrickml : public EiDeviceInfo {
private:
    EiBrickml() = delete;
    ei_device_sensor_t sensors[EI_DEVICE_N_SENSORS];
    EiBrickMlState state;
    EiDeviceMemory *data_flash;
    EiDeviceMemory *sd_card;

    bool is_sampling;
    void (*sample_read_callback)(void);
    void (*sample_multi_read_callback)(uint8_t);
    bool sd_storage;

public:
    EiBrickml(EiDeviceMemory* code_flash, EiDeviceMemory* data_flash_to_set, EiDeviceMemory *sd_card_to_set);
    ~EiBrickml();
    void init_device_id(void);
    void load_config(void) override;
    bool save_config(void) override;
    void clear_config(void);
    uint32_t get_data_output_baudrate(void) override;

    bool start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms) override;
    bool stop_sample_thread(void) override;
    void sample_thread(void);

    void set_state(EiBrickMlState state);
    EiBrickMlState get_state(void);

    bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size) override;

    bool test_flash(void);

    void set_default_data_output_baudrate(void) override;
    void set_max_data_output_baudrate(void) override;

    bool is_max_baudrate(void);
#if MULTI_FREQ_ENABLED == 1
    bool start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned);
#endif
    void set_sd_storage(bool to_set){sd_storage = to_set;};
    bool is_sd_in_use(void) {return (sd_storage);};
    bool is_sd_present(void);
    EiDeviceMemory* get_device_memory_in_use(void);
    bool init_sd(void);
};

#endif /* EI_DEVICE_TEMPLATE_H_ */
