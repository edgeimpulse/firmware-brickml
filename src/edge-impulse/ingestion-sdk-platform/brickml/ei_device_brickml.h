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
