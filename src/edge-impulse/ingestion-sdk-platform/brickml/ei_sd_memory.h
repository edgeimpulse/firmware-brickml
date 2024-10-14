/*
 * Copyright (c) 2023 EdgeImpulse Inc.
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

#ifndef EDGE_IMPULSE_INGESTION_SDK_PLATFORM_BRICKML_EI_SD_MEMORY_H_
#define EDGE_IMPULSE_INGESTION_SDK_PLATFORM_BRICKML_EI_SD_MEMORY_H_

#include "firmware-sdk/ei_device_memory.h"
#include <string>


class EiSDMemory : public EiDeviceMemory {
private:
    bool        sd_card_inserted;
    const char  drive[6] = "/sd0/";
    std::string latest_file;
    bool        file_is_open;
    uint16_t    total_file;

protected:
    uint32_t read_data(uint8_t *data, uint32_t address, uint32_t num_bytes) override;
    uint32_t write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes) override;
    uint32_t erase_data(uint32_t address, uint32_t num_bytes) override;
public:    
    EiSDMemory();

    bool init(void);

    bool open_sample_file(char*filename, bool write);
    bool close_sample_file(void);
    void rewind(void);
    bool is_inserted(void){return sd_card_inserted;};
    bool read_content(void);
    uint32_t test(void);
    bool setup_sampling(const char* sensor_name, const char* lable_name) override;
    void finalize_samplig(void) override;
    bool open_latest_file(bool write);
    uint16_t get_file_number(char* pattern);
};

#endif /* EDGE_IMPULSE_INGESTION_SDK_PLATFORM_BRICKML_EI_SD_MEMORY_H_ */

