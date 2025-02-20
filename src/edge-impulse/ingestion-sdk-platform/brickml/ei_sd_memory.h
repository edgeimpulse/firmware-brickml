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

