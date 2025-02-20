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

#ifndef EDGE_IMPULSE_INGESTION_SDK_PLATFORM_BRICKML_EI_QSPI_MEMORY_H_
#define EDGE_IMPULSE_INGESTION_SDK_PLATFORM_BRICKML_EI_QSPI_MEMORY_H_

#include "firmware-sdk/ei_device_memory.h"
#include "peripheral/qspi.h"

class EiQspiMemory : public EiDeviceMemory {
private:
    uint16_t residual_to_write;
    uint8_t residual_array[PAGE_WRITE_SIZE];
    uint32_t last_offset;

    uint32_t bytes_written;

protected:
    uint32_t read_data(uint8_t *data, uint32_t address, uint32_t num_bytes) override;
    uint32_t write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes) override;
    uint32_t erase_data(uint32_t address, uint32_t num_bytes) override;
public:
    uint32_t flush_data(void) override;
    EiQspiMemory();
};

#endif /* EDGE_IMPULSE_INGESTION_SDK_PLATFORM_BRICKML_EI_QSPI_MEMORY_H_ */
