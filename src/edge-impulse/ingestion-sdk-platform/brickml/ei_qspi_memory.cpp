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

#include <brickml/ei_qspi_memory.h>
#include <string.h>

EiQspiMemory::EiQspiMemory():
    EiDeviceMemory(0, 90, QSPI_MEMORY_SIZE, SECTOR_SIZE)
{
    used_blocks = 0;    // config is done in data flash (for now)
    //flash_handler_init();
    residual_to_write = 0;
    memset(residual_array, 0, sizeof(residual_array));
    last_offset = 0;
    bytes_written = 0;
}

uint32_t EiQspiMemory::read_data(uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    return qspi_read(data, num_bytes, address);
}

uint32_t EiQspiMemory::write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    uint32_t written = 0;
    const uint8_t* p_write;
    uint32_t to_write = num_bytes;

    p_write = data;

    if (residual_to_write != 0) /* still some residual */
    {
        if ((to_write + residual_to_write) < PAGE_WRITE_SIZE)   /* even whit this write, we don't read the minimum to write */
        {
            for (uint16_t i = 0; i < to_write; i ++)
            {
                residual_array[residual_to_write + i] = data[i];  /* complete residual array and write it*/
            }
            residual_to_write += (uint8_t)to_write;

            last_offset+=to_write;

            return num_bytes;   /* and we finish here...*/
        }
        else    /* ok we can write residual plus something new */
        {
            for (uint16_t i = residual_to_write; i < PAGE_WRITE_SIZE; i ++)
            {
                residual_array[i] = data[i - residual_to_write];  /* complete residual array and write it*/
            }

            //written = flash_handler_write((t_flash_memory)flash_type, (base_address + bytes_written), residual_array, PAGE_WRITE_SIZE);
            written = qspi_write(residual_array, PAGE_WRITE_SIZE, (bytes_written));
            bytes_written += written;
            p_write = &data[(written - residual_to_write)]; /* let's move it */

            to_write -= (PAGE_WRITE_SIZE - residual_to_write);  /* subtract the amount written - BUG !*/
            num_bytes -= (PAGE_WRITE_SIZE - residual_to_write);

            residual_to_write = 0;
            memset(residual_array, 0xFF, sizeof(residual_array));   /* fill with 0xFF */

            last_offset += written;   /* update it */
        }

    }

    //to_write = flash_handler_get_blocks_number(to_write, PAGE_WRITE_SIZE)*PAGE_WRITE_SIZE;
    to_write = (to_write/PAGE_WRITE_SIZE)*PAGE_WRITE_SIZE;

    if (to_write != 0)  /* if any, write them */
    {
        /* the handler expect the number of bytes */
        //written = flash_handler_write((t_flash_memory)flash_type, (base_address + bytes_written), p_write, to_write);
        written = qspi_write(p_write, to_write, (bytes_written));
        bytes_written += written;
        last_offset += written;  // what if not updated ?
    }

    if (to_write != num_bytes)
    {
        residual_to_write = static_cast<uint8_t>(num_bytes - to_write);   /* calc the residual bytes */
        for (uint8_t i = 0; i < residual_to_write; i ++)
        {
            residual_array[i] = p_write[to_write + i];
        }
    }

    return num_bytes;
}

uint32_t EiQspiMemory::erase_data(uint32_t address, uint32_t num_bytes)
{
    last_offset = 0;    // ??
    residual_to_write = 0;
    memset(residual_array, 0xFF, sizeof(residual_array));   /* fill with 0xFF */

    bytes_written = 0;

    return qspi_erase(address, num_bytes);
}

/**
 *
 */
uint32_t EiQspiMemory::flush_data(void)
{
    uint32_t written = 0;

    if (residual_to_write > 0)
    {
        for (uint16_t i = residual_to_write; i < PAGE_WRITE_SIZE; i++ ){
            residual_array[i] = 0xFF;   // fill
        }

        //written = flash_handler_write((t_flash_memory)flash_type, (base_address + bytes_written), residual_array, PAGE_WRITE_SIZE);
        written = qspi_write(residual_array, (PAGE_WRITE_SIZE), bytes_written);
        last_offset += written;
        bytes_written += written;

        residual_to_write = 0;
        memset(residual_array, 0xFF, sizeof(residual_array));   /* fill with 0xFF */
    }

    return 0;

}
