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
void EiQspiMemory::write_residual(void)
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

}
