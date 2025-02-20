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
#include "sd_handler.h"
#include "FreeRTOS.h" /* Must come first. */
//
#include <stdio.h>
//

//
#include "ff_headers.h"
#include "portable/ra6m5/ff_sddisk.h"
#include "ff_stdio.h"
#include "ff_utils.h"
//
#include "hw_config.h"
#include "peripheral/spi_drv.h"

static void stop(void);

static FF_Disk_t *pxDisk;
static FF_FILE *sample_file;

static bool sd_init_done = false;

/**
 * @brief Mount sd card
 * @return
 */
bool sd_handler_init(void)
{
    if (sd_init_done == false) {
        spi_init();
        sd_init_done = true;
        pxDisk = FF_SDDiskInit("sd0");
        if (pxDisk == NULL) {
            return false;
        }

        FF_Error_t xError = FF_SDDiskMount(pxDisk);
        if (FF_isERR(xError) != pdFALSE) {
            FF_PRINTF("FF_SDDiskMount: %s\n",
                      (const char *)FF_GetErrMessage(xError));
            stop();
        }
        FF_FS_Add("/sd0", pxDisk);
    }

    return (pxDisk != NULL);
}

/**
 *
 * @return
 */
bool sd_handler_is_present(void)
{
    // if not initialized, will initialize
    return sd_handler_init();
}

/**
 *
 * @return
 */
uint32_t sd_handler_get_sectors(void)
{
    sd_card_t * pSD = sd_get_by_name("sd0");

    if (pSD != NULL) {
        if (true == sd_card_detect(pSD)) {
            return (uint32_t)sd_sectors(pSD);
        }
    }

    return 0;
}

/**
 * @brief Unmount sd card
 * @return
 */
bool sd_handler_deinit(void)
{
    FF_FS_Remove("/sd0");
    FF_Unmount(pxDisk);
    FF_SDDiskDelete(pxDisk);    // ?

    return true;
}

/**
 * @brief Open filename
 * @param filename
 * @param mode
 * @return
 */
bool sd_handler_open_file(const char* filename, uint8_t mode)
{
    bool retval = false;
    const char* mode_str[] = {"r", "w", "a"};

    if ((mode < 3) && (filename != NULL) && (sample_file == NULL)) {
        sample_file = ff_fopen(filename, mode_str[mode]);

        if (sample_file != NULL) {
            retval = true;
        }
    }

    return retval;
}

/**
 * @brief Read max_size bytes and put into buffer
 * 
 * @param buffer 
 * @param max_size 
 * @param read_size 
 * @return uint32_t 
 */
uint32_t sd_handler_read_file(uint8_t * buffer, uint32_t max_size, uint32_t* read_size)
{
    uint32_t read_bytes = 0;

    if (sample_file == NULL) {
        return 0;
    }

    do {
        *read_size = ff_fread(&buffer[read_bytes], 1, max_size, sample_file);

        read_bytes += *read_size;
        max_size -= *read_size;

    } while((ff_feof(sample_file) == 0) && max_size);   // till EOF and i can store
    
    
    return read_bytes;
}

/**
 * @brief Write buffer to open fal
 * @param buffer
 * @param bytes
 * @return
 */
int32_t sd_handler_write_file(const uint8_t * buffer, uint32_t bytes)
{
    int32_t retval = -1;
    if (sample_file != NULL) {

        retval = ff_fwrite(buffer, 1, bytes, sample_file);

        FF_SDDiskFlush(pxDisk);
    }
    
    return retval;
}

/**
 * @brief close the cirrent open file
 * @return
 */
bool sd_handler_close_file(void)
{
    FF_SDDiskFlush(pxDisk);
    if (ff_fclose(sample_file) == 0) {
        sample_file = NULL;
        return true;
    }

    return false;
}

/**
 * @brief 
 * 
 * @return uint32_t 
 */
uint32_t sd_handler_test_write(void)
{
    FF_FILE *pxFile = ff_fopen("/sd0/filename.txt", "a");

    if (!pxFile) {
        FF_PRINTF("ff_fopen failed: %s (%d)\n", strerror(stdioGET_ERRNO()),
                  stdioGET_ERRNO());
        stop();
    }
    //if (ff_fprintf(pxFile, "Hello, world!\n") < 0) {
    //    FF_PRINTF("ff_fprintf failed: %s (%d)\n", strerror(stdioGET_ERRNO()),
    //              stdioGET_ERRNO());
    //    stop();
    //}

    if (-1 == ff_fclose(pxFile)) {
        FF_PRINTF("ff_fclose failed: %s (%d)\n", strerror(stdioGET_ERRNO()),
                  stdioGET_ERRNO());
        stop();
    }

    return 0;
}

static void stop(void) 
{
    FF_SDDiskFlush(pxDisk);
    //vTaskSuspend(NULL);
    //__breakpoint();
}

/**
 * @brief 
 * 
 * @return uint16_t 
 */
uint16_t sd_handler_get_file_numer(const char *pcDirectoryToScan, const char* to_match)
{
    uint16_t count = 0;
    FF_FindData_t *pxFindStruct;

    /* FF_FindData_t can be large, so it is best to allocate the structure
    dynamically, rather than declare it as a stack variable. */
    pxFindStruct = ( FF_FindData_t * ) pvPortMalloc( sizeof( FF_FindData_t ) );

    /* FF_FindData_t must be cleared to 0. */
    memset( pxFindStruct, 0x00, sizeof( FF_FindData_t ) );

    /* The first parameter to ff_findfist() is the directory being searched.  Do
    not add wildcards to the end of the directory name. */
    if (ff_findfirst(pcDirectoryToScan, pxFindStruct) == 0)
    {
        do
        {
            /* Point pcAttrib to a string that describes the file. */
            if( ( pxFindStruct->ucAttributes & FF_FAT_ATTR_DIR ) == 0 )
            {
                // it's a file
                if (to_match == NULL) {
                    count++;
                }
                else  if ((strlen(pxFindStruct->pcFileName) != 0) && (pxFindStruct->ulFileSize != 0)) {
                    if (strstr(pxFindStruct->pcFileName, to_match) != NULL) {
                        count++;
                    }
                }
            }

        } while (ff_findnext( pxFindStruct ) == 0);
    }

    /* Free the allocated FF_FindData_t structure. */
    vPortFree (pxFindStruct);

    return count;
}

/**
 *
 * @param pcDirectoryToScan
 * @param to_match
 * @return
 */
bool sd_handler_check_is_available(const char *pcDirectoryToScan, const char* to_match)
{
    bool avialable = true;

    FF_FindData_t *pxFindStruct;

    /* FF_FindData_t can be large, so it is best to allocate the structure
    dynamically, rather than declare it as a stack variable. */
    pxFindStruct = ( FF_FindData_t * ) pvPortMalloc( sizeof( FF_FindData_t ) );

    /* FF_FindData_t must be cleared to 0. */
    memset( pxFindStruct, 0x00, sizeof( FF_FindData_t ) );

    /* The first parameter to ff_findfist() is the directory being searched.  Do
    not add wildcards to the end of the directory name. */
    if (ff_findfirst(pcDirectoryToScan, pxFindStruct) == 0)
    {
        do
        {
            /* Point pcAttrib to a string that describes the file. */
            if( ( pxFindStruct->ucAttributes & FF_FAT_ATTR_DIR ) == 0 )
            {
                if ((strlen(pxFindStruct->pcFileName) != 0) && (pxFindStruct->ulFileSize != 0)) {
                    if (strcmp(pxFindStruct->pcFileName, to_match) == 0) {
                        avialable = false;
                        break;
                    }
                }
            }

        } while (ff_findnext( pxFindStruct ) == 0);
    }

    /* Free the allocated FF_FindData_t structure. */
    vPortFree (pxFindStruct);

    return avialable;
}
