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

#ifndef PERIPHERAL_SD_SD_HANDLER_H_
#define PERIPHERAL_SD_SD_HANDLER_H_

#include "bsp_api.h"

FSP_CPP_HEADER

extern bool sd_handler_init(void);
extern bool sd_handler_deinit(void);
extern bool sd_handler_open_file(const char* filename, uint8_t mode);
extern uint32_t sd_handler_read_file(uint8_t * buffer, uint32_t max_size, uint32_t* read_size);
extern int32_t sd_handler_write_file(const uint8_t * buffer, uint32_t bytes);
extern bool sd_handler_close_file(void);
extern int sdmmc_exist_check(void);
extern uint16_t sd_handler_get_file_numer(const char *pcDirectoryToScan, const char* to_match);
extern bool sd_handler_check_is_available(const char *pcDirectoryToScan, const char* to_match);
extern bool sd_handler_is_present(void);
extern uint32_t sd_handler_get_sectors(void);

FSP_CPP_FOOTER

#endif /* PERIPHERAL_SD_SD_HANDLER_H_ */
