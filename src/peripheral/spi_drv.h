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

#ifndef SPI_DRV_H_
#define SPI_DRV_H_

#include "bsp_api.h"

FSP_CPP_HEADER

int spi_write(uint8_t *data, uint32_t bytes);
int spi_read(uint8_t *buf, uint32_t size);
int spi_write_read(const uint8_t *out, uint8_t *in, uint32_t size);
int spi_init(void);

FSP_CPP_FOOTER

#endif /* SPI_DRV_H_ */
