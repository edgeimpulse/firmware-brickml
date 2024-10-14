/* spi.h
Copyright 2021 Carl John Kugler III

Licensed under the Apache License, Version 2.0 (the License); you may not use 
this file except in compliance with the License. You may obtain a copy of the 
License at

   http://www.apache.org/licenses/LICENSE-2.0 
Unless required by applicable law or agreed to in writing, software distributed 
under the License is distributed on an AS IS BASIS, WITHOUT WARRANTIES OR 
CONDITIONS OF ANY KIND, either express or implied. See the License for the 
specific language governing permissions and limitations under the License.
*/

#ifndef _SPI_H_
#define _SPI_H_

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
//
#include <stdbool.h>
//

#define SPI_FILL_CHAR (0xFF)

// "Class" representing SPIs
typedef struct {
    // SPI HW
    //spi_inst_t *hw_inst;
    uint32_t miso_gpio;  // SPI MISO GPIO number (not pin number)
    uint32_t mosi_gpio;
    uint32_t sck_gpio;
    uint32_t baud_rate;
    // State variables:
    bool initialized;         // Assigned dynamically
    TaskHandle_t owner;       // Assigned dynamically
    SemaphoreHandle_t mutex;  // Assigned dynamically
} spi_t;

#ifdef __cplusplus
extern "C" {
#endif

    bool spi_transfer(spi_t *pSPI, const uint8_t *tx, uint8_t *rx, size_t length);

#ifdef __cplusplus
}
#endif

#endif
/* [] END OF FILE */
