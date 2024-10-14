/* spi.c
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

#include <stdbool.h>
//
#include "FreeRTOS.h"
//
#include "spi.h"
#include "peripheral/spi_drv.h"


// SPI Transfer: Read & Write (simultaneously) on SPI bus
//   If the data that will be received is not important, pass NULL as rx.
//   If the data that will be transmitted is not important,
//     pass NULL as tx and then the SPI_FILL_CHAR is sent out as each data
//     element.
bool spi_transfer(spi_t *pSPI, const uint8_t *tx, uint8_t *rx, size_t length)
{
    configASSERT(xTaskGetCurrentTaskHandle() == pSPI->owner);
    configASSERT(tx || rx);
    (void)pSPI;

    spi_write_read(tx, rx, (uint32_t)length);

    return true;
}

/* [] END OF FILE */
