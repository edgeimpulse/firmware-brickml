/* sd_spi.c
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

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* FreeRTOS includes. */
#include "FreeRTOS.h"
//#include "FreeRTOSFATConfig.h" // for DBG_PRINTF
//
#include "sd_card.h"
#include "sd_spi.h"
#include "spi.h"
#include "bsp_pin_cfg.h"

#define TRACE_PRINTF printf  // task_printf

void sd_spi_go_high_frequency(sd_card_t *pSD) {

}
void sd_spi_go_low_frequency(sd_card_t *pSD) {

}

static void sd_spi_lock(sd_card_t *pSD) {
    configASSERT(pSD->spi->mutex);
    xSemaphoreTake(pSD->spi->mutex, portMAX_DELAY);
    configASSERT(0 == pSD->spi->owner);    
    pSD->spi->owner = xTaskGetCurrentTaskHandle();
}
static void sd_spi_unlock(sd_card_t *pSD) {
    configASSERT(xTaskGetCurrentTaskHandle() == pSD->spi->owner);
    pSD->spi->owner = 0;
    xSemaphoreGive(pSD->spi->mutex);
}

// Would do nothing if pSD->ss_gpio were set to GPIO_FUNC_SPI.
static void sd_spi_select(sd_card_t *pSD) {
#if (BRICKML_SOM == 0)

    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(SDCARD_CS, BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();
    // A fill byte seems to be necessary, sometimes:
    uint8_t fill = SPI_FILL_CHAR;
#endif
}

static void sd_spi_deselect(sd_card_t *pSD) {
#if (BRICKML_SOM == 0)

    R_BSP_PinWrite(SDCARD_CS, BSP_IO_LEVEL_LOW);
    /*
    MMC/SDC enables/disables the DO output in synchronising to the SCLK. This
    means there is a posibility of bus conflict with MMC/SDC and another SPI
    slave that shares an SPI bus. Therefore to make MMC/SDC release the MISO
    line, the master device needs to send a byte after the CS signal is
    deasserted.
    */
    uint8_t fill = SPI_FILL_CHAR;
#endif
}
/* Some SD cards want to be deselected between every bus transaction */
void sd_spi_deselect_pulse(sd_card_t *pSD) {
#if (BRICKML_SOM == 0)
    sd_spi_deselect(pSD);
    // tCSH Pulse duration, CS high 200 ns
    sd_spi_select(pSD);
#endif
}
void sd_spi_acquire(sd_card_t *pSD) {
#if (BRICKML_SOM == 0)
    sd_spi_lock(pSD);
    sd_spi_select(pSD);
#endif
}

void sd_spi_release(sd_card_t *pSD) {
#if (BRICKML_SOM == 0)
    sd_spi_deselect(pSD);
    sd_spi_unlock(pSD);
#endif
}

bool sd_spi_transfer(sd_card_t *pSD, const uint8_t *tx, uint8_t *rx,
                     size_t length) {
#if (BRICKML_SOM == 0)
    return spi_transfer(pSD->spi, tx, rx, length);
#else
    return false;
#endif
}

uint8_t sd_spi_write(sd_card_t *pSD, const uint8_t value) {
    u_int8_t received = SPI_FILL_CHAR;
#if (BRICKML_SOM == 0)
    configASSERT(xTaskGetCurrentTaskHandle() == pSD->spi->owner);
    bool success = spi_transfer(pSD->spi, &value, &received, 1);
    configASSERT(success);
#endif
    return received;
}

void sd_spi_send_initializing_sequence(sd_card_t * pSD) {
#if (BRICKML_SOM == 0)
    bool old_ss = false;
    // Set DI and CS high and apply 74 or more clock pulses to SCLK:
    //gpio_put(pSD->ss_gpio, 1);
    uint8_t ones[10];
    memset(ones, 0xFF, sizeof ones);
    //absolute_time_t timeout_time = make_timeout_time_ms(1);
    int timeout_time = 0;
    do {
        sd_spi_transfer(pSD, ones, NULL, sizeof ones);
    } while(0); //while (0 < absolute_time_diff_us(get_absolute_time(), timeout_time));
#endif
}

/* [] END OF FILE */
