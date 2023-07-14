/*
 * qspi.h
 *
 *  Created on: Jan 4, 2023
 *      Author: parallels
 */

#ifndef PERIPHERAL_QSPI_H_
#define PERIPHERAL_QSPI_H_

#include <bsp_api.h>

FSP_CPP_HEADER

#define SECTOR_SIZE                     (4096u)

/* QSPI flash page Size */
#define PAGE_WRITE_SIZE                 (256U)

#define QSPI_MEMORY_SIZE                (0x800000)

extern uint32_t qspi_init(void);
extern uint32_t qspi_deinit(void);
extern uint32_t qspi_read(uint8_t* data, uint32_t bytes, uint32_t address);
extern uint32_t qspi_write(const uint8_t* data, uint32_t bytes, uint32_t address);
extern uint32_t qspi_erase(uint32_t address, uint32_t bytes);
extern uint32_t qspi_test(void);

FSP_CPP_FOOTER

#endif /* PERIPHERAL_QSPI_H_ */
