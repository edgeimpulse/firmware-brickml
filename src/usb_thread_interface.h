/*
 * usb_thread_interface.h
 *
 *  Created on: Jan 4, 2023
 *      Author: parallels
 */

#ifndef USB_THREAD_INTERFACE_H_
#define USB_THREAD_INTERFACE_H_

#include "stdint.h"

extern char ei_get_serial_byte(uint8_t is_inference_running);
extern uint32_t ei_get_serial_buffer(uint8_t* pbuffer, uint32_t max_len);
extern void usb_clean_buffer(void);


#endif /* USB_THREAD_INTERFACE_H_ */
