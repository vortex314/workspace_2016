/*
 * usbserial.h
 *
 *  Created on: 2-sep.-2016
 *      Author: lieven2
 */

#ifndef USB_SERIAL_H_
#define USB_SERIAL_H_

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef void (*usb_callback)(uint8_t* data,uint32_t length);
extern void usb_init();
extern void usb_poll();
extern void usb_on_rxd(usb_callback f);
extern bool usb_txd(uint8_t* data,uint32_t length);
extern void doUsb();

#ifdef __cplusplus
}
#endif

#endif /* USB_SERIAL_H_ */
