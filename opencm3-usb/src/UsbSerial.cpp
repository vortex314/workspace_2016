/*
 * UsbSerial.cpp
 *
 *  Created on: 2-sep.-2016
 *      Author: lieven2
 */

#include <usb_serial.h>
#include <UsbSerial.h>

UsbSerial usbSerial;

void usbCallback(uint8_t* data, uint32_t length) {
	for (uint32_t i = 0; i < length; i++) {
		if (usbSerial.canReceive()) {
			usbSerial.receive(data[i]);
		} else {
			usbSerial._rxd_overflow++;
		}
	}
//	LOGF("%s", data);
}

UsbSerial::UsbSerial() :
		Actor("UsbSerial"), BufferedByteStream(256) {
	_rxd_overflow = 0;
}

UsbSerial::~UsbSerial() {

}

void UsbSerial::init() {
	usb_init();
	open();
}

void UsbSerial::loop() {
	usb_poll(); // not needed
	/*	if (hasData())
	 publish(RXD);*/
}

Erc UsbSerial::open() {

	usb_on_rxd(usbCallback);
	return E_OK;
}

Erc UsbSerial::close() {
	usb_on_rxd(0);
	return E_OK;
}

void UsbSerial::flush() {

}
