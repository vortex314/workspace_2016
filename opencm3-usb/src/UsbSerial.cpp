/*
 * UsbSerial.cpp
 *
 *  Created on: 2-sep.-2016
 *      Author: lieven2
 */

#include <usb_serial.h>
#include <UsbSerial.h>
#include <EventBus.h>

UsbSerial usb;

void usbCallback(uint8_t* data, uint32_t length) {
	Bytes bytes(data,length);
	Cbor cbor(200);
	cbor.addKeyValue(H("data"),bytes);
	eb.publish(H("usb.rxd"),cbor);
	return;
}

UsbSerial::UsbSerial() :
		Actor("UsbSerial"), BufferedByteStream(256) {
	_rxd_overflow = 0;
	_rxd_event_send = false;
}

UsbSerial::~UsbSerial() {

}

void UsbSerial::setup() {
	usb_init();
	open();
	eb.subscribe(0, [](Cbor& cbor) { // all events
		usb.loop();
	});
}



void UsbSerial::loop() {
	usb_poll(); // not needed
	/*
	if (hasData() && !_rxd_event_send) {
		eb.publish(RXD);
		_rxd_event_send = true;
	}*/
	if (hasToSend()) {
		uint8_t buffer[64];
		int i = 0;
		while (i < sizeof(buffer) && hasToSend()) {
			buffer[i++] = toSend();
		}
		if (i) {
			usb_txd(buffer, i);
		}
	}
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
	if (hasToSend()) {
			uint8_t buffer[64];
			int i = 0;
			while (i < sizeof(buffer) && hasToSend()) {
				buffer[i++] = toSend();
			}
			if (i) {
				usb_txd(buffer, i);
			}
		}
}
