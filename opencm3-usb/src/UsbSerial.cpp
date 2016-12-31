/*
 * UsbSerial.cpp
 *
 *  Created on: 2-sep.-2016
 *      Author: lieven2
 */

#include <usb_serial.h>
//#include <cdcacm.h>
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
}

__ALIGN_BEGIN uint8_t buffer2[64] __ALIGN_END ;

void UsbSerial::onEvent(Cbor& cbor) { // called every 10 msec

	usb_poll(); // not needed
	timeout(10);
	/*
	if (hasData() && !_rxd_event_send) {
		eb.publish(H("usb.rxd"));
		_rxd_event_send = true;
	}*/
	if (hasToSend()) {
		int i = 0;
		while (i < sizeof(buffer2) && hasToSend()) {
			buffer2[i++] = toSend();
		}
		if (i) {
			usb_txd(buffer2, i);
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

#define MAX_PACKET_SIZE 64

__ALIGN_BEGIN uint8_t buffer[MAX_PACKET_SIZE] __ALIGN_END ;


void UsbSerial::flush() {
	if (hasToSend()) {
			int i = 0;
			while (i < sizeof(buffer) && hasToSend()) {
				buffer[i++] = toSend();
			}
			if (i) {
				usb_txd(buffer, i);
			}
		}
}
