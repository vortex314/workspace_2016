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

#define MAX_PACKET_SIZE 64

UsbSerial usb;
//__ALIGN_BEGIN uint8_t buffer2[MAX_PACKET_SIZE] __ALIGN_END ;
__ALIGN_BEGIN uint8_t buffer[MAX_PACKET_SIZE] __ALIGN_END;

void usbRxdCallback(uint8_t* data, uint32_t length) {
	Bytes bytes(data, length);
	Cbor cbor(300);
	cbor.addKeyValue(H("data"), bytes);
	eb.publish(H("usb.rxd"), cbor);
	return;
}

void usbTxdCallback(void) {
	usb.usbTxd();
}

UsbSerial::UsbSerial() :
		Actor("UsbSerial"), BufferedByteStream(512) {
	_rxd_overflow = 0;
	_rxd_event_send = false;
}

UsbSerial::~UsbSerial() {
	close();
}

void UsbSerial::setup() {
	usb_init();
	open();
	timeout(100);	// wait 100 msec befor first event
	eb.subscribe(this);
}

void UsbSerial::loop() {
	usb_poll(); // not needed
}

Erc UsbSerial::open() {
	usb_on_rxd(usbRxdCallback);
	usb_on_txd(usbTxdCallback);
	return E_OK;
}

Erc UsbSerial::close() {
	usb_on_rxd(0);
	usb_on_txd(0);
	return E_OK;
}

void UsbSerial::usbTxd() {
	if (hasToSend()) {
		int i = 0;
		while ((i < sizeof(buffer)) && hasToSend()) {
			buffer[i++] = toSend();
		}
		if (i) {
			usb_txd(buffer, i);
		}
	}
}

void UsbSerial::onEvent(Cbor& cbor){
//	usbTxd();
}

void UsbSerial::flush() {
	// will be handled async by usbTxd
	usbTxd();
}
