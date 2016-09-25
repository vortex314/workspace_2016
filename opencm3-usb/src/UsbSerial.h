/*
 * UsbSerial.h
 *
 *  Created on: 2-sep.-2016
 *      Author: lieven2
 */

#ifndef USBSERIAL_H_
#define USBSERIAL_H_

#include <CircBuf.h>
#include <Actor.h>
#include <BufferedByteStream.h>


class UsbSerial : public Actor,public BufferedByteStream {

public:
	uint32_t _rxd_overflow;
	bool _rxd_event_send;
	UsbSerial();
	virtual ~UsbSerial();
	void init();
	void loop();
	Erc open();
	Erc close();
	void flush();
};

extern UsbSerial usbSerial;

#endif /* USBSERIAL_H_ */
