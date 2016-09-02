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



class UsbSerial : public Actor {
public:
	UsbSerial();
	virtual ~UsbSerial();
	void init();
	void poll();
	bool hasData();
	bool hasSpace();

};

#endif /* USBSERIAL_H_ */
