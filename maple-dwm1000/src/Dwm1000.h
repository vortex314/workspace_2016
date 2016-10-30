/*
 * Dwm1000.h
 *
 *  Created on: 27-okt.-2016
 *      Author: lieven2
 */

#ifndef DWM1000_H_
#define DWM1000_H_

#include <Connector.h>
#include <Spi.h>
#include <EventBus.h>

class Dwm1000 : public Actor{
	Connector* _connector;
	Spi* _spi;
	Gpio* _pinReset;
	Gpio* _pinIrq;
public:
	Dwm1000();
	virtual ~Dwm1000();
	void setConnector(uint32_t idx);
	void onIrq();
	void setup();
	void reset();
	void onEvent(Cbor& cbor);
};

#endif /* DWM1000_H_ */
