/*
 * DWM1000_Tag.h
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#ifndef DWM1000_Tag_H_
#define DWM1000_Tag_H_

#include <Connector.h>
#include <EventBus.h>

class DWM1000_Tag: public Actor {
	uint32_t _count;
	static uint32_t _status_reg;
	Connector* _connector;
	Spi* _spi;
	Gpio* _pinReset;
	Gpio* _pinIrq;
public:
	DWM1000_Tag();
	virtual ~DWM1000_Tag();
	void mode(uint32_t m);
	void init();
	void resetChip();
	void initSpi();
	static bool interrupt_detected;
	bool isInterruptDetected();
	void clearInterrupt();
	void enableIsr();
	static void onIrq();
		void onEvent(Cbor& cbor);
		void setup();
};

#endif /* DWM1000_Tag_H_ */
