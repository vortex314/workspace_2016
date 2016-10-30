/*
 * Led.h
 *
 *  Created on: 26-sep.-2016
 *      Author: lieven2
 */

#ifndef LED_H_
#define LED_H_

#include <Actor.h>

class Led: public Actor {
	uint32_t _interval;
	bool _isOn;
public:
	Led() ;
	~Led();
	void onEvent(Cbor& cbor) ;
	void setup() ;
	void loop();
	void blinkFast();
	void blinkSlow() ;
};

#define MAPLE_MINI

#ifdef MAPLE_MINI
#define LED_PORT GPIOB
#define LED_PIN	 GPIO1
#endif
#ifdef STM32F103_MINIMAL
#define LED_PORT GPIOC
#define LED_PIN	 GPIO13
#endif

#endif /* LED_H_ */
