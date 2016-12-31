/*
 * Led.cpp
 *
 *  Created on: 26-sep.-2016
 *      Author: lieven2
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <Led.h>
#include <EventBus.h>

Led::Led() :
		Actor("Led") {
	_interval = 100;
	_isOn = true;
}

Led::~Led() {
}

void Led::setup() {
	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
	GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
	gpio_set(LED_PORT, LED_PIN);
	timeout(100);
}

void Led::onEvent(Cbor& cbor) {
	if (timeout()) {
		if (_isOn) {
			_isOn = false;
			gpio_set(LED_PORT, LED_PIN);
		} else {
			_isOn = true;
			gpio_clear(LED_PORT, LED_PIN);
		}
		timeout(_interval);
	}
}

void Led::blinkFast() {
	_interval = 100;
}

void Led::blinkSlow() {
	_interval = 500;
}

