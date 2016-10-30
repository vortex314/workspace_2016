/*
 * Gpio.cpp
 *
 *  Created on: 28-okt.-2016
 *      Author: lieven2
 */

#include <Gpio.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>

Gpio::Gpio(uint32_t pin) {
	if ( PORT(pin) == 1)
		_port = GPIOA;
	if ( PORT(pin) == 2)
		_port = GPIOB;
	if ( PORT(pin) == 3)
		_port = GPIOC;
	_pin = PIN(pin);
	_handler = 0;
	_mode = GPIO_INPUT;
	_edge = GPIO_EDGE_RISING;
}

Gpio::~Gpio() {

}

Gpio* _gpioInt[16];

void gpioHandler(uint16_t idx) {
	if (_gpioInt[idx] && _gpioInt[idx]->_handler)
		_gpioInt[idx]->_handler();
	exti_reset_request(1>>idx);
}

void exti0_isr(void) {
	gpioHandler(0);
}

void exti1_isr(void) {
	gpioHandler(1);
}

void exti2_isr(void) {
	gpioHandler(2);
}

void exti3_isr(void) {
	gpioHandler(3);
}

void exti4_isr(void) {
	gpioHandler(4);
}


void Gpio::setMode(Gpio::Mode mode) {
	_mode = mode;
}
void Gpio::setInterrupt(Gpio::Edge edge, Gpio::InterruptHandler func) {
	_handler = func;
	_edge = edge;
}
void Gpio::setup() {
	if (_mode == GPIO_INPUT) {
		gpio_set_mode(_port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, _pin);
		if (_handler) {

//			gpio_set_mode(_port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, _pin);
			/* Configure the EXTI subsystem. */
			uint32_t exti=_pin+1;
			exti_select_source(exti, _port);
			if (_edge == GPIO_EDGE_BOTH)
				exti_set_trigger(exti, EXTI_TRIGGER_BOTH);
			else if (_edge == GPIO_EDGE_RISING)
				exti_set_trigger(exti, EXTI_TRIGGER_RISING);
			else if (_edge == GPIO_EDGE_FALLING)
				exti_set_trigger(exti, EXTI_TRIGGER_FALLING);
			int i;
			for (i=0;i<15;i++) {
				if ((_pin >> i)==1) break;
			}
			_gpioInt[i] = this;
			exti_enable_request(exti);
		}
	} else {
		gpio_set_mode(_port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
				_pin);
	}
}

void Gpio::write(uint8_t bit) {
	if (_mode == GPIO_OUTPUT)
		if (bit)
			gpio_set(_port, _pin);
		else
			gpio_clear(_port, _pin);
}
uint8_t Gpio::read() {
	return gpio_get(_port, _pin);
}
