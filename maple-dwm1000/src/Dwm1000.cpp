/*
 * Dwm1000.cpp
 *
 *  Created on: 27-okt.-2016
 *      Author: lieven2
 */

#include "Dwm1000.h"
#include <Sys.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f1/gpio.h>

Dwm1000* _gDwm1000;

Dwm1000::Dwm1000() :
		Actor("DWM1000") {
	_spi = 0;
	_pinIrq = 0;
	_pinReset = 0;
	_connector = 0;
	_gDwm1000 = this;
}

Dwm1000::~Dwm1000() {

}

void Dwm1000::onIrq() {

}

void Dwm1000::setConnector(uint32_t idx) {
	_connector = new Connector(idx);
	_spi = _connector->getSpi();
	_pinReset = _connector->getGpio(Connector::PIN_TXD);
	_pinReset->setMode(Gpio::GPIO_OUTPUT);
	_pinIrq = _connector->getGpio(Connector::PIN_RXD);
	_pinIrq->setMode(Gpio::GPIO_INPUT);
	_pinIrq->setInterrupt(Gpio::GPIO_EDGE_FALLING, []() {
		if ( _gDwm1000 ) _gDwm1000->onIrq();
	});
	timeout(1000);
}

#define PORT_RESET GPIOA
#define PIN_RESET GPIO2

#define PORT_IRQ GPIOA
#define PIN_IRQ GPIO3

uint16_t exti_line_state;

void Dwm1000::setup() {
	_spi->setup();
	_pinReset->setup();
	_pinIrq->setup();
}

void delay(uint32_t msec) {
	uint64_t endTime = Sys::millis() + msec;
	while (Sys::millis() < endTime)
		;
}

void Dwm1000::reset() { // 20msec
	gpio_set(PORT_RESET, PIN_RESET);
	delay(10);
	gpio_clear(PORT_RESET, PIN_RESET);
	delay(10);
	gpio_set(PORT_RESET, PIN_RESET);
}

uint8_t init[] = { };
Bytes input(100);
Bytes output(0);

void Dwm1000::onEvent(Cbor& cbor) {
	uint16_t event;
	cbor.getKeyValue(0, event);
	PT_BEGIN()
	while (true) {
		PT_YIELD_UNTIL(event == H("timeout"));
		_spi->xfer(input, output.map(init, sizeof(init)));
		timeout(1000);
	}
PT_END()
}
