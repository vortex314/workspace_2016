/*
 * Connector.h
 *
 *  Created on: 28-okt.-2016
 *      Author: lieven2
 */

#ifndef CONNECTOR_H_
#define CONNECTOR_H_

#include <Usart.h>
#include <Spi.h>
#include <Gpio.h>
#include <I2C.h>

class Connector {
	uint32_t _idx;
	Usart* _usart;
	Spi* _spi;
	I2C* i2c;
	Gpio* _pinTxd;
	Gpio* _pinRxd;
	Gpio* _pinMosi;
	Gpio* _pinMiso;
	Gpio* _pinSck;
	Gpio* _pinNss;
	Gpio* _pinSda;
	Gpio* _pinScl;

public:
	typedef enum {
		PIN_TXD,
		PIN_RXD,
		PIN_SDA,
		PIN_SCL,
		PIN_MOSI,
		PIN_MISO,
		PIN_SCK,
		PIN_NSS
	} Pin;
	Connector(uint32_t idx);
	virtual ~Connector();

	Usart* getUsart();
	Spi* getSpi();
	I2C* getI2C();
	Gpio* getGpio(Pin pin);
};

#endif /* CONNECTOR_H_ */
