/*
 * Connector.cpp
 *
 *  Created on: 28-okt.-2016
 *      Author: lieven2
 */

#include <Connector.h>

Connector::Connector(uint32_t idx) {
	_idx = idx;
	_usart = 0;
	_spi = 0;
	i2c = 0;
	_pinTxd = 0;
	_pinRxd = 0;
	_pinMosi = 0;
	_pinMiso = 0;
	_pinSck = 0;
	_pinNss = 0;
	_pinSda = 0;
	_pinScl = 0;

}

Connector::~Connector() {

}

Spi* Connector::getSpi() {
	if (_spi)
		return _spi;
	if (_pinMiso == 0 && _pinMosi == 0 && _pinSck == 0 && _pinNss == 0) {
		if (_idx == 1)
			_spi = new Spi(2);
		else if (_idx == 2)
			_spi = new Spi(1);
		return _spi;
	}
	return 0;
}

Gpio* Connector::getGpio(Pin pin) {
	if (pin == PIN_TXD) {
		if (_pinTxd)
			return _pinTxd;
		if (_usart == 0) {
			if (_idx == 1)
				_pinTxd = new Gpio(PA(9));
			else if (_idx == 2)
				_pinTxd = new Gpio(PA(2));
		}
		return _pinTxd;
	}

	if (pin == PIN_RXD) {
		if (_pinRxd)
			return _pinRxd;
		if (_usart == 0) {
			if (_idx == 1)
				_pinRxd = new Gpio(PA(10));
			else if (_idx == 2)
				_pinRxd = new Gpio(PA(3));
		}
		return _pinRxd;
	}
	return 0;
}

