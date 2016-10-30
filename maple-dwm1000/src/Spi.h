/*
 * Spi.h
 *
 *  Created on: 26-okt.-2016
 *      Author: lieven2
 */

#ifndef SPI_H_
#define SPI_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>

#include <Bytes.h>
#include <Gpio.h>

class Spi {

public:
	typedef void (*InterruptHandler)(void);
	typedef enum {
		POL0_PHA0, POL1_PHA0, POL0_PHA1, POL1_PHA1
	} Mode;

	Spi(uint32_t spi);
	virtual ~Spi();
	void setClock(uint32_t clock);
	void setMode(Mode mode);
	void setInterrupt(InterruptHandler hdlr);
	void setup();

	void xfer(Bytes& in, Bytes& out);
	uint8_t xfer(uint8_t b);
private :
	uint32_t _SPI;
	Mode _mode;
	uint32_t _clock;
	Gpio* _pinCS;
};

#endif /* SPI_H_ */
