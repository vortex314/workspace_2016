/*
 * Gpio.h
 *
 *  Created on: 28-okt.-2016
 *      Author: lieven2
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>

#define PA(x) (0x100+(x))
#define PB(x) (0x200+(x))
#define PC(x) (0x300+(x))
#define PORT(x) ((x & 0x300)>>8)
#define PIN(x) (x & 0xFF)

class Gpio {

public:
	typedef void (*InterruptHandler)(void);
	typedef enum {
		GPIO_OUTPUT, GPIO_INPUT
	} Mode;

	typedef enum {
		GPIO_EDGE_RISING, GPIO_EDGE_FALLING, GPIO_EDGE_BOTH
	} Edge;

	Gpio(uint32_t pin);
	virtual ~Gpio();
	void setMode(Mode mode);
	void setInterrupt(Edge edge, InterruptHandler func);
	void setup();

	void write(uint8_t bit);
	uint8_t read();

	InterruptHandler _handler;
private:
	uint32_t _port;
	uint16_t _pin;
	Mode _mode;

	Edge _edge;
};

#endif /* GPIO_H_ */
