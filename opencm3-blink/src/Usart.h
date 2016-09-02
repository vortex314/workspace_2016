/*
 * Usart.h
 *
 *  Created on: Aug 14, 2016
 *      Author: lieven2
 */

#ifndef USART_H_
#define USART_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <stdio.h>
#include <errno.h>
#include <Actor.h>
#define IRAM
#include <CircBuf.h>
#include <Bytes.h>

#define UART_BUFFER_SIZE 512

int _write(int file, char *ptr, int len);

class Usart :public Actor  {
public:
	CircBuf _txd;
	CircBuf _rxd;


	Usart();
	virtual ~Usart();

	static void usart1_isr(void);
	void init();
	void loop();

	 Erc write(Bytes& data);
	 Erc write(uint8_t b);
	 bool hasData();
	 bool hasSpace();
	 uint8_t read();
	 void receive(uint8_t b);
	 Erc setBaudrate(uint32_t baudrate);
	 uint32_t getBaudrate();
	 Erc setMode(const char* str);
	 void getMode(char* str);

};

extern Usart usart1;

#endif /* USART_H_ */
