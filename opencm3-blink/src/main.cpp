/*
 * main.c
 *
 *  Created on: 7-aug.-2016
 *      Author: 600104947
 */

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <Sys.h>


/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

// void* __dso_handle;

static void clock_setup(void) {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable clocks for GPIO port B (for GPIO_USART3_TX) and USART3. */
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_USART3);
}

static void usart_setup(void) {
	/* Setup GPIO pin GPIO_USART1_TX. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
	GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
	GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 460800);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);

}

static void gpio_setup(void) {
	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
	GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

void usart_send_string(const char *s){
	while(*s){
		usart_send_blocking(USART1,*(s++));
	}
}

#include <Actor.h>
#include <Log.h>

void usartLog(char* data,uint32_t length){
	data[length]='\n';
	data[length+1]='\0';
	usart_send_string(data);
}

#define MAPLE_MINI

#ifdef MAPLE_MINI
#define LED_PORT GPIOB
#define LED_PIN	 GPIO1
#endif
#ifdef STM32F103_MINIMAL
#define LED_PORT GPIOC
#define LED_PIN	 GPIO13
#endif

class Led : public Actor{
	uint32_t _interval;
	bool _isOn;
public:

	Led() :
			Actor("Led") {
		_interval = 100;
		_isOn = true;
	}

	~Led() {
	}

	void init() {

		/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
		gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
		gpio_set(LED_PORT,LED_PIN);

		timeout(100);
	}

	void loop() {
		if (timeout()) {
			if (_isOn) {
				_isOn = false;
				gpio_set(LED_PORT,LED_PIN);
			} else {
				_isOn = true;
				gpio_clear(LED_PORT,LED_PIN);
			}
			timeout(_interval);
		}
	}

	void blinkFast(Header h) {
		(void)h;
		_interval = 100;
	}

	void blinkSlow(Header h) {
		(void)h;
		_interval = 500;
	}
};

Led led;
#include <Usart.h>

static void systick_setup(void) {
	/* 72MHz / 8 => 9000000 counts per second. */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

	/* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(8999);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}

int main(void) {
	led.init();
	clock_setup();
//	gpio_setup();
	usart_setup();
	systick_setup();

	Sys::hostname("STM32F103");

	Log.setOutput(usartLog);

	/* Blink the LED (PC12) on the board with every transmitted byte. */


	Actor::initAll();

	while (1) {
//		gpio_toggle(LED_PORT, LED_PIN); /* LED on/off */
		LOGF("The quick brown fox jumps over the lazy dog in 1234567890-:/;,");
//		Sys::delay(100);
		for(int i=0;i<10000000;i++) i+=1;
//		usart_send_string("Hello\n");
		Actor::eventLoop();

	}

	return 0;
}

