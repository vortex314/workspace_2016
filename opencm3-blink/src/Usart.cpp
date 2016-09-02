/*
 * Usart.cpp
 *
 *  Created on: Aug 14, 2016
 *      Author: lieven2
 */

#include <Usart.h>

Usart usart1;

Usart::Usart() :
		Actor("Usart"), _txd(UART_BUFFER_SIZE), _rxd(UART_BUFFER_SIZE) {

}

Usart::~Usart() {

}

/******************************************************************************
 * Simple ringbuffer implementation from open-bldc's libgovernor that
 * you can find at:
 * https://github.com/open-bldc/open-bldc/tree/master/source/libgovernor
 *****************************************************************************/
void Usart::init() {

	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);
	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;
}

Erc Usart::write(uint8_t b) {
	_txd.write(b);
	return E_OK;
}

void usart1_isr(void) {
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0)
			&& ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
		usart1._rxd.write(usart_recv(USART1));

		/* Enable transmit interrupt so it sends back the data. */
		USART_CR1(USART1) |= USART_CR1_TXEIE;
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0)
			&& ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		if (usart1._txd.hasData()) {
			/* Put data into the transmit register. */
			usart_send(USART1, usart1._txd.read());
		} else {
			/* Disable the TXE interrupt, it's no longer needed. */
			USART_CR1(USART1) &= ~USART_CR1_TXEIE;
		}

	}
}

int _write(int file, char *ptr, int len) {
	int ret;

	if (file == 1) {
		for (int i = 0; i < len; i++) {
			if (usart1._txd.hasSpace())
				usart1._txd.write(*(ptr + i));
			else
				break;
		}
		USART_CR1(USART1) |= USART_CR1_TXEIE;
	}
	return 0;
}

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

void Usart::loop() {

}
