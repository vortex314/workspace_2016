/*
 * Usart.cpp
 *
 *  Created on: Aug 14, 2016
 *      Author: lieven2
 */

#include <Usart.h>

Usart usart1(1);

Usart::Usart(int idx) :
		Actor("Usart"), BufferedByteStream(UART_BUFFER_SIZE), _baudrate(115200), _usartBase(
		USART1 + (idx - 1) * 0x400) {
}

Usart::~Usart() {

}

void Usart::init() {
	open();
}

void Usart::loop() {
	if (hasData())
		publish(RXD);
}

Erc Usart::open() {
	/* Setup GPIO pin GPIO_USART1_TX. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
	GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
	GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, _baudrate);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);
	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE | USART_CR1_TXEIE;

	/* Finally enable the USART. */
	usart_enable(USART1);
	return E_OK;
}

Erc Usart::close() {
	return E_OK;
}

void Usart::flush() {
	USART_CR1(USART1) |= USART_CR1_TXEIE;
}

void usart1_isr(void) {
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0)
			&& ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
		usart1.receive(usart_recv(USART1));

		/* Enable transmit interrupt so it sends back the data. */
//		USART_CR1(USART1) |= USART_CR1_TXEIE;
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0)
			&& ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		if (usart1.hasToSend()) {
			/* Put data into the transmit register. */
			usart_send(USART1, usart1.toSend());
		} else {
			/* Disable the TXE interrupt, it's no longer needed. */
			USART_CR1(USART1) &= ~USART_CR1_TXEIE;
		}

	}
}

int _write(int file, char *ptr, int len) {
//	int ret;

	if (file == 1) {
		for (int i = 0; i < len; i++) {
			if (usart1.hasSpace())
				usart1.write(*(ptr + i));
			else
				break;
		}
		USART_CR1(USART1) |= USART_CR1_TXEIE;
	}
	return 0;
}

