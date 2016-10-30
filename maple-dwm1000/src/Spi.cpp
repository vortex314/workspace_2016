/*
 * Spi.cpp
 *
 *  Created on: 26-okt.-2016
 *      Author: lieven2
 */

#include "Spi.h"

Spi::Spi(uint32_t spiIdx) {
	if (spiIdx == 1)
		_SPI = SPI1;
	else
		_SPI = SPI2;
}

Spi::~Spi() {
}

void Spi::setup() {
	if (_SPI == SPI1) {
		/* Enable SPI1 Periph and gpio clocks */
		rcc_periph_clock_enable(RCC_SPI1);
		/* Configure GPIOs: SS=PA4, SCK=PA5, MISO=PA6 and MOSI=PA7 */
		gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4 |
		GPIO5 |
		GPIO7);
		gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
		GPIO6);
		_pinCS = new Gpio(PA(4));
		_pinCS->setMode(Gpio::Mode::GPIO_OUTPUT);
		_pinCS->write(1);
	} else if (_SPI == SPI2) {
		rcc_periph_clock_enable(RCC_SPI2);
		/* Configure GPIOs: SS=PB12, SCK=PB13, MISO=PB15 and MOSI=PB14 */

		gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO12 | GPIO13 | GPIO14);
		gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
		GPIO15);
		_pinCS = new Gpio(PB(12));
		_pinCS->setMode(Gpio::Mode::GPIO_OUTPUT);
		_pinCS->write(1);
	}
	/*
	 * 	spi_send_lsb_first(_SPI);
	 spi_set_full_duplex_mode(_SPI);
	 spi_set_standard_mode(_SPI,0);
	 spi_set_clock_phase_0(_SPI);
	 spi_set_clock_polarity_0(_SPI);
	 */

	/* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
	spi_reset(_SPI);

	/* Set up SPI in Master mode with:
	 * Clock baud rate: 1/64 of peripheral clock frequency
	 * Clock polarity: Idle High
	 * Clock phase: Data valid on 1st clock pulse
	 * Data frame format: 8-bit
	 * Frame format: MSB First
	 */
	spi_init_master(_SPI, //
			SPI_CR1_BAUDRATE_FPCLK_DIV_64, // clock 72MHZ/64 ??
			SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE, // CPOL=0
			SPI_CR1_CPHA_CLK_TRANSITION_1, // CPHA=0
			SPI_CR1_DFF_8BIT, // 1 byte at a time
			SPI_CR1_MSBFIRST); // MSB first

	/*
	 * Set NSS management to software.
	 *
	 * Note:
	 * Setting nss high is very important, even if we are controlling the GPIO
	 * ourselves this bit needs to be at least set to 1, otherwise the spi
	 * peripheral will not send any data out.
	 */
	spi_enable_software_slave_management(_SPI);
	spi_set_nss_high(_SPI);

	/* Enable SPI1 periph. */
	spi_enable(_SPI);

}

void Spi::setMode(Mode m) {
	_mode = m;
}

void Spi::setClock(uint32_t clock) {
	_clock = clock;
}

uint8_t Spi::xfer(uint8_t b) {
	_pinCS->write(0);
	uint16_t w = spi_xfer(_SPI, (uint16_t) b);
	_pinCS->write(1);
	return w;
}

void Spi::xfer(Bytes& input, Bytes& output) {
	output.offset(0);
	_pinCS->write(0);
	for (uint32_t i = 0; i < output.length(); i++) {
		input.write(spi_xfer(_SPI, output.read()));
	}
	_pinCS->write(1);
}

void spi1_isr() {

}

