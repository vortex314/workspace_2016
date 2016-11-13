/*
 * DWM1000Driver.cpp
 *
 *  Created on: 30-okt.-2016
 *      Author: lieven2
 */

#include <DWM1000Driver.h>
#include <deca_device_api.h>

DWM1000_Driver* _DWM1000;

DWM1000_Driver::DWM1000_Driver() :
		Actor("DWM1000") {
	_spi = 0;
	_pinIrq = 0;
	_pinReset = 0;
	_connector = 0;
	_DWM1000 = this;
	init();
}

DWM1000_Driver::~DWM1000_Driver() {
}
//______________________________________________________ HARd RESEST DWM1000 via PIN
//
void DWM1000_Driver::resetChip() {

	_pinReset->write(0);
	deca_sleep(10);
	_pinReset->write(1);
	deca_sleep(10);
}
//______________________________________________________
//

bool DWM1000_Driver::getInterrupt() {
	return _pinIrq->read();
}
//______________________________________________________
//
void DWM1000_Driver::init() {
	int idx = 2;
	_connector = new Connector(idx);
	_spi = _connector->getSpi();
	_spi->setMode(Spi::Mode::POL0_PHA0);
	_spi->setClock(100000);
	_pinReset = _connector->getGpio(Connector::PIN_TXD);
	_pinReset->setMode(Gpio::GPIO_OUTPUT);
	_pinIrq = _connector->getGpio(Connector::PIN_RXD);
	_pinIrq->setMode(Gpio::GPIO_INPUT);
	_pinIrq->setInterrupt(Gpio::GPIO_EDGE_FALLING, []() {
		if ( _DWM1000 ) _DWM1000->onIrq();
	});
	timeout(1000);
}
//______________________________________________________
//
void DWM1000_Driver::setup() {
	_spi->setup();
	_pinReset->setup();
	_pinIrq->setup();
}
//______________________________________________________
//
Spi* DWM1000_Driver::getSpi() {
	return _spi;
}

#define SPI_BUFSIZE 1025
Bytes spiInput(SPI_BUFSIZE);
Bytes spiOutput(SPI_BUFSIZE);
//______________________________________________________
//
extern "C" int readfromspi // returns offset where requested data begins in supplied buffer, or, -1 for error.
(uint16 headerLength,       // input parameter - number of bytes header to write
		const uint8 *headerBuffer, // input parameter - pointer to buffer containing the 'headerLength' bytes of header to write
		uint32 readlength,  // input parameter - number of bytes data being read
		uint8 *readBuffer // input parameter - pointer to buffer containing to return the data (NB: size required = headerLength + readlength)
		) {
	ASSERT(headerLength < SPI_BUFSIZE);
	ASSERT(readlength < SPI_BUFSIZE);
	spiOutput.clear().append((uint8_t*) headerBuffer, headerLength);
	for (uint32_t i = 0; i < readlength; i++) // fill with zero bytes
		spiOutput.write((uint8_t) 0);

	spiInput.clear();

	_DWM1000->getSpi()->xfer(spiInput, spiOutput);

	spiInput.offset(headerLength);
	Bytes input(readBuffer,readlength);
	while(spiInput.hasData()){
		input.write(spiInput.read());
	}

	return DWT_SUCCESS;
}
//______________________________________________________
//
extern "C" int writetospi            // returns 0 for success, or, -1 for error.
(uint16 headerLength,  // input parameter - number of bytes header being written
		const uint8 *headerBuffer, // input parameter - pointer to buffer containing the 'headerLength' bytes of header to be written
		uint32 bodylength, // input parameter - number of bytes data being written
		const uint8 *bodyBuffer // input parameter - pointer to buffer containing the 'bodylength' bytes od data to be written
		) {
	ASSERT(headerLength+bodylength < SPI_BUFSIZE);
	spiOutput.clear().append((uint8_t*) headerBuffer, headerLength).append(
			(uint8_t*) bodyBuffer, bodylength);
	spiInput.clear();

	_DWM1000->getSpi()->xfer(spiInput, spiOutput);
	return DWT_SUCCESS;
}
//______________________________________________________
//
extern "C" void deca_sleep(unsigned int time_ms) {
	uint64_t endTime = Sys::millis() + time_ms;
	while (endTime > Sys::millis())
		;
}

