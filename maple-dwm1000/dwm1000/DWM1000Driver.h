/*
 * DWM1000Driver.h
 *
 *  Created on: 30-okt.-2016
 *      Author: lieven2
 */

#ifndef DWM1000DRIVER_H_
#define DWM1000DRIVER_H_

#include <stdint.h>
#include <deca_types.h>

#include <EventBus.h>
#include <Connector.h>

class DWM1000_Driver : public Actor {
public:
	DWM1000_Driver();
	virtual ~DWM1000_Driver();
	void setup();
	void init();
	void resetChip();
	bool getInterrupt();
	virtual void onIrq(){};
	Spi* getSpi();

private:
	Connector* _connector;
	Spi* _spi;
	Gpio* _pinReset;
	Gpio* _pinIrq;
};

extern "C" int readfromspi                         // returns offset where requested data begins in supplied buffer, or, -1 for error.
(
    uint16       headerLength,          // input parameter - number of bytes header to write
    const uint8 *headerBuffer,          // input parameter - pointer to buffer containing the 'headerLength' bytes of header to write
    uint32       readlength,            // input parameter - number of bytes data being read
    uint8       *readBuffer             // input parameter - pointer to buffer containing to return the data (NB: size required = headerLength + readlength)
) ;

extern "C" int writetospi                          // returns 0 for success, or, -1 for error.
(
    uint16       headerLength,          // input parameter - number of bytes header being written
    const uint8 *headerBuffer,          // input parameter - pointer to buffer containing the 'headerLength' bytes of header to be written
    uint32       bodylength,            // input parameter - number of bytes data being written
    const uint8 *bodyBuffer             // input parameter - pointer to buffer containing the 'bodylength' bytes od data to be written
) ;

extern "C" void deca_sleep(unsigned int time_ms);



#endif /* DWM1000DRIVER_H_ */
