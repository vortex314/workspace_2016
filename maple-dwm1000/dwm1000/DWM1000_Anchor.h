/*
 * DWM1000_Anchor_Tag.h
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#ifndef DWM1000_Anchor_H_
#define DWM1000_Anchor_H_

#include <Connector.h>
#include <DWM1000Driver.h>

class DWM1000_Anchor: public DWM1000_Driver {
	uint32_t _count;
	static uint32_t _status_reg;

public:
	DWM1000_Anchor();
	virtual ~DWM1000_Anchor();
	void mode(uint32_t m);
//	void init();
	void initSpi();
	void enableIsr();
	static bool interrupt_detected ;
	void onIrq();

	bool isInterruptDetected();
	void clearInterrupt();
	void init();
	void onEvent(Cbor& cbor);
};

#endif /* DWM1000_Anchor_Tag_H_ */
