/*
 * Stream.h
 *
 *  Created on: 2-sep.-2016
 *      Author: lieven2
 */

#ifndef STREAM_H_
#define STREAM_H_
#include <Bytes.h>
class Stream {
public:
	Stream(){};
	virtual ~Stream(){};
	virtual Erc open()=0;
	virtual Erc close()=0;
	virtual Erc write(Bytes& data)=0;
	virtual Erc write(uint8_t data)=0;
	virtual uint8_t read()=0;
	virtual Erc read(Bytes& data)=0;
	virtual bool hasData()=0;
	virtual bool hasSpace()=0;
	virtual void flush()=0;
};

#endif /* STREAM_H_ */
