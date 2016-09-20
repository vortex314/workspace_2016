/*
 * BufferedStream.h
 *
 *  Created on: 2-sep.-2016
 *      Author: lieven2
 */

#ifndef BUFFEREDSTREAM_H_
#define BUFFEREDSTREAM_H_
#include <Bytes.h>
#include <CircBuf.h>
#include <Stream.h>
class BufferedStream : public Stream{
	CircBuf _rxd;
	CircBuf _txd;
public:
	// user interface
	BufferedStream(uint32_t sizeBuffer);
	~BufferedStream();
	virtual Erc open()=0;
	virtual Erc close()=0;
	virtual void flush()=0;
	Erc write(Bytes& data);
	Erc write(uint8_t data);
	uint8_t read();
	Erc read(Bytes& data);
	bool hasData();
	bool hasSpace();
	// ISR
	bool canReceive();
	void receive(uint8_t data);
	bool hasToSend();
	uint8_t toSend();
};

#endif /* BUFFEREDSTREAM_H_ */
