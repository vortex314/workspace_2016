/*
 * SlipStream.cpp
 *
 *  Created on: 24-sep.-2016
 *      Author: lieven2
 */

#include <SlipStream.h>

SlipStream::SlipStream(int size,ByteStream& stream) : Bytes(size),Actor("SlipStream"),_stream(stream) {
_escaped=false;
_error_bad_crc=0;
}

SlipStream::~SlipStream() {
	// TODO Auto-generated destructor stub
}
//_________________________________________________________________________


void SlipStream::streamWriteCrc(Bytes& bytes) {
	bytes.offset(-1); // position at end
	Crc crc = Fletcher16(bytes.data(), bytes.used());
	streamWriteEscaped(crc.b[0]);
	streamWriteEscaped(crc.b[1]);
}

Crc SlipStream::Fletcher16(uint8_t *begin, int length) {
	uint16_t sum1 = 0;
	uint16_t sum2 = 0;
	uint8_t *end = begin + length;
	uint8_t *index;

	for (index = begin; index < end; index++) {
		sum1 = (sum1 + *index) % 255;
		sum2 = (sum2 + sum1) % 255;
//		LOGF(" %X , %X ", sum1, sum2);
	};
	Crc crc={sum2,sum1};
	return crc;
}
//_________________________________________________________________________

bool SlipStream::isGoodCrc() //PUBLIC
//_________________________________________________________________________
{
	if (_limit < 3)
		return false; // need at least 3 bytes
	Crc crc = Fletcher16(_start, _limit - 2);
	if ((*(_start + _limit - 2) == crc.b[0])
			&& ((*(_start + _limit - 1) == crc.b[1])))
		return true;
	return false;
}

//_________________________________________________________________________

void SlipStream::removeCrc() //PUBLIC
//_________________________________________________________________________
{
	if ( length() > 2)
	length(length()-2);
}
//PUBLIC

//PUBLIC
//_________________________________________________________________________

/*
 #define END 0xC0
 #define ESC 0xDB
 */
//_____________________________________

void SlipStream::streamWriteEscaped(uint8_t b){
if ( b==END ){
			_stream.write(ESC);
			_stream.write(ESC_END);
		} else if ( b==ESC) {
			_stream.write(ESC);
			_stream.write(ESC_ESC);
		} else
			_stream.write(b);
}


void SlipStream::send(Bytes& bytes){
	bytes.offset(0);
	_stream.write(END);
	while(bytes.hasData()){
		byte b= bytes.read();
		streamWriteEscaped(b);
		}
	streamWriteCrc(bytes);
	_stream.write(END);
	}



void SlipStream::onRecv(uint8_t b) {
	if (b == END) {
		if (offset() > 2){
			if ( isGoodCrc()) {
				removeCrc();
				publishSync(Header(RXD));
			}
			else
				_error_bad_crc++;
		}
		clear();
	} else if (b == ESC) {
		_escaped = true;
	} else if (b == ESC_ESC && _escaped) {
		write(ESC);
		_escaped = false;
	} else if (b == ESC_END && _escaped) {
		write(END);
		_escaped = false;
	} else {
		write(b);
	}
}

void SlipStream::loop(){
	if ( _stream.hasData()) {
		onRecv(_stream.read());
	}
}
