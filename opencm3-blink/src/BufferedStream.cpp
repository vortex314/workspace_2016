/*
 * BufferedStream.cpp
 *
 *  Created on: 2-sep.-2016
 *      Author: lieven2
 */

#include <BufferedStream.h>

BufferedStream::BufferedStream(uint32_t size) :
		_rxd(size), _txd(size) {

}

BufferedStream::~BufferedStream() {

}

Erc BufferedStream::write(uint8_t b) {
	return _txd.write(b);
	return E_OK;
}

Erc BufferedStream::write(Bytes& bytes) {
	Erc erc;
	bytes.offset(0);
	if (!_txd.hasSpace(bytes.length()))
		return ENOBUFS;
	while (bytes.hasData()) {
		erc = write(bytes.read());
		if (erc)
			return erc;
	}
	return E_OK;
}

bool BufferedStream::hasSpace() {
	return _txd.hasSpace();
}

bool BufferedStream::hasData() {
	return _rxd.hasData();
}

bool BufferedStream::hasToSend() {
	return _txd.hasData();
}

uint8_t BufferedStream::toSend() {
	return _txd.read();
}

uint8_t BufferedStream::read() {
	return _rxd.read();
}

Erc BufferedStream::read(Bytes& bytes) {
	while (hasData())
		bytes.write(read());
	return E_OK;
}

bool BufferedStream::canReceive() {
	return _rxd.hasSpace();
}
void BufferedStream::receive(uint8_t data) {
	_rxd.write(data);
}

