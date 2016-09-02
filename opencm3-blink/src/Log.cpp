/*
 * LOg.cpp
 *
 *  Created on: Jul 3, 2016
 *      Author: lieven
 */

#include <Log.h>
#include <stdlib.h>
#include <stdio.h>
#ifdef ARDUINO
#include <WString.h>
#include <Arduino.h>
#endif
#ifdef OPENCM3
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#endif
LogManager Log;

void serialLog(char* start, uint32_t length) {
#ifdef ARDUINO
	Serial.write(start, length);
	Serial.write("\r\n");
#endif
#ifdef OPENCM3
	start++;
	length++;
#endif
}

LogManager::LogManager() {
	_record = new char[LINE_LENGTH];
	enable();
	_offset = 0;
	defaultOutput();
}

LogManager::~LogManager() {
	delete _record;
}

bool LogManager::enabled() {
	return _enabled;
}
void LogManager::disable() {
	_enabled = false;
}
void LogManager::enable() {
	_enabled = true;
}

void LogManager::defaultOutput() {
	_logFunction = serialLog;
}

void LogManager::setOutput(LogFunction function) {
	_logFunction = function;
}

void LogManager::printf(const char* fmt, ...) {
	va_list args;
	va_start(args, fmt);
	if (_offset < LINE_LENGTH)
		_offset += vsnprintf((char*) (_record + _offset), LINE_LENGTH - _offset,
				fmt, args);
	va_end(args);
}

void LogManager::flush() {
	if (_logFunction)
		_logFunction(_record, _offset);
	_offset = 0;
}

