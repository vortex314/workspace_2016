#include <Arduino.h>
#include <usb_serial.h>

const int led = PB1;

void setup(void) {
	pinMode(led, OUTPUT);
	digitalWrite(led, 1); // turn led off
	Serial1.begin(115200);
	// Initialize virtual COM over USB on Maple Mini
	Serial.begin(9600); // BAUD has no effect on USB serial: placeholder for physical UART
	// wait for serial monitor to be connected.

	while (!(Serial.isConnected() && (Serial.getDTR() || Serial.getRTS()))) {
		// toggleLED();     // <--- retired function can be user replaced
		delay(100);         // fast blink
	}
}

void loop(void) {
	while (Serial.isConnected() && Serial.available()) {
		Serial1.write(Serial.read());
	}
	Serial1.println(" The quick brown fox jumps over the lazy dog.");
	digitalWrite(led, 0); // turn led on
	delay(100);
	digitalWrite(led, 1); // turn led off
	delay(900);
}
