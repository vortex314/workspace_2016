/*
 * main.c
 *
 *  Created on: 7-aug.-2016
 *      Author: 600104947
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <Sys.h>
#include <UsbSerial.h>
#include <SlipStream.h>
#include <Actor.h>
#include <Log.h>
#include <Usart.h>
#include <Led.h>
#include <EventBus.h>
#include <Cbor.h>

// void* __dso_handle;
static void clock_setup(void) {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable clocks for GPIO port B (for GPIO_USART3_TX) and USART3. */
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_USART3);
}

static void gpio_setup(void) {
	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
	GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

void usart_send_string(const char *s) {
	while (*s) {
		usart_send_blocking(USART1, *(s++));
	}
}

void usartLog(char* data, uint32_t length) {
	Bytes bytes((uint8_t*) data, length);
	usart1.write(bytes);
	usart1.flush();
}

void usartBufferedLog(char* data, uint32_t length) {
	Bytes bytes((uint8_t*) data, length);
	usart1.write(bytes);
	usart1.write('\n');
	usart1.flush();
}

static void systick_setup(void) {
	/* 72MHz / 8 => 9000000 counts per second. */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(8999);
	systick_interrupt_enable(); /* Start counting. */
	systick_counter_enable();
}

class Tracer: public Actor {
public:
	Tracer() :
			Actor("Tracer") {
	}
	void setup() {
		timeout(1000);
		eb.subscribe(H("timeout"), this, (MethodHandler) &Tracer::onEvent);
	}
	void onEvent(Cbor& msg) {
		uint16_t event;
		msg.getKeyValue(0, event);
		Str str(100);
		Cbor cbor(100);
		PT_BEGIN()
		;
		PT_WAIT_UNTIL(event == H("init"));

		while (true) {
			PT_WAIT_UNTIL(timeout());
			cbor.clear();
			cbor.addKeyValue(0, H("mqtt.connect"));
			str = "limero.ddns.net";
			cbor.addKeyValue(H("host"), str);
			cbor.addKeyValue(H("port"), 1883);
			eb.publish(cbor);
			timeout(1000);
			PT_WAIT_UNTIL(event == H("timeout"));

			cbor.clear();
			cbor.addKeyValue(0, H("mqtt.publish"));
			cbor.addKeyValue(H("topic"), "stm32/system/alive");
			cbor.addKeyValue(H("message"), "true");
			eb.publish(cbor);
			timeout(1000);
			PT_WAIT_UNTIL(event == H("timeout"));

			cbor.clear();
			cbor.addKeyValue(0, H("mqtt.publish"));
			cbor.addKeyValue(H("topic"), "stm32/system/uptime");
			cbor.addKeyValue(H("message"), Sys::millis());
			eb.publish(cbor);
			timeout(1000);
			PT_WAIT_UNTIL(event == H("timeout"));

		}
	PT_END()
}
};

SlipStream ss(256, usb);
Tracer tracer;
Led led;

int main(void) {

	clock_setup();
	gpio_setup();
	gpio_set(GPIOC, GPIO13);
	usart1.setup();
	systick_setup();
	led.setup();
	tracer.setup();
	ss.setup();
	usb.setup();

	Sys::hostname("STM32F103");

	//	Log.setOutput(usartLog);
	Log.setOutput(usartBufferedLog);
	LOGF(" ready to log ?");

	eb.subscribe(0, [](Cbor& cbor) { // all events
				usart_send_string(" any event \n");
				uint16_t cmd;
				if ( cbor.getKeyValue(0,cmd) ) {
					if ( cmd==H("mqtt.connect") || cmd==H("mqtt.subscribe")|| cmd==H("mqtt.publish")) {
						ss.send(cbor);
						LOGF("send");
					}
				}
			});

	while (1) {
		eb.eventLoop();

		if (usb.hasData()) {
			uint32_t count = 0;
			while (usb.hasData()) {
				ss.onRecv(usb.read());
				count++;
			}
			LOGF(" DATA RXD %d bytes.", count);
		}
	}

	return 0;
}

