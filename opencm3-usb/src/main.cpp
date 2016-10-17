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
	rcc_periph_clock_enable(RCC_USB);

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


char logBuffer[256];

void bufferLog(char* data, uint32_t length) {
	strncpy(logBuffer,data,length);
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
		eb.subscribe(this);	// trap all events
	}

	void sendConnect() {
		Str str(20);
		Cbor cbor(50);
		cbor.addKeyValue(0, H("mqtt.connect"));
		str = "limero.ddns.net";
		cbor.addKeyValue(H("host"), str);
		cbor.addKeyValue(H("port"), 1883);
		eb.publish(cbor);
	}

	void sendSubscribe() {
		Str str(20);
		Cbor cbor(50);
		cbor.addKeyValue(0, H("mqtt.subscribe"));
		str = "put/stm32/#";
		cbor.addKeyValue(H("topic"), str);
		eb.publish(cbor);
	}

	void onEvent(Cbor& msg) {
		int a=H("timeout");
		int b=H("link.pong");
		uint16_t event;
		msg.getKeyValue(0, event);
		Str str(100);
		Cbor cbor(100);
		PT_BEGIN()
		PT_YIELD_UNTIL(timeout());
		while (true) {
			CONNECTING: while (true) {
				timeout(2000);
				eb.publish(H("link.ping"));
				PT_YIELD_UNTIL(
						(event == H("timeout")) || (event == H("link.pong")));
				if (event == H("link.pong"))
					break;
			}

			while (true) {
				timeout(2000);
				sendConnect();
				PT_YIELD_UNTIL(
						(event == H("timeout"))
								|| (event == H("mqtt.connack")));
				if (event == H("mqtt.connack") && !msg.gotoKey(H("error")))
					break;
			}

			while (true) {
				timeout(1000);
				sendSubscribe();
				PT_YIELD_UNTIL(
						(event == H("timeout")) || (event == H("mqtt.suback")));
				if (event == H("mqtt.suback")) {
					if (msg.gotoKey(H("error")))
						goto CONNECTING;
					else
						break;
				}
			}

			while (true) {
				timeout(100);
				cbor.clear();
				cbor.addKeyValue(0, H("mqtt.publish"));
				cbor.addKeyValue(H("topic"), "stm32/system/alive");
				cbor.addKeyValue(H("message"), "true");
				eb.publish(cbor);
				cbor.clear();
				cbor.addKeyValue(0, H("mqtt.publish"));
				cbor.addKeyValue(H("topic"), "stm32/system/uptime");
				str.append(Sys::millis());
				cbor.addKeyValue(H("message"), str);
				PT_YIELD_UNTIL(
						event == H("timeout") || event == H("mqtt.puback"));
				if (event == H("mqtt.puback"))
					if (msg.gotoKey(H("error")))
						goto CONNECTING;
			}
		}
	PT_END()
}
};

SlipStream ss(256, usb);
Tracer tracer;
Led led;

#pragma weak hard_fault_handler = HardFault_Handler

// Use the 'naked' attribute so that C stacking is not used.
extern "C" __attribute__((naked))
 void HardFault_Handler(void){
        /*
         * Get the appropriate stack pointer, depending on our mode,
         * and use it as the parameter to the C handler. This function
         * will never return
         */
         // ".syntax unified\n"
        __asm(
                        "MOVS   R0, #4  \n"
                        "MOV    R1, LR  \n"
                        "TST    R0, R1  \n"
                        "BEQ    _MSP    \n"
                        "MRS    R0, PSP \n"
                        "B      HardFault_HandlerC      \n"
                "_MSP:  \n"
                        "MRS    R0, MSP \n"
                        "B      HardFault_HandlerC      \n"
                );
        // ".syntax divided\n"
}

extern "C" void HardFault_HandlerC(unsigned long *hardfault_args){
  volatile unsigned long stacked_r0 ;
  volatile unsigned long stacked_r1 ;
  volatile unsigned long stacked_r2 ;
  volatile unsigned long stacked_r3 ;
  volatile unsigned long stacked_r12 ;
  volatile unsigned long stacked_lr ;
  volatile unsigned long stacked_pc ;
  volatile unsigned long stacked_psr ;
  volatile unsigned long _CFSR ;
  volatile unsigned long _HFSR ;
  volatile unsigned long _DFSR ;
  volatile unsigned long _AFSR ;
  volatile unsigned long _BFAR ;
  volatile unsigned long _MMAR ;

  stacked_r0 = ((unsigned long)hardfault_args[0]) ;
  stacked_r1 = ((unsigned long)hardfault_args[1]) ;
  stacked_r2 = ((unsigned long)hardfault_args[2]) ;
  stacked_r3 = ((unsigned long)hardfault_args[3]) ;
  stacked_r12 = ((unsigned long)hardfault_args[4]) ;
  stacked_lr = ((unsigned long)hardfault_args[5]) ;
  stacked_pc = ((unsigned long)hardfault_args[6]) ;
  stacked_psr = ((unsigned long)hardfault_args[7]) ;

  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;

  // Hard Fault Status Register
  _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

  // Debug Fault Status Register
  _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

  // Auxiliary Fault Status Register
  _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
  // Bus Fault Address Register
  _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

  __asm("BKPT #0\n") ; // Break into the debugger
}

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>
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
//	SCB_SHCSR |= SCB_SHCSR_MEMFAULTENA;
//	NVIC_SetPriority(MemoryM, 1);

	Sys::hostname("STM32F103");

	//	Log.setOutput(usartLog);
//	Log.setOutput(usartBufferedLog);
	Log.setOutput(bufferLog);
	LOGF(" ready to log ?");

	eb.subscribe(0, [](Cbor& cbor) { // route events to gateway
//				usart_send_string(" any event \n");
				uint16_t cmd;
				if ( cbor.getKeyValue(0,cmd) ) {
					if ( cmd==H("mqtt.connect") || cmd==H("mqtt.subscribe")|| cmd==H("mqtt.publish") || cmd==H("link.ping")) {
						ss.send(cbor);
						LOGF("send");
					}
				}
			});

	while (1) {
		eb.eventLoop();
	}

	return 0;
}

