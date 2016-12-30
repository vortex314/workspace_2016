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
#include <libopencm3/stm32/flash.h>

#include <Sys.h>
#include <UsbSerial.h>
#include <SlipStream.h>
#include <Actor.h>
#include <Log.h>
#include <Usart.h>
#include <Led.h>
#include <EventBus.h>
#include <Cbor.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>
#include <usb_serial.h>

EventBus eb(5120, 256);

// void* __dso_handle;

void rcc_clock_setup_in_hse_8mhz_out_48mhz(void) {
	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

	/* Enable external high-speed oscillator 8MHz. */
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

	/*
	 * Set prescalers for AHB, ADC, ABP1, ABP2.
	 * Do this before touching the PLL (TODO: why?).
	 */
	rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV); /* Set. 48MHz Max. 72MHz */
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV4); /* Set. 12MHz Max. 14MHz */
	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2); /* Set. 24MHz Max. 36MHz */
	rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV); /* Set. 48MHz Max. 72MHz */

	/*
	 * Sysclk runs with 24MHz -> 0 waitstates.
	 * 0WS from 0-24MHz
	 * 1WS from 24-48MHz
	 * 2WS from 48-72MHz
	 */
	flash_set_ws(FLASH_ACR_LATENCY_1WS);

	/*
	 * Set the PLL multiplication factor to 3.
	 * 8MHz (external) * 6 (multiplier) = 48MHz
	 */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL6);
	/* Select HSE as PLL source. */
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);
	/*
	 * External frequency undivided before entering PLL
	 * (Only valid / needed for HSE).
	 */
	rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);

	rcc_set_usbpre(RCC_CFGR_USBPRE_PLL_CLK_NODIV);

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);
	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);
	/* Set the peripheral clock frequencies used */
	rcc_ahb_frequency = 48000000;
	rcc_apb1_frequency = 24000000;
	rcc_apb2_frequency = 48000000;
}

static void clock_setup(void) {

//	rcc_periph_clock_enable(RCC_USB);
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable clocks for GPIO port B (for GPIO_USART3_TX) and USART3. */
//	rcc_periph_clock_enable(RCC_USART1);
//	rcc_periph_clock_enable(RCC_USART2);
//	rcc_periph_clock_enable(RCC_USART3);
//	rcc_clock_setup_in_hse_8mhz_out_48mhz();
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
	strncpy(logBuffer, data, length);
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
//_______________________________________________________________________________________________________________________________________
//
void publishInt(const char* topic,uint32_t value) {
        INFO("%s : %d ",topic,value);

    char sValue[30];
    sprintf(sValue,"%u",value);

    eb.request(H("mqtt"),H("publish"),H("MqttCl")).addKeyValue(H("topic"),topic).addKeyValue(H("message"),sValue);
    eb.send();

}
//_______________________________________________________________________________________________________________________________________
//
#define PREFIX "limero/"
class Counter {
    uint32_t _interval;
    const char* _name;
    uint64_t _start;
    uint32_t _count;
public:

    Counter(const char* name,uint32_t interval) {
        _interval=interval*1000;
        _name=name;
        _start=0;
    }

    void inc() {
        _count++;
        if ( Sys::_upTime- _start > _interval ) flush();
    }

    void flush() {
         uint32_t v=(1000*_count)/(Sys::millis()-_start);
       char field[30];
        strcpy(field,PREFIX);

        strcat(field,_name);
        strcat(field,"/count");
        publishInt(field,_count);

        strcpy(field,PREFIX);
        strcat(field,_name);
        strcat(field,"/perSec");
        publishInt(field,v);


        _start=Sys::millis();
        _count=0;
    }


};
Counter requests("requests",10);
Counter responses("responses",10);
Counter mismatchs("mismatch",10);
Counter correct("correct",10);
Counter timeouts("timeout",10);
//_______________________________________________________________________________________________________________________________________
//
class Tester: public Actor {
    uint32_t _counter;
    uint32_t _correct;
    uint32_t _incorrect;
    uint32_t _timeouts;
public:
    Tester() :
        Actor("Tester") {
        _counter=0;
        _correct=0;
        _incorrect=0;
        _timeouts=0;
    }
    void setup() {
        timeout(1000);
        eb.onReply(0,H("ping")).subscribe(this);
    }
    void onEvent(Cbor& msg) {
        PT_BEGIN()
        ;

        while (true) {
//           timeout(5000);
//           PT_YIELD_UNTIL(  timeout());

            eb.request(H("Echo"),H("ping"),H("Tester")).addKeyValue(H("uint32_t"),_counter);
            eb.send();
            requests.inc();

            timeout(5000);
            PT_YIELD_UNTIL( eb.isReply(0,H("ping")) || eb.isEvent(H("sys"),H("timeout")));
            uint32_t counter;
            if ( msg.getKeyValue(H("uint32_t"),counter) ) {
                if ( counter==_counter ) {
                    correct.inc();
                } else {
                    mismatchs.inc();
                }
            } else {
                timeouts.inc();
            }

        }
        PT_END()
    }
};


//_______________________________________________________________________________________________________________________________________
//

Tester tester;

class Tracer: public Actor {
public:
	Tracer() :
			Actor("Tracer") {
	}
	void setup() {
		timeout(1000);
		eb.onDst(H("Tracer")).subscribe(this);	// trap all events
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
		volatile int a = H("timeout");
		volatile int b = H("link.pong");
		volatile int c = H("mqtt.connack");
		uint16_t event;
		msg.getKeyValue(0, event);
		Str str(100);
		Cbor cbor(100);
		PT_BEGIN()
		PT_YIELD_UNTIL(timeout());
		while (true) {
			CONNECTING: while (true) {
				timeout(2000);
				eb.publish(H("link"), H("ping"));
				PT_YIELD_UNTIL(
						(event == H("timeout")) || (event == H("link.pong")));
				if (event == H("link.pong"))
					break;
			}

			MQTT_CONNECT: while (true) {
				timeout(2000);
				sendConnect();
				PT_YIELD_UNTIL(
						(event == H("timeout"))
								|| (event == H("mqtt.connack")));
				if (event == H("mqtt.connack") && !msg.gotoKey(H("error")))
					break;
				goto CONNECTING;
			}

			MQTT_SUBSCRIBE: while (true) {
				timeout(2000);
				sendSubscribe();
				PT_YIELD_UNTIL(
						(event == H("timeout")) || (event == H("mqtt.suback")));
				if (event == H("mqtt.suback") && !msg.gotoKey(H("error")))
					break;
				goto CONNECTING;
			}

			MQTT_PUBLISH: while (true) {
				cbor.clear();
				cbor.addKeyValue(0, H("mqtt.publish"));
				cbor.addKeyValue(H("topic"), "stm32/system/alive");
				cbor.addKeyValue(H("message"), "true");
				eb.publish(cbor);
				timeout(100);
				PT_YIELD_UNTIL(
						event == H("timeout") || event == H("mqtt.puback"));
				cbor.clear();
				cbor.addKeyValue(0, H("mqtt.publish"));
				cbor.addKeyValue(H("topic"), "stm32/system/uptime");
				str.append(Sys::millis());
				cbor.addKeyValue(H("message"), str);
				eb.publish(cbor);
				timeout(100);
				PT_YIELD_UNTIL(
						event == H("timeout") || event == H("mqtt.puback"));
				if (event == H("mqtt.puback"))
					if (msg.gotoKey(H("error")))
						goto CONNECTING;
//				timeout(100);
//				PT_YIELD_UNTIL(event == H("timeout")); // sleep

			}
		}
	PT_END()
}
};

SlipStream ss(256, usart1);
Tracer tracer;
Led led;

#pragma weak hard_fault_handler = HardFault_Handler

// Use the 'naked' attribute so that C stacking is not used.
extern "C" __attribute__((naked))
void HardFault_Handler(void) {
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

extern "C" void HardFault_HandlerC(unsigned long *hardfault_args) {
	volatile unsigned long stacked_r0;
	volatile unsigned long stacked_r1;
	volatile unsigned long stacked_r2;
	volatile unsigned long stacked_r3;
	volatile unsigned long stacked_r12;
	volatile unsigned long stacked_lr;
	volatile unsigned long stacked_pc;
	volatile unsigned long stacked_psr;
	volatile unsigned long _CFSR;
	volatile unsigned long _HFSR;
	volatile unsigned long _DFSR;
	volatile unsigned long _AFSR;
	volatile unsigned long _BFAR;
	volatile unsigned long _MMAR;

	stacked_r0 = ((unsigned long) hardfault_args[0]);
	stacked_r1 = ((unsigned long) hardfault_args[1]);
	stacked_r2 = ((unsigned long) hardfault_args[2]);
	stacked_r3 = ((unsigned long) hardfault_args[3]);
	stacked_r12 = ((unsigned long) hardfault_args[4]);
	stacked_lr = ((unsigned long) hardfault_args[5]);
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);

	// Configurable Fault Status Register
	// Consists of MMSR, BFSR and UFSR
	_CFSR = (*((volatile unsigned long *) (0xE000ED28)));

	// Hard Fault Status Register
	_HFSR = (*((volatile unsigned long *) (0xE000ED2C)));

	// Debug Fault Status Register
	_DFSR = (*((volatile unsigned long *) (0xE000ED30)));

	// Auxiliary Fault Status Register
	_AFSR = (*((volatile unsigned long *) (0xE000ED3C)));

	// Read the Fault Address Registers. These may not contain valid values.
	// Check BFARVALID/MMARVALID to see if they are valid values
	// MemManage Fault Address Register
	_MMAR = (*((volatile unsigned long *) (0xE000ED34)));
	// Bus Fault Address Register
	_BFAR = (*((volatile unsigned long *) (0xE000ED38)));

	__asm("BKPT #0\n");
	// Break into the debugger
}



extern "C" int my_usb();
int main(void) {

	volatile uint16_t hh = H("timeout");
	static_assert(H("timeout")==45638," testing");
//	static_assert(HASH("timeout")!=0," HASH ");
//	static_assert(HH("timeout")!=0," HH ");
//	my_usb();
	clock_setup();
	gpio_setup(); // not used
	gpio_set(GPIOC, GPIO13); // not used
	usart1.setup();
	systick_setup();
	led.setup();
	tracer.setup();
	usb.setup();
	ss.setup();
	tester.setup();

//	SCB_SHCSR |= SCB_SHCSR_MEMFAULTENA;
//	NVIC_SetPriority(MemoryM, 1);

	Sys::hostname("STM32F103");

	//	Log.setOutput(usartLog);
//	Log.setOutput(usartBufferedLog);
	Log.setOutput(bufferLog);
	LOGF(" ready to log ?");

	eb.onDst(H("mqtt")).subscribe([](Cbor& cbor) {
		usart1.write(cbor);
	});
	eb.onDst(H("Echo")).subscribe([](Cbor& cbor) {
		usart1.write(cbor);
	});

	eb.onEvent(H("usb"),H("rxd")).subscribe( [](Cbor& cbor) { // send usb data to slip processing
//				LOGF(" usb.rxd execute ");
				Bytes data(1000);
				if (cbor.getKeyValue(H("data"),data)) {
					data.offset(0);
					while (data.hasData()) {
						ss.onRecv(data.read());
					}
				} else LOGF(" no usb data ");
			});

	while (1) {
		eb.eventLoop();
	}

	return 0;
}

