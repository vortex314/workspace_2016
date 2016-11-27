/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <usb_serial.h>

#define MAPLE_MINI

#ifdef MAPLE_MINI
#define LED_PORT GPIOB
#define LED_PIN	 GPIO1
#define USB_PUP_PORT GPIOB
#define USB_PUP_PIN	GPIO9
#endif
#ifdef STM32F103_MINIMAL
#define LED_PORT GPIOC
#define LED_PIN	 GPIO13
#endif

static const struct usb_device_descriptor dev = { //
		.bLength = USB_DT_DEVICE_SIZE, //
				.bDescriptorType = USB_DT_DEVICE,  //
				.bcdUSB = 0x0200, //
				.bDeviceClass = USB_CLASS_CDC, //
				.bDeviceSubClass = 0, //
				.bDeviceProtocol = 0, //
				.bMaxPacketSize0 = 64, //
				.idVendor = 0x0483, //
				.idProduct = 0x5740, //
				.bcdDevice = 0x0200, //
				.iManufacturer = 1, //
				.iProduct = 2, //
				.iSerialNumber = 3, //
				.bNumConfigurations = 1, //
		};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = { //
		{ //
				.bLength = USB_DT_ENDPOINT_SIZE, //
						.bDescriptorType = USB_DT_ENDPOINT, //
						.bEndpointAddress = 0x83, //
						.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT, //
						.wMaxPacketSize = 16, //
						.bInterval = 255, //
				}//
		};

static const struct usb_endpoint_descriptor data_endp[] = { { //
		.bLength = USB_DT_ENDPOINT_SIZE, //
				.bDescriptorType = USB_DT_ENDPOINT, //
				.bEndpointAddress = 0x01, //
				.bmAttributes = USB_ENDPOINT_ATTR_BULK, //
				.wMaxPacketSize = 64, //
				.bInterval = 1, //
		},//
		{ //
		.bLength = USB_DT_ENDPOINT_SIZE, //
				.bDescriptorType = USB_DT_ENDPOINT, //
				.bEndpointAddress = 0x82, //
				.bmAttributes = USB_ENDPOINT_ATTR_BULK, //
				.wMaxPacketSize = 64, //
				.bInterval = 1, //
		} };

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
}__attribute__((packed)) cdcacm_functional_descriptors = { //
				.header = { //
						.bFunctionLength =
								sizeof(struct usb_cdc_header_descriptor), //
								.bDescriptorType = CS_INTERFACE, //
								.bDescriptorSubtype =
								USB_CDC_TYPE_HEADER, //
								.bcdCDC = 0x0110, //
						},//
				.call_mgmt = { //
								.bFunctionLength =
										sizeof(struct usb_cdc_call_management_descriptor), //
								.bDescriptorType = CS_INTERFACE, //
								.bDescriptorSubtype =
								USB_CDC_TYPE_CALL_MANAGEMENT, //
								.bmCapabilities = 0, //
								.bDataInterface = 1, //
						}, .acm = { //
								.bFunctionLength =
										sizeof(struct usb_cdc_acm_descriptor), //
								.bDescriptorType = CS_INTERFACE, //
								.bDescriptorSubtype = USB_CDC_TYPE_ACM, //
								.bmCapabilities = 0, //
						}, .cdc_union = { //
						.bFunctionLength =
								sizeof(struct usb_cdc_union_descriptor), //
								.bDescriptorType = CS_INTERFACE, //
								.bDescriptorSubtype = USB_CDC_TYPE_UNION, //
								.bControlInterface = 0, //
								.bSubordinateInterface0 = 1, //
						}, //
		};

static const struct usb_interface_descriptor comm_iface[] = { { //
		.bLength = USB_DT_INTERFACE_SIZE, //
				.bDescriptorType = USB_DT_INTERFACE, //
				.bInterfaceNumber = 0, //
				.bAlternateSetting = 0, //
				.bNumEndpoints = 1, //
				.bInterfaceClass = USB_CLASS_CDC, //
				.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM, //
				.bInterfaceProtocol = USB_CDC_PROTOCOL_AT, //
				.iInterface = 0, //
				.endpoint = comm_endp, //
				.extra = &cdcacm_functional_descriptors, //
				.extralen = sizeof(cdcacm_functional_descriptors), } };

static const struct usb_interface_descriptor data_iface[] = { { //
		.bLength = USB_DT_INTERFACE_SIZE, //
				.bDescriptorType = USB_DT_INTERFACE, //
				.bInterfaceNumber = 1, //
				.bAlternateSetting = 0, //
				.bNumEndpoints = 2, //
				.bInterfaceClass = USB_CLASS_DATA, //
				.bInterfaceSubClass = 0, //
				.bInterfaceProtocol = 0, //
				.iInterface = 0, .endpoint = data_endp, } };

static const struct usb_interface ifaces[] = { { //
		.num_altsetting = 1, //
				.altsetting = comm_iface, //
		},//
		{ //
		.num_altsetting = 1, //
				.altsetting = data_iface, //
		}//
};

static const struct usb_config_descriptor config = { //
		.bLength =
		USB_DT_CONFIGURATION_SIZE, //
				.bDescriptorType = USB_DT_CONFIGURATION, //
				.wTotalLength = 0, //
				.bNumInterfaces = 2, //
				.bConfigurationValue = 1, //
				.iConfiguration = 0, //
				.bmAttributes = 0x80, //
				.bMaxPower = 0x32, //
				.interface = ifaces, };

static const char *usb_strings[] = { "Black Sphere Technologies",
		"CDC-ACM Demo", "DEMO", };

static int cdcacm_control_request(usbd_device *usbd_dev,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req)) {
	(void) complete;
	(void) buf;
	(void) usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		char local_buf[10];
		struct usb_cdc_notification *notif = (void *) local_buf;

		/* We echo signals back to host as notification. */
		notif->bmRequestType = 0xA1;
		notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
		notif->wValue = 0;
		notif->wIndex = 0;
		notif->wLength = 2;
		local_buf[8] = req->wValue & 3;
		local_buf[9] = 0;
		usbd_ep_write_packet(usbd_dev, 0x83, buf, 10);
		return 1;
	}
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding))
			return 0;
		return 1;
	}
	return 0;
}
//__________________________________________________________________________________________________
//
usb_rxd_callback usb_rxd_function = 0;
usb_txd_callback usb_txd_function = 0;	//TODO
static bool usb_transmitting = false; //TODO
static bool usb_connected; 								// store USB connection status

uint8_t usbd_control_buffer[128];			/* Buffer to be used for control requests. */
static usbd_device *g_usbd_dev;
//__________________________________________________________________________________________________
//
void usb_on_rxd(usb_rxd_callback f) {
	usb_rxd_function = f;
}
//__________________________________________________________________________________________________
//
void usb_on_txd(usb_txd_callback f) {
	usb_txd_function = f;
}
//_____________________________________________________________________________________ usb_is_transmitting
//
bool usb_is_transmitting() {
	return usb_transmitting;
}
//________________________________________________________________________________________ suspend_cb
// use suspend callback to detect disconnect
static void suspend_cb(void) {
	usb_connected = false;
	usb_transmitting = false;
}
//_______________________________________________________________________________________  resume_cb
//

static void resume_cb(void) {
	usb_connected = true;
	usb_transmitting = false;
}
//______________________________________________________________________________________   reset_cb
//

static void reset_cb(void) {
	usb_connected = false;
	usb_transmitting = false;
}
//______________________________________________________________________________________  cdcacm_tx_cb
//

static void cdcacm_tx_cb(usbd_device *usbd_dev, uint8_t ep) { //EP IN callback transaction
	usb_transmitting = false;
	usb_txd_function();
}
//__________________________________________________________________________________________________
//
bool usb_txd(uint8_t* data, uint32_t length) {
//	if ( !usb_transmitting)
	if (usbd_ep_write_packet(g_usbd_dev, 0x82, data, length)) {
		usb_transmitting = true;
		return true;
	}
	return false;
}
//______________________________________________________________________________________ cdcacm_data_rx_cb
//
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) { //EP OUT callback
	(void) ep;
	(void) usbd_dev;

	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

	if (usb_rxd_function) {
		usb_rxd_function(buf, len);
	}
}
//__________________________________________________________________________________________________
//
static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue) {
	(void) wValue;
	(void) usbd_dev;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
			cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_tx_cb); // was NULL last arg
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(usbd_dev,
	USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
	USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, cdcacm_control_request);

	/*	usbd_register_control_callback(usbd_dev, //
	 USB_REQ_TYPE_ENDPOINT, //
	 USB_REQ_TYPE_IN, cdcacm_tx_cb);*/
	// use config and suspend callback to detect connect
	usb_connected = true;
	usbd_register_suspend_callback(usbd_dev, suspend_cb);
	usbd_register_resume_callback(usbd_dev, resume_cb);
	usbd_register_reset_callback(usbd_dev, reset_cb);
}
//__________________________________________________________________________________________________
//
void usb_wakeup_isr(void) {
	usbd_poll(g_usbd_dev);
}
//__________________________________________________________________________________________________
//
void usb_lp_can_rx0_isr(void) {
	usbd_poll(g_usbd_dev);
}
/* Setup pin to pull up the D+ high, so autodect works
 * with the bootloader.  The circuit is active low. */
/* Setup GPIOC Pin 12 to pull up the D+ high, so autodect works
 * with the bootloader.  The circuit is active low. */
void usb_renumeration_force() {
	gpio_set_mode(USB_PUP_PORT, GPIO_MODE_OUTPUT_2_MHZ,
	GPIO_CNF_OUTPUT_OPENDRAIN, USB_PUP_PIN);
	gpio_set(USB_PUP_PORT, USB_PUP_PIN);
	for (int i = 0; i < 0x800000; i++)
		__asm__("nop");

	gpio_clear(USB_PUP_PORT, USB_PUP_PIN);
}
//__________________________________________________________________________________________________
//
void usb_init() {
	usb_renumeration_force();

	g_usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings,
			3, usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(g_usbd_dev, cdcacm_set_config);

	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ); // enable only after usbd_dev is ready
	nvic_enable_irq(NVIC_USB_WAKEUP_IRQ);
}
//__________________________________________________________________________________________________
//
void usb_poll() {
	usbd_poll(g_usbd_dev);
}
//__________________________________________________________________________________________________
//
#define __NVIC_PRIO_BITS		4
static inline void __set_BASEPRI(uint32_t value) {
	__asm volatile ("MSR basepri, %0" : : "r" (value) : "memory");
}


