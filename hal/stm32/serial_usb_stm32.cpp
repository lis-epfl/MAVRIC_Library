/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file    serial_usb_stm32.cpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Implementation of serial over USB for stm32
 *
 * \details Incomplete implementation (TODO)
 *          - Implemented:
 *              * buffered, blocking writing
 *              * Read functions
 *              * Receive interrupt callback
 *          - NOT implemented:
 *              * buffered input
 *
 ******************************************************************************/

#include "hal/stm32/serial_usb_stm32.hpp"

extern "C"
{
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/otg_fs.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
}

static usbd_device* usbd_dev;

static const struct usb_device_descriptor dev = {
                                                	.bLength            = USB_DT_DEVICE_SIZE,
                                                	.bDescriptorType    = USB_DT_DEVICE,
                                                	.bcdUSB             = 0x0200,
                                                	.bDeviceClass       = USB_CLASS_CDC,
                                                	.bDeviceSubClass    = 0,
                                                	.bDeviceProtocol    = 0,
                                                	.bMaxPacketSize0    = 64,
                                                    .idVendor           = 0x0483,   // default
                                                    .idProduct          = 0x5740,  // default
                                                	// .idVendor           = 0xe9f1,   // e9f1 = EPFL
                                                 //    .idProduct          = 0x0002,  // 0002 = Sparky
                                                    .bcdDevice          = 0x0200,
                                                	.iManufacturer      = 1,
                                                	.iProduct           = 2,
                                                	.iSerialNumber      = 3,
                                                	.bNumConfigurations = 1,
                                                };

/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] =   {
                                                                {
                                                                	.bLength           = USB_DT_ENDPOINT_SIZE,
                                                                	.bDescriptorType   = USB_DT_ENDPOINT,
                                                                	.bEndpointAddress  = 0x83,
                                                                	.bmAttributes      = USB_ENDPOINT_ATTR_INTERRUPT,
                                                                	.wMaxPacketSize    = 16,
                                                                	.bInterval         = 255,
                                                                }
                                                            };

static const struct usb_endpoint_descriptor data_endp[] =   {
                                                                {
                                                                	.bLength           = USB_DT_ENDPOINT_SIZE,
                                                                	.bDescriptorType   = USB_DT_ENDPOINT,
                                                                	.bEndpointAddress  = 0x01,
                                                                	.bmAttributes      = USB_ENDPOINT_ATTR_BULK,
                                                                	.wMaxPacketSize    = 64,
                                                                	.bInterval         = 0,
                                                                },
                                                                {
                                                                	.bLength           = USB_DT_ENDPOINT_SIZE,
                                                                	.bDescriptorType   = USB_DT_ENDPOINT,
                                                                	.bEndpointAddress  = 0x82,
                                                                	.bmAttributes      = USB_ENDPOINT_ATTR_BULK,
                                                                	.wMaxPacketSize    = 64,
                                                                	.bInterval         = 0,
                                                                }
                                                            };

static const struct
{
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
                                                            .header =   {
                                                                            .bFunctionLength    = sizeof(struct usb_cdc_header_descriptor),
                                                                            .bDescriptorType    = CS_INTERFACE,
                                                                            .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
                                                                            .bcdCDC             = 0x0110,
                                                                        },
                                                            .call_mgmt = {
                                                                            .bFunctionLength    = sizeof(struct usb_cdc_call_management_descriptor),
                                                                            .bDescriptorType    = CS_INTERFACE,
                                                                            .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
                                                                            .bmCapabilities     = 0,
                                                                            .bDataInterface     = 1,
                                                                        },
                                                            .acm =  {
                                                                        .bFunctionLength    = sizeof(struct usb_cdc_acm_descriptor),
                                                                        .bDescriptorType    = CS_INTERFACE,
                                                                        .bDescriptorSubtype = USB_CDC_TYPE_ACM,
                                                                        .bmCapabilities     = 0,
                                                                    },
                                                            .cdc_union = {
                                                                            .bFunctionLength        = sizeof(struct usb_cdc_union_descriptor),
                                                                            .bDescriptorType        = CS_INTERFACE,
                                                                            .bDescriptorSubtype     = USB_CDC_TYPE_UNION,
                                                                            .bControlInterface      = 0,
                                                                            .bSubordinateInterface0 = 1,
                                                                        }
                                                            };

static const struct usb_interface_descriptor comm_iface[] = {
                                                                {
                                                                	.bLength            = USB_DT_INTERFACE_SIZE,
                                                                	.bDescriptorType    = USB_DT_INTERFACE,
                                                                	.bInterfaceNumber   = 0,
                                                                	.bAlternateSetting  = 0,
                                                                	.bNumEndpoints      = 1,
                                                                	.bInterfaceClass    = USB_CLASS_CDC,
                                                                	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
                                                                	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
                                                                	.iInterface         = 0,
                                                                	.endpoint           = comm_endp,
                                                                	.extra              = &cdcacm_functional_descriptors,
                                                                	.extralen           = sizeof(cdcacm_functional_descriptors)
                                                                }
                                                            };

static const struct usb_interface_descriptor data_iface[] = {
                                                                {
                                                                	.bLength            = USB_DT_INTERFACE_SIZE,
                                                                	.bDescriptorType    = USB_DT_INTERFACE,
                                                                	.bInterfaceNumber   = 1,
                                                                	.bAlternateSetting  = 0,
                                                                	.bNumEndpoints      = 2,
                                                                	.bInterfaceClass    = USB_CLASS_DATA,
                                                                	.bInterfaceSubClass = 0,
                                                                	.bInterfaceProtocol = 0,
                                                                	.iInterface         = 0,
                                                                	.endpoint           = data_endp,
                                                                }
                                                            };

static const struct usb_config_descriptor::usb_interface ifaces[] = {
                                                                        {NULL, 1, NULL, comm_iface},
                                                                    	{NULL, 1, NULL, data_iface}
                                                                    };

static const struct usb_config_descriptor config = {
                                                    	.bLength               = USB_DT_CONFIGURATION_SIZE,
                                                    	.bDescriptorType       = USB_DT_CONFIGURATION,
                                                    	.wTotalLength          = 0,
                                                    	.bNumInterfaces        = 2,
                                                    	.bConfigurationValue   = 1,
                                                    	.iConfiguration        = 0,
                                                    	.bmAttributes          = 0x80,
                                                    	.bMaxPower             = 0x32,
                                                    	.interface             = ifaces,
                                                    };

static const char * usb_strings[] = {
                                    	"MAVRIC Autopilot",
                                    	"STM32F4",
                                        "USB CDC"
                                    };

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static int cdcacm_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		return 1;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding)) {
			return 0;
		}

		return 1;
	}
	return 0;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep);

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
			cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Serial_usb_stm32::Serial_usb_stm32(conf_t config)
{
    // Store config
    config_ = config;

    // set interupt callback to null
    irq_callback = NULL;
}


bool Serial_usb_stm32::init(void)
{
    // Init usb hardware
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_OTGFS);

    if (config_.enable_vbus_detection)
    {
        // VBUS detection
        gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO12);
        gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);
    }
    else
    {
        // No VBUS detection
        OTG_FS_GCCFG|=OTG_GCCFG_NOVBUSSENS;
        gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
        gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);
    }

    usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
			usb_strings, 3,
			usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

    nvic_enable_irq(NVIC_OTG_FS_IRQ);

    usbd_poll(usbd_dev);

    // Set handler to point at this object, if there are more than 1 usb
    // this would need to change to an array pointing to all of the objects
    instance_ = this;

    return true;
}



uint32_t Serial_usb_stm32::readable(void)
{
    return rx_buffer_.readable();
}



uint32_t Serial_usb_stm32::writeable(void)
{
    return tx_buffer_.writeable();
}


void Serial_usb_stm32::flush(void)
{
    // Block until everything is sent
    while (!tx_buffer_.empty())
    {
        // Get number of bytes to send
        uint32_t to_send = tx_buffer_.readable();

        // Limit to 64 bytes at a time
        if (to_send >= 64)
        {
            to_send = 64;
        }

        // Get data into a buffer
        uint8_t buf[to_send];
        for (size_t i = 0; i < to_send; i++)
        {
            tx_buffer_.get(buf[i]);
        }

        // Send data
        usbd_ep_write_packet(usbd_dev, 0x82, buf, to_send);
    }
}


bool Serial_usb_stm32::attach(serial_interrupt_callback_t func)
{
    // Set callback function for data read
    irq_callback = func;
    return true;
}


bool Serial_usb_stm32::write(const uint8_t* bytes, const uint32_t size)
{
    bool ret = false;

    // Queue bytes
    if (writeable() >= size)
    {
        for (uint32_t i = 0; i < size; ++i)
        {
            tx_buffer_.put(bytes[i]);
        }
        ret = true;
    }


    // Flush
    flush();

    return ret;
}


bool Serial_usb_stm32::read(uint8_t* bytes, const uint32_t size)
{
    bool ret = false;

    // Check if there is enough data to be read
    if (rx_buffer_.readable() >= size)
    {
        ret = true;

        // Read data
        for (uint32_t i = 0; i < size; ++i)
        {
            ret &= rx_buffer_.get(bytes[i]);
        }
    }

    return ret;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Serial_usb_stm32* Serial_usb_stm32::instance_ = NULL;

void otg_fs_isr(void)
{
    usbd_poll(usbd_dev);
}


static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
    // Calls the static function which determines what to do with incoming data
    Serial_usb_stm32::irq();
}


// Calls the handler for the desired usb class (only 1 usb atm)
void Serial_usb_stm32::irq(void)
{
    // If the Serial_usb_stm32 object has been created
    if (instance_)
    {
        // Run interrupt function
        instance_->irq_handler();
    }
}


// Determines what to do with incoming data
void Serial_usb_stm32::irq_handler(void)
{
    char buf[64];
    uint32_t len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    for (uint32_t i = 0; i < len; i++)
    {
        rx_buffer_.put_lossy(buf[i]);
    }

    // Call callback function if attached
    if (irq_callback != NULL)
    {
        irq_callback(this);
    }
}
