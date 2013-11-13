/**
 * \file
 *
 * \brief USB Device Communication Device Class (CDC) interface definitions.
 *
 * Copyright (C) 2009 Atmel Corporation. All rights reserved.
 *
 * \page License
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
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef _UDI_CDC_H_
#define _UDI_CDC_H_

#include "conf_usb.h"
#include "usb_protocol.h"
#include "usb_protocol_cdc.h"
#include "udd.h"
#include "udc_desc.h"
#include "udi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup udi_group
 * \defgroup udi_cdc_group UDI for Communication Device Class
 *
 * @{
 */

/**
 * \name Interface Descriptor
 *
 * The following structures provide the interface descriptor.
 * It must be implemented in USB configuration descriptor.
 */
//@{


/**
 * \brief Communication Class interface descriptor
 *
 * Interface descriptor with associated functional and endpoint
 * descriptors for the CDC Communication Class interface.
 */
typedef struct {
	//! Standard interface descriptor
	usb_iface_desc_t iface;
	//! CDC Header functional descriptor
	usb_cdc_hdr_desc_t header;
	//! CDC Abstract Control Model functional descriptor
	usb_cdc_acm_desc_t acm;
	//! CDC Union functional descriptor
	usb_cdc_union_desc_t union_desc;
	//! CDC Call Management functional descriptor
	usb_cdc_call_mgmt_desc_t call_mgmt;
	//! Notification endpoint descriptor
	usb_ep_desc_t ep_notify;
} udi_cdc_comm_desc_t;


/**
 * \brief Data Class interface descriptor
 *
 * Interface descriptor with associated endpoint descriptors for the
 * CDC Data Class interface.
 */
typedef struct {
	//! Standard interface descriptor
	usb_iface_desc_t iface;
	//! Data IN/OUT endpoint descriptors
	usb_ep_desc_t ep_in;
	usb_ep_desc_t ep_out;
} udi_cdc_data_desc_t;


//! By default no string associated to these interfaces
#ifndef UDI_CDC_IAD_STRING_ID
#define UDI_CDC_IAD_STRING_ID   0
#endif
#ifndef UDI_CDC_COMM_STRING_ID
#define UDI_CDC_COMM_STRING_ID   0
#endif
#ifndef UDI_CDC_DATA_STRING_ID
#define UDI_CDC_DATA_STRING_ID   0
#endif

//! CDC communication enpoints size for all speed
#define UDI_CDC_COMM_EP_SIZE        64
//! CDC data enpoints size for all speed (no need to use 512B for HS)
#define UDI_CDC_DATA_EPS_SIZE       64

#ifndef UDI_CDC_MULTIPLE
//! Fill field including interface numbers
#  define UDI_CDC_COMM_DESC_IFACE      \
   .iface.bInterfaceNumber       = UDI_CDC_COMM_IFACE_NUMBER,\
   .call_mgmt.bDataInterface     = UDI_CDC_DATA_IFACE_NUMBER,\
   .union_desc.bMasterInterface  = UDI_CDC_COMM_IFACE_NUMBER,\
   .union_desc.bSlaveInterface0  = UDI_CDC_DATA_IFACE_NUMBER,
#  define UDI_CDC_DATA_DESC_IFACE     \
   .iface.bInterfaceNumber       = UDI_CDC_DATA_IFACE_NUMBER,
#else
//! TODO for multiple CDC interface
#  define UDI_CDC_COMM_DESC_IFACE
#  define UDI_CDC_DATA_DESC_IFACE
#endif

#define UDI_CDC_IAD_DESC        {\
   .bLength                      = sizeof(usb_iad_desc_t),\
   .bDescriptorType              = USB_DT_IAD,\
   .bFirstInterface              = UDI_CDC_COMM_IFACE_NUMBER,\
   .bInterfaceCount              = 2,\
   .bFunctionClass               = CDC_CLASS_COMM,\
   .bFunctionSubClass            = CDC_SUBCLASS_ACM,\
   .bFunctionProtocol            = CDC_PROTOCOL_V25TER,\
   .iFunction                    = UDI_CDC_IAD_STRING_ID,\
   }

//! Content of CDC COMM interface descriptor for all speed
#define UDI_CDC_COMM_DESC        {\
   UDI_CDC_COMM_DESC_IFACE \
   .iface.bLength                = sizeof(usb_iface_desc_t),\
   .iface.bDescriptorType        = USB_DT_INTERFACE,\
   .iface.bAlternateSetting      = 0,\
   .iface.bNumEndpoints          = 1,\
   .iface.bInterfaceClass        = CDC_CLASS_COMM,\
   .iface.bInterfaceSubClass     = CDC_SUBCLASS_ACM,\
   .iface.bInterfaceProtocol     = CDC_PROTOCOL_V25TER,\
   .iface.iInterface             = UDI_CDC_COMM_STRING_ID,\
   .header.bFunctionLength       = sizeof(usb_cdc_hdr_desc_t),\
   .header.bDescriptorType       = CDC_CS_INTERFACE,\
   .header.bDescriptorSubtype    = CDC_SCS_HEADER,\
   .header.bcdCDC                = LE16(0x0110),\
   .call_mgmt.bFunctionLength    = sizeof(usb_cdc_call_mgmt_desc_t),\
   .call_mgmt.bDescriptorType    = CDC_CS_INTERFACE,\
   .call_mgmt.bDescriptorSubtype = CDC_SCS_CALL_MGMT,\
   .call_mgmt.bmCapabilities     = \
			CDC_CALL_MGMT_SUPPORTED | CDC_CALL_MGMT_OVER_DCI,\
   .acm.bFunctionLength          = sizeof(usb_cdc_acm_desc_t),\
   .acm.bDescriptorType          = CDC_CS_INTERFACE,\
   .acm.bDescriptorSubtype       = CDC_SCS_ACM,\
   .acm.bmCapabilities           = CDC_ACM_SUPPORT_LINE_REQUESTS,\
   .union_desc.bFunctionLength   = sizeof(usb_cdc_union_desc_t),\
   .union_desc.bDescriptorType   = CDC_CS_INTERFACE,\
   .union_desc.bDescriptorSubtype= CDC_SCS_UNION,\
   .ep_notify.bLength            = sizeof(usb_ep_desc_t),\
   .ep_notify.bDescriptorType    = USB_DT_ENDPOINT,\
   .ep_notify.bEndpointAddress   = UDI_CDC_COMM_EP,\
   .ep_notify.bmAttributes       = USB_EP_TYPE_INTERRUPT,\
   .ep_notify.wMaxPacketSize     = LE16(UDI_CDC_COMM_EP_SIZE),\
   .ep_notify.bInterval          = 0xFF,\
   }


//! Content of CDC DATA interface descriptor for all speed
#define UDI_CDC_DATA_DESC     {\
   UDI_CDC_DATA_DESC_IFACE \
   .iface.bLength                = sizeof(usb_iface_desc_t),\
   .iface.bDescriptorType        = USB_DT_INTERFACE,\
   .iface.bAlternateSetting      = 0,\
   .iface.bNumEndpoints          = 2,\
   .iface.bInterfaceClass        = CDC_CLASS_DATA,\
   .iface.bInterfaceSubClass     = 0,\
   .iface.bInterfaceProtocol     = 0,\
   .iface.iInterface             = UDI_CDC_DATA_STRING_ID,\
   .ep_in.bLength                = sizeof(usb_ep_desc_t),\
   .ep_in.bDescriptorType        = USB_DT_ENDPOINT,\
   .ep_in.bEndpointAddress       = UDI_CDC_DATA_EP_IN,\
   .ep_in.wMaxPacketSize         = LE16(UDI_CDC_DATA_EPS_SIZE),\
   .ep_in.bmAttributes           = USB_EP_TYPE_BULK,\
   .ep_in.bInterval              = 0,\
   .ep_out.bLength               = sizeof(usb_ep_desc_t),\
   .ep_out.bDescriptorType       = USB_DT_ENDPOINT,\
   .ep_out.bEndpointAddress      = UDI_CDC_DATA_EP_OUT,\
   .ep_out.wMaxPacketSize        = LE16(UDI_CDC_DATA_EPS_SIZE),\
   .ep_out.bmAttributes          = USB_EP_TYPE_BULK,\
   .ep_out.bInterval             = 0,\
   }
//@}


//! Global struture which contains standard UDI API for UDC
extern UDC_DESC_STORAGE udi_api_t udi_api_cdc_comm;
extern UDC_DESC_STORAGE udi_api_t udi_api_cdc_data;

/**
 * \name Interface for application
 *
 * These routines are used by memory to transfer its data 
 * to/from USB CDC endpoint.
 */
//@{

/**
 * \brief Notify a state change of DCD signal
 *
 * \param b_set      DCD is enabled if true, else disabled
 */
void udi_cdc_ctrl_signal_dcd(bool b_set);

/**
 * \brief Notify a state change of DSR signal
 *
 * \param b_set      DSR is enabled if true, else disabled
 */
void udi_cdc_ctrl_signal_dsr(bool b_set);

/**
 * \brief Notify a framing error
 */
void udi_cdc_signal_framing_error(void);

/**
 * \brief Notify a parity error
 */
void udi_cdc_signal_parity_error(void);

/**
 * \brief Notify a overrun
 */
void udi_cdc_signal_overrun(void);

/**
 * \brief This function checks if a character has been received on the CDC line
 *
 * \return \c 1 if a byte is ready to be read.
 */
bool udi_cdc_is_rx_ready(void);

/**
 * \brief Waits and gets a value on CDC line
 *
 * \return value read on CDC line
 */
int udi_cdc_getc(void);

/**
 * \brief Reads a RAM buffer on CDC line
 *
 * \param buf       Values readed
 * \param size      Number of value readed
 *
 * \return the number of data remaining
 */
iram_size_t udi_cdc_read_buf(int* buf, iram_size_t size);

/**
 * \brief This function checks if a new character sent is possible
 * The type int is used to support scanf redirection from compiler LIB.
 *
 * \return \c 1 if a new chracter can be sent
 */
bool udi_cdc_is_tx_ready(void);

/**
 * \brief Puts a byte on CDC line
 * The type int is used to support printf redirection from compiler LIB.
 *
 * \param value      Value to put
 *
 * \return \c 1 if function was successfully done, otherwise \c 0.
 */
int udi_cdc_putc(int value);

/**
 * \brief Writes a RAM buffer on CDC line
 *
 * \param buf       Values to write
 * \param size      Number of value to write
 *
 * \return the number of data remaining
 */
iram_size_t udi_cdc_write_buf(const int* buf, iram_size_t size);

//@}

//@}

#ifdef __cplusplus
}
#endif
#endif // _UDI_CDC_H_
