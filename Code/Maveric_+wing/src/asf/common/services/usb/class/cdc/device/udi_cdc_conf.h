/**
 * \file
 *
 * \brief Default CDC configuration for a USB Device with a single interface
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

#ifndef _UDI_CDC_CONF_H_
#define _UDI_CDC_CONF_H_

#include "usb_protocol_cdc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup udi_cdc_group
 * \defgroup udi_cdc_group_conf Default CDC configuration for a USB Device
 * with a single interface CDC
 *
 * @{
 */

//! Control endpoint size (Endpoint 0)
#define  USB_DEVICE_EP_CTRL_SIZE       64

//! Endpoints' numbers used by single CDC interface
#define  UDI_CDC_DATA_EP_IN            (1 | USB_EP_DIR_IN)	// TX
#define  UDI_CDC_DATA_EP_OUT           (2 | USB_EP_DIR_OUT)	// RX
#define  UDI_CDC_COMM_EP               (3 | USB_EP_DIR_IN)	// Notify endpoint

//! Interface numbers
#define  UDI_CDC_COMM_IFACE_NUMBER     0
#define  UDI_CDC_DATA_IFACE_NUMBER     1

/**
 * \name UDD Configuration
 */
//@{
//! 3 endpoints used by single CDC interface
#define  USB_DEVICE_MAX_EP             3
//@}

//@}

#ifdef __cplusplus
}
#endif
#endif // _UDI_CDC_CONF_H_
