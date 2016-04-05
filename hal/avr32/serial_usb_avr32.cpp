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
 * \file    serial_usb_avr32.cpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Implementation of serial over USB for avr32
 *
 * \details Incomplete implementation (TODO)
 *          - Implemented:
 *              * buffered, blocking writing
 *          - NOT implemented:
 *              * Read functions
 *              * Receive interrupt callback
 *              * buffered input
 *
 ******************************************************************************/


#include "hal/avr32/serial_usb_avr32.hpp"

extern "C"
{
#include <stdint.h>
#include "libs/asf/common/utils/stdio/stdio_usb/stdio_usb.h"
#include "libs/asf/common/services/usb/class/cdc/device/udi_cdc.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Serial_usb_avr32::Serial_usb_avr32(serial_usb_avr32_conf_t config)
{
    // Store config
    config_ = config;

    // set interupt callback to null
    irq_callback = NULL;
}


bool Serial_usb_avr32::init(void)
{
    // Init usb hardware
    stdio_usb_init(NULL);
    stdio_usb_enable();
    stdio_usb_vbus_event(true);
    handlers_ = this;

    return true;
}



uint32_t Serial_usb_avr32::readable(void)
{
    return rx_buffer_.readable();
}



uint32_t Serial_usb_avr32::writeable(void)
{
    return tx_buffer_.writeable();
}


void Serial_usb_avr32::flush(void)
{
    uint8_t byte = 0;

    // Block until everything is sent
    while (!tx_buffer_.empty())
    {
        if (udi_cdc_is_tx_ready())
        {
            // Get one byte
            tx_buffer_.get(byte);

            // Write byte
            stdio_usb_putchar(NULL, (int)byte);
        }
    }
}


bool Serial_usb_avr32::attach(serial_interrupt_callback_t func)
{
    irq_callback = func;
    return true;
}


bool Serial_usb_avr32::write(const uint8_t* bytes, const uint32_t size)
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


    // Try to flush "softly":  do not block if fails more than size times
    for (uint8_t i = 0; i < size; i++)
    {
        while (udi_cdc_is_tx_ready() && (!tx_buffer_.empty()))
        {
            uint8_t byte = 0;
            tx_buffer_.get(byte);
            stdio_usb_putchar(NULL, (int)byte);
        }
    }


    // If buffer is almost full, flush
    // if( writeable() < 80 )
    // {
    //  flush();
    // }

    return ret;
}


bool Serial_usb_avr32::read(uint8_t* bytes, const uint32_t size)
{
    bool ret = false;
    if (rx_buffer_.readable() >= size) // Testing
    {
        ret = true;
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

Serial_usb_avr32* Serial_usb_avr32::handlers_ = NULL;


// Notify incoming usb data
void usb_interupt_rx_notify()
{
    // Calls the static function which determines what to do with incoming data
    Serial_usb_avr32::irq();
}


// Calls the handler for the desired usb class (only 1 usb atm)
void Serial_usb_avr32::irq(void)
{
    if (handlers_)
    {
        handlers_->irq_handler();
    }
}


// Determines what to do with incoming data
void Serial_usb_avr32::irq_handler(void)
{
    // Receive all data
    while (udi_cdc_is_rx_ready())
    {
        uint8_t c1 = 0;
        stdio_usb_getchar_read_buf(NULL, (int*)&c1);
        rx_buffer_.put_lossy(c1);
    }

    // Call callback function if attached
    if (irq_callback != NULL)
    {
        irq_callback(this);
    }
}