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
 * \file    serial_chibios.cpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Implementation of serial peripheral using ChibiOS
 *
 ******************************************************************************/


#include "hal/chibios/serial_chibios.hpp"

///< Array of 'this' pointers used for interrupt handling
// const uint8_t SERIAL_MAX_NUMBER = 6;
static Serial_chibios* handlers_[Serial_chibios::SERIAL_MAX_NUMBER] = {0};

// declaration of irq callbacks
extern "C"
{
void txend1(UARTDriver *uartp);
void rxerr(UARTDriver *uartp, uartflags_t e);
void rxend(UARTDriver *uartp);
void usart0_txend2(UARTDriver *uartp);
void usart1_txend2(UARTDriver *uartp);
void usart2_txend2(UARTDriver *uartp);
void usart3_txend2(UARTDriver *uartp);
void usart4_txend2(UARTDriver *uartp);
void usart5_txend2(UARTDriver *uartp);
void usart0_rxchar(UARTDriver *uartp, uint16_t c);
void usart1_rxchar(UARTDriver *uartp, uint16_t c);
void usart2_rxchar(UARTDriver *uartp, uint16_t c);
void usart3_rxchar(UARTDriver *uartp, uint16_t c);
void usart4_rxchar(UARTDriver *uartp, uint16_t c);
void usart5_rxchar(UARTDriver *uartp, uint16_t c);
const uartcb_t tx_callbacks[] = {&usart0_txend2, &usart1_txend2, &usart2_txend2,
                                 &usart3_txend2, &usart4_txend2, &usart5_txend2};
const uartccb_t rx_callbacks[] = {&usart0_rxchar, &usart1_rxchar, &usart2_rxchar,
                                  &usart3_rxchar, &usart4_rxchar, &usart5_rxchar};
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Serial_chibios::Serial_chibios(conf_t config):
    config_(config),
    is_sending_(false),
    irq_callback_(0)
{}


bool Serial_chibios::init(void)
{
    // Register instance
    handlers_[config_.id] = this;

    // UART configuration
    UARTConfig uartconfig =
    {
        txend1,
        tx_callbacks[config_.id],
        rxend,
        rx_callbacks[config_.id],
        rxerr,
        config_.baudrate,
        0,
        USART_CR2_LINEN,
        0
    };

    // Start device
    uartStart(config_.device, &uartconfig);

    // Make sure everything is stopped
    uartStopReceive(config_.device);
    uartStopSend(config_.device);

    return true;
}



uint32_t Serial_chibios::readable(void)
{
    return rx_buffer_.readable();
}



uint32_t Serial_chibios::writeable(void)
{
    return tx_buffer_.writeable();
}


void Serial_chibios::flush(void)
{
    // Start transmission
    if (is_sending_ == false)
    {
        tx_irq_handler();
    }

    // Wait until the transmit buffer is empty
    while (!tx_buffer_.empty())
    {
        ;
    }
}


bool Serial_chibios::attach(serial_interrupt_callback_t func)
{
    irq_callback_ = func;
    return true;
}


bool Serial_chibios::write(const uint8_t* bytes, const uint32_t size)
{
    bool ret = false;

    if (is_sending_)
    {
        //  Queue byte
        if (writeable() >= size)
        {
            for (uint32_t i = 0; i < size; ++i)
            {
                tx_buffer_.put(bytes[i]);
            }
            ret = true;
        }
    }
    else
    {
        // Notify we are sending
        is_sending_ = true;

        // Send now
        uartStartSend(config_.device, size, bytes);
    }


    return ret;
}


bool Serial_chibios::read(uint8_t* bytes, const uint32_t size)
{
    bool ret = false;

    if (readable() >= size)
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

void Serial_chibios::tx_irq_handler(void)
{
    uint32_t to_send = tx_buffer_.readable();
    if (to_send > 0)
    {
        // Limit number of bytes we send
        if (to_send > 64)
        {
            to_send = 64;
        }

        // Get data from output buffer
        uint8_t data[to_send];
        for (size_t i = 0; i < to_send; i++)
        {
            tx_buffer_.get(data[i]);
        }

        // Put data into the transmit register
        // uartStopSendI(&UARTD2);
        uartStartSendI(config_.device, to_send, data);

        // Notify we are sending
        is_sending_ = true;
    }
    else
    {
        // Notify we are not sending
        is_sending_ = false;
    }
}

void Serial_chibios::rx_irq_handler(uint16_t c)
{
    // Put data in rx buffer
    rx_buffer_.put_lossy(c);

    // Call callback function if attached
    if (irq_callback_ != 0)
    {
        irq_callback_(this);
    }
}


/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
void txend1(UARTDriver *uartp)
{
  (void)uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
void rxerr(UARTDriver *uartp, uartflags_t e)
{
  (void)uartp;
  (void)e;
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
void rxend(UARTDriver *uartp)
{
  (void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
void usart0_txend2(UARTDriver *uartp)
{
    (void)uartp;
    handlers_[0]->tx_irq_handler();
}
void usart1_txend2(UARTDriver *uartp)
{
    (void)uartp;
    handlers_[1]->tx_irq_handler();
}
void usart2_txend2(UARTDriver *uartp)
{
    (void)uartp;
    handlers_[2]->tx_irq_handler();
}
void usart3_txend2(UARTDriver *uartp)
{
    (void)uartp;
    handlers_[3]->tx_irq_handler();
}
void usart4_txend2(UARTDriver *uartp)
{
    (void)uartp;
    handlers_[4]->tx_irq_handler();
}
void usart5_txend2(UARTDriver *uartp)
{
    (void)uartp;
    handlers_[5]->tx_irq_handler();
}


/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
void usart0_rxchar(UARTDriver *uartp, uint16_t c)
{
    (void)uartp;
    handlers_[0]->rx_irq_handler(c);
}
void usart1_rxchar(UARTDriver *uartp, uint16_t c)
{
    (void)uartp;
    handlers_[1]->rx_irq_handler(c);
}
void usart2_rxchar(UARTDriver *uartp, uint16_t c)
{
    (void)uartp;
    handlers_[2]->rx_irq_handler(c);
}
void usart3_rxchar(UARTDriver *uartp, uint16_t c)
{
    (void)uartp;
    handlers_[3]->rx_irq_handler(c);
}
void usart4_rxchar(UARTDriver *uartp, uint16_t c)
{
    (void)uartp;
    handlers_[4]->rx_irq_handler(c);
}
void usart5_rxchar(UARTDriver *uartp, uint16_t c)
{
    (void)uartp;
    handlers_[5]->rx_irq_handler(c);
}
