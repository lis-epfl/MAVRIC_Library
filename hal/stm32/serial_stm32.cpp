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
 * \file    serial_avr32.cpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Implementation of serial peripheral for avr32
 *
 ******************************************************************************/


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include "hal/stm32/serial_stm32.hpp"




///< Array of 'this' pointers used for interrupt handling
static Serial_stm32* handlers_[SERIAL_STM32_MAX_NUMBER] = {0};


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Serial_stm32::Serial_stm32(serial_stm32_conf_t config)
{
    config_         = config;
    irq_callback    = 0;
}


bool Serial_stm32::init(void)
{
    // Enable the USART interrupt
    switch (config_.device)
    {
        case SERIAL_STM32_1:
            rcc_periph_clock_enable(RCC_USART1);
            nvic_enable_irq(NVIC_USART1_IRQ);
            handlers_[0] = this;
            break;

        case SERIAL_STM32_2:
            rcc_periph_clock_enable(RCC_USART2);
            nvic_enable_irq(NVIC_USART2_IRQ);
            handlers_[1] = this;
            break;

        case SERIAL_STM32_3:
            rcc_periph_clock_enable(RCC_USART3);
            nvic_enable_irq(NVIC_USART3_IRQ);
            handlers_[2] = this;
            break;

        case SERIAL_STM32_4:
            rcc_periph_clock_enable(RCC_UART4);
            nvic_enable_irq(NVIC_UART4_IRQ);
            handlers_[3] = this;
            break;

        case SERIAL_STM32_5:
            rcc_periph_clock_enable(RCC_UART5);
            nvic_enable_irq(NVIC_UART5_IRQ);
            handlers_[4] = this;
            break;

        case SERIAL_STM32_6:
            rcc_periph_clock_enable(RCC_USART6);
            nvic_enable_irq(NVIC_USART6_IRQ);
            handlers_[5] = this;
            break;

        case SERIAL_STM32_7:
            rcc_periph_clock_enable(RCC_UART7);
            nvic_enable_irq(NVIC_UART7_IRQ);
            handlers_[6] = this;
            break;

        case SERIAL_STM32_8:
            rcc_periph_clock_enable(RCC_UART8);
            nvic_enable_irq(NVIC_UART8_IRQ);
            handlers_[7] = this;
            break;

        default:
            break;
    }

    // Setup GPIO pins for TX
    gpio_mode_setup(config_.tx_port,
                    GPIO_MODE_AF,
                    GPIO_PUPD_NONE,
                    config_.tx_pin);

    // Setup GPIO pins for RX
    gpio_mode_setup(config_.rx_port,
                    GPIO_MODE_AF,
                    GPIO_PUPD_NONE,
                    config_.rx_pin);

    // Setup TX and RX pins alternate function
    gpio_set_af(config_.tx_port, config_.tx_af, config_.tx_pin);
    gpio_set_af(config_.rx_port, config_.rx_af, config_.rx_pin);

    // Setup USART parameters
    usart_set_baudrate(config_.device, config_.baudrate);
    usart_set_databits(config_.device, config_.databits);
    usart_set_stopbits(config_.device, config_.stopbits);
    usart_set_mode(config_.device, config_.mode);
    usart_set_parity(config_.device, config_.parity);
    usart_set_flow_control(config_.device, config_.flow_control);

    // Enable RX interrupt
    usart_enable_rx_interrupt(config_.device);

    // Finally enable the USART
    usart_enable(config_.device);

    return true;
}



uint32_t Serial_stm32::readable(void)
{
    return rx_buffer_.readable();
}



uint32_t Serial_stm32::writeable(void)
{
    return tx_buffer_.writeable();
}


void Serial_stm32::flush(void)
{
    usart_enable_tx_interrupt(config_.device);

    // Wait until the transmit buffer is empty
    while (!tx_buffer_.empty())
    {
        ;
    }
}


bool Serial_stm32::attach(serial_interrupt_callback_t func)
{
    irq_callback = func;
    return true;
}


bool Serial_stm32::write(const uint8_t* bytes, const uint32_t size)
{
    bool ret = false;

    //  Queue byte
    if (writeable() >= size)
    {
        for (uint32_t i = 0; i < size; ++i)
        {
            tx_buffer_.put(bytes[i]);
        }
        ret = true;
    }

    // Start transmission
    if (tx_buffer_.readable() >= 1)
    {
        usart_enable_tx_interrupt(config_.device);
    }

    return ret;
}


bool Serial_stm32::read(uint8_t* bytes, const uint32_t size)
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



/**
 * \brief       Main irq function
 *
 * \details     This function has to be static in order to be registerable
 *              as interrupt handler. It does internally the dispatch to
 *              call the 'irq_handler' function on the right object instance
 */
__attribute__((interrupt))
void usart1_isr(void)
{
    handlers_[0]->irq_handler();
}


__attribute__((interrupt))
void usart2_isr(void)
{
    handlers_[1]->irq_handler();
}


__attribute__((interrupt))
void usart3_isr(void)
{
    handlers_[2]->irq_handler();
}


__attribute__((interrupt))
void uart4_isr(void)
{
    handlers_[3]->irq_handler();
}


__attribute__((interrupt))
void uart5_isr(void)
{
    handlers_[4]->irq_handler();
}


__attribute__((interrupt))
void usart6_isr(void)
{
    handlers_[5]->irq_handler();
}


__attribute__((interrupt))
void uart7_isr(void)
{
    handlers_[6]->irq_handler();
}


__attribute__((interrupt))
void uart8_isr(void)
{
    handlers_[7]->irq_handler();
}


void Serial_stm32::irq_handler(void)
{
    uint8_t data = 0;

    // Check if we were called because of RXNE
    if (((USART_CR1(config_.device) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(config_.device)  & USART_SR_RXNE)    != 0))
    {

        // Retrieve the data from the peripheral
        data = usart_recv(config_.device);
        rx_buffer_.put_lossy(data);

        // Enable transmit interrupt so it sends back the data
        usart_enable_tx_interrupt(config_.device);
    }

    // Check if we were called because of TXE
    if (((USART_CR1(config_.device) & USART_CR1_TXEIE) != 0) &&
            ((USART_SR(config_.device)  & USART_SR_TXE)    != 0))
    {
        if (tx_buffer_.readable() > 0)
        {
            // Get data from output buffer
            tx_buffer_.get(data);

            // Put data into the transmit register
            usart_send(config_.device, data);
        }
        else
        {
            // Disable the TXE interrupt as we don't need it anymore
            usart_disable_tx_interrupt(config_.device);
        }
    }

    // Call callback function if attached
    if (irq_callback != 0)
    {
        irq_callback(this);
    }
}