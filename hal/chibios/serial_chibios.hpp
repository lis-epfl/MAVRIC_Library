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
 * \file    serial_chibios.hpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Implementation of serial peripheral using ChibiOS
 *
 ******************************************************************************/

#ifndef SERIAL_CHIBIOS_HPP_
#define SERIAL_CHIBIOS_HPP_

#include "hal/common/serial.hpp"

extern "C"
{
#include "hal.h"
}

#include "util/buffer.hpp"


/**
 * \brief   Implementation of serial peripheral for avr32
 */
class Serial_chibios: public Serial
{
public:
    /**
     * \brief   Enumerate the possible UARTs
     */
    typedef enum
    {
        SERIAL_1          = 0,
        SERIAL_2          = 1,
        SERIAL_3          = 2,
        SERIAL_4          = 3,
        SERIAL_5          = 4,
        SERIAL_6          = 5,
        SERIAL_MAX_NUMBER = 6
    } devices_t;

    /**
     * \brief Configuration structure
     */
    struct conf_t
    {
        devices_t   id;
        UARTDriver* device;
        uint32_t    baudrate;
    };

    /**
     * \brief   Initialises the peripheral
     *
     * \param   config      Device configuration
     */
    Serial_chibios(conf_t config);


    /**
     * \brief   Hardware initialization
     *
     * \return  true Success
     * \return  false Error
     */
    bool init(void);


    /**
     * \brief   Test if there are bytes available to read
     *
     * \return  Number of incoming bytes available
     */
    uint32_t readable(void);


    /**
     * \brief   Test if there is space available to write bytes
     *
     * \return  Number of bytes available for writing
     */
    uint32_t writeable(void);


    /**
     * \brief   Sends instantaneously all outgoing bytes
     *
     * \return  Number of bytes available for writing
     */
    void flush(void);


    /**
     * \brief   Attach a function to call after a receive interrupt is generated
     *
     * \details A default handler should be provided by the implementation to
     *          add the incoming data in a buffer, so is not mandatory to call
     *          this method. The function callback will be called after the
     *          interrupt handler
     *
     * \param   func        Pointer to the callback function
     *
     * \return  true        Success
     * \return  false       Failed
     */
    bool attach(serial_interrupt_callback_t func);


    /**
     * \brief   Write bytes on the serial line
     *
     * \param   byte        Outgoing bytes
     * \param   size        Number of bytes to write
     *
     * \return  true        Data successfully written
     * \return  false       Data not written
     */
    bool write(const uint8_t* bytes, const uint32_t size = 1);


    /**
     * \brief   Read bytes from the serial line
     *
     * \param   bytes       Incoming bytes
     * \param   size        Number of bytes to read
     *
     * \return  true        Data successfully read
     * \return  false       Data not read
     */
    bool read(uint8_t* bytes, const uint32_t size = 1);


    /**
     * \brief       Default RX interrupt-handling function
     */
    void rx_irq_handler(uint16_t c);


    /**
     * \brief       Default TX interrupt-handling function
     */
    void tx_irq_handler(void);


private:
    conf_t            config_;            ///< Configuration
    Buffer_T<1024>  tx_buffer_;         ///< Transmission buffer
    Buffer_T<1024>  rx_buffer_;         ///< Reception buffer
    bool              is_sending_;        ///< Flag telling if we are currently sending a buffer

    /**
     * \brief       Callback function to be called after an interrupt
     *
     * \details     By default NULL, can be modified via the 'attach' method
     */
    serial_interrupt_callback_t irq_callback_;
};


#endif /* SERIAL_CHIBIOS_HPP_ */
