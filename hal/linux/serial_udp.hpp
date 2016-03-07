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
 * \file    serial_udp.hpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Linux implementation of serial peripherals using UDP
 *
 ******************************************************************************/

#ifndef SERIAL_UDP_H_
#define SERIAL_UDP_H_

#include "hal/common/serial.hpp"
#include "util/buffer.hpp"

extern "C"
{
#include <stdint.h>
#include <sys/socket.h>
#include <arpa/inet.h>
}

/**
 *  Configuration structure
 */
typedef struct
{
    const char* target_ip;
    int32_t     target_port;
    int32_t     local_port;
} serial_udp_conf_t;


/**
 * \brief   Default configuration
 *
 * \return  Config structure
 */
static inline serial_udp_conf_t serial_udp_default_config();


class Serial_udp: public Serial
{
public:
    /**
     * \brief   Initialises the peripheral
     *
     * \param   config      Device configuration
     */
    Serial_udp(serial_udp_conf_t config = serial_udp_default_config());


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
     * \brief   Write a byte on the serial line
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


private:
    serial_udp_conf_t   config_;

    Buffer_tpl<1024>    tx_buffer_;
    Buffer              rx_buffer_;

    int                 socket_;
    struct sockaddr_in  target_addr_;
    struct sockaddr_in  local_addr_;
};


/**
 * \brief   Default configuration
 *
 * \return  Config structure
 */
static inline serial_udp_conf_t serial_udp_default_config()
{
    serial_udp_conf_t   conf;

    conf.target_ip      = "127.0.0.1";
    conf.target_port    = 14550;
    conf.local_port     = 14000;

    return conf;
}


#endif /* SERIAL_UDP_H_ */