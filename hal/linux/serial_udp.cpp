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
 * \file    serial_udp.cpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Linux implementation of serial peripherals using UDP
 *
 ******************************************************************************/

#include <stdexcept>
#include <unistd.h>
#include <fcntl.h>

#include "hal/linux/serial_udp.hpp"

//FASYNC is an existing flag on linux, missing in cygwin
#ifndef FASYNC
#define FASYNC _FASYNC
#endif


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Serial_udp::Serial_udp(serial_udp_conf_t config)
{
    // Copy config
    config_ = config;

    // Create udp socket
    socket_ = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == -1)
    {
        throw std::runtime_error("could not create UDP socket");
    }

    // Set local address and port
    local_addr_.sin_family      = AF_INET;                      // Internet socket
    local_addr_.sin_addr.s_addr = INADDR_ANY;                   // Any host ip
    local_addr_.sin_port        = htons(config_.local_port);    // Port provided by config

    // Bind the socket local port - necessary to receive packets from qgroundcontrol
    int r = bind(socket_, (struct sockaddr*)&local_addr_, sizeof(struct sockaddr));
    if (r != 0)
    {
        close(socket_);
        throw std::runtime_error("could not bind UDP socket");
    }

    // Attempt to make it non blocking
    if (fcntl(socket_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
        close(socket_);
        throw std::runtime_error("could set non blocking UDP socket");
    }

    // Set target address and port
    target_addr_.sin_family         = AF_INET;
    target_addr_.sin_addr.s_addr    = inet_addr(config_.target_ip);
    target_addr_.sin_port           = htons(config_.target_port);
}


bool Serial_udp::init(void)
{
    return true;
}


uint32_t Serial_udp::readable(void)
{
    int32_t  recsize;
    uint32_t n_bytes_to_read = rx_buffer_.writeable();
    char     buf[n_bytes_to_read];

    recsize = recv(socket_, buf, n_bytes_to_read, 0);

    for (int32_t i = 0; i < recsize; i++)
    {
        rx_buffer_.put(buf[i]);
    }

    return rx_buffer_.readable();
}



uint32_t Serial_udp::writeable(void)
{
    return tx_buffer_.writeable();
}


void Serial_udp::flush(void)
{
    uint32_t n_bytes_to_send = tx_buffer_.readable();
    uint8_t to_send[n_bytes_to_send];
    uint32_t n_sent = 0;

    for (uint32_t i = 0; i < n_bytes_to_send; ++i)
    {
        tx_buffer_.get(to_send[i]);
    }

    while (n_bytes_to_send > 0)
    {
        int32_t ret = sendto(socket_,
                             (const char*)&to_send[n_sent],
                             n_bytes_to_send,
                             0,
                             (struct sockaddr*)&target_addr_,
                             sizeof(struct sockaddr_in));

        if (ret != -1)
        {
            n_sent += ret;
            n_bytes_to_send -= ret;
        }
    }
}


bool Serial_udp::attach(serial_interrupt_callback_t func)
{
    // Not implemented
    return false;
}


bool Serial_udp::write(const uint8_t* bytes, const uint32_t size)
{
    bool ret = true;

    // Queue byte
    for (uint32_t i = 0; i < size; ++i)
    {
        ret &= tx_buffer_.put(bytes[i]);
    }

    // Start transmission
    if (tx_buffer_.readable() >= 1)
    {
        flush();
    }

    return ret;
}


bool Serial_udp::read(uint8_t bytes[], const uint32_t size)
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
