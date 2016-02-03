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
 * \file 	serial_usb_linux_io.hpp
 *
 * \author 	MAV'RIC Team
 *
 * \brief 	Implementation of serial wrapping around iostream (std::cout and std::cin)
 *
 * \details write(...) writes to std::cout    read(...) reads from std::cin
 *
 ******************************************************************************/


#include "hal/linux/serial_linux_io.hpp"
#include <iostream>

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Serial_linux_io::Serial_linux_io(serial_linux_io_conf_t config)
{
    config_ = config;
}


bool Serial_linux_io::init(void)
{
    return true;
}



uint32_t Serial_linux_io::readable(void)
{
    // Not implemented
    return 0;
}



uint32_t Serial_linux_io::writeable(void)
{
    // Not implemented
    return 0;
}


void Serial_linux_io::flush(void)
{
    std::cout.flush();
}


bool Serial_linux_io::attach(serial_interrupt_callback_t func)
{
    // Not implemented
    return false;
}


bool Serial_linux_io::write(const uint8_t* bytes, const uint32_t size)
{
    const char* text = (char*)bytes;
    std::cout.write(text, size);
    return true;
}


bool Serial_linux_io::read(uint8_t* bytes, const uint32_t size)
{
    std::cin.read((char*)bytes, size);
    return true;
}

/**
 * \brief 	write newline character to stream ('\n')
 *
 * \return 	success
 */
bool Serial_linux_io::newline()
{
    const uint8_t newline[2] = {'\n'};
    return write(newline, 1);
}
