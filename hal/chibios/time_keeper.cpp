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
 * \file time_keeper.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief This file is used to interact with the clock of the microcontroller
 *
 * \detail  Implementation for STM32
 *
 *
 ******************************************************************************/

#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "libs/ChibiOS/os/hal/include/hal.h"
}


void time_keeper_init(void)
{
    ;
}


double time_keeper_get_s(void)
{
    // time in seconds since system start
    return (double)(osalOsGetSystemTimeX()) / 1000000.0;
}


uint64_t time_keeper_get_ms(void)
{
    // milliseconds since system start
    return osalOsGetSystemTimeX() / 1000;
}


uint64_t time_keeper_get_us(void)
{
    // microseconds since system start. Will run over after an hour.
    return osalOsGetSystemTimeX();
}


void time_keeper_delay_us(uint64_t microseconds)
{
    osalThreadSleepMicroseconds(microseconds);
}


void time_keeper_delay_ms(uint64_t milliseconds)
{
    osalThreadSleepMilliseconds(milliseconds);
}


void time_keeper_sleep_us(uint64_t microseconds)
{
    osalThreadSleepMicroseconds(microseconds);
}
