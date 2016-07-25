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
 * \file time_keeper.hpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 *
 * \brief This file is used to interact with the clock of the microcontroller
 *
 ******************************************************************************/


#ifndef TIME_KEEPER_HPP_
#define TIME_KEEPER_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * \brief   This function initialize the clock of the microcontroller
 */
void time_keeper_init(void);


/**
 * \brief   This function returns the time in seconds since system start
 *
 * \return  The time in seconds since system start
 */
double time_keeper_get_s(void);


/**
 * \brief   This function returns the time in milliseconds since system start
 *
 * \return The time in milliseconds since system start
 */
uint64_t time_keeper_get_ms(void);


/**
 * \brief   This function returns the time in microseconds since system start.
 *
 * \warning Will run over after an hour.
 *
 * \return The time in microseconds since system start
 */
uint64_t time_keeper_get_us(void);


/**
 * \brief   Functions that runs for the parameters input microseconds before returning
 *
 * \param   microseconds        The number of microseconds to wait
 */
void time_keeper_delay_us(uint64_t microseconds);


/**
 * \brief   Wait for X ms
 *
 * \param   until_time      The time during which the function will run
 */
void time_keeper_delay_ms(uint64_t milliseconds);


/**
 * \brief   Sleep for X ms
 *
 * \param   until_time      The time during which the function will run
 */
void time_keeper_sleep_us(uint64_t microseconds);


#ifdef __cplusplus
}
#endif

#endif /* TIME_KEEPER_HPP_ */
