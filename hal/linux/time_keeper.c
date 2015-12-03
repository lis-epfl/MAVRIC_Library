/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file time_keeper.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is used to interact with the clock of the microcontroller
 * 
 ******************************************************************************/
 

#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include "time_keeper.h"

#define TK_AST_FREQUENCY 1000000					///< Timer ticks per second (32 bit timer, >1h time-out at 1MHz, >years at 1kHz. We'll go for precision here...)
#define AST_PRESCALER_SETTING 5						///< Log(SOURCE_CLOCK/AST_FREQ)/log(2)-1 when running from PBA (64Mhz), 5 (1Mhz), or 15 (~1khz, not precisely though).


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	raw timer ticks
 *
 * \return	The raw timer ticks
 */
uint64_t time_keeper_get_s_ticks(void);


/**
 * \brief	Transforms the timer ticks into seconds
 *
 * \param	timer_ticks		The timer ticks
 *
 * \return	The time in seconds
 */
float time_keeper_ticks_to_seconds(uint64_t timer_ticks);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

uint64_t time_keeper_get_s_ticks()
{ 	
	//raw timer ticks
	struct timeval tv;
	
	gettimeofday(&tv,0);
	return tv.tv_sec*1000000+tv.tv_usec;
}


float time_keeper_ticks_to_seconds(uint64_t timer_ticks)
{
	return ((double)timer_ticks / (double)TK_AST_FREQUENCY);
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void time_keeper_init(void) 
{
	;
}


double time_keeper_get_s(void)
{
	// time in seconds since system start
	return time_keeper_ticks_to_seconds(time_keeper_get_s_ticks());
}


uint64_t time_keeper_get_ms(void)
{
	//milliseconds since system start
	return time_keeper_get_s_ticks() / 1000; /// (TK_AST_FREQUENCY / 1000);
}


uint64_t time_keeper_get_us(void)
{
	// microseconds since system start. Will run over after an hour.
	return time_keeper_get_s_ticks() * (1000000 / TK_AST_FREQUENCY);
}


void time_keeper_delay_us(uint64_t microseconds)
{
	uint64_t now = time_keeper_get_us();
	while (time_keeper_get_us() < now + microseconds)
	{
		;
	}
}


void time_keeper_delay_ms(uint64_t milliseconds) 
{
	uint64_t now = time_keeper_get_us();
	
	while (time_keeper_get_us() < now + 1000 * milliseconds) 
	{
		;
	}
}


void time_keeper_sleep_us(uint64_t microseconds) 
{
	struct timespec reqtime;
	reqtime.tv_sec 	= 0;
	reqtime.tv_nsec = 1000 * microseconds;

	nanosleep(&reqtime, NULL);
}