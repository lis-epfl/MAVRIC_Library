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


#include "time_keeper.h"
#include "ast.h"

#define TK_AST_FREQUENCY 1000000					///< Timer ticks per second (32 bit timer, >1h time-out at 1MHz, >years at 1kHz. We'll go for precision here...)
#define AST_PRESCALER_SETTING 5						///< Log(SOURCE_CLOCK/AST_FREQ)/log(2)-1 when running from PBA (64Mhz), 5 (1Mhz), or 15 (~1khz, not precisely though).


void time_keeper_init()
{
	ast_init_counter(&AVR32_AST, AST_OSC_PB, AST_PRESCALER_SETTING, 0);
	ast_enable(&AVR32_AST);
}


uint32_t time_keeper_get_time_ticks()
{
	//raw timer ticks
	return ast_get_counter_value(&AVR32_AST);
}


double time_keeper_get_time()
{
	// time in seconds since system start
	return time_keeper_ticks_to_seconds(time_keeper_get_time_ticks());
}


uint32_t time_keeper_get_millis()
{
	//milliseconds since system start
	return time_keeper_get_time_ticks() / 1000; /// (TK_AST_FREQUENCY / 1000);
}


uint32_t time_keeper_get_micros()
{
	// microseconds since system start. Will run over after an hour.
	return time_keeper_get_time_ticks() * (1000000 / TK_AST_FREQUENCY);
}


float time_keeper_ticks_to_seconds(uint32_t timer_ticks)
{
	return ((double)timer_ticks / (double)TK_AST_FREQUENCY);
}


void time_keeper_delay_micros(int32_t microseconds)
{
	uint32_t now = time_keeper_get_micros();
	while (time_keeper_get_micros() < now + microseconds);
}


void time_keeper_delay_until(uint32_t until_time)
{
	while (time_keeper_get_micros() < until_time)
	{
		;
	}	
}


void time_keeper_delay_ms(int32_t t) 
{
	uint32_t now = time_keeper_get_micros();
	
	while (time_keeper_get_micros() < now + 1000 * t) 
	{
		;
	}
};