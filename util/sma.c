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
 * \file sma.c
 * 
 * \author MAV'RIC Team
 * \author Dylan Bourgeois
 *   
 * \brief A Simple Moving Average utility (SMA)
 *
 ******************************************************************************/

#include "sma.h"
#include "print_util.h"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void sma_init(sma_t * sma, uint16_t period)
{
	sma->current_avg = 0;
	sma->nb_samples = 0;
	sma->sum = 0;
	sma->period = SAMPLING_PERIOD;
}

void sma_update(sma_t * sma, int16_t sample)
{
	print_util_dbg_log_value("sma = ", (int32_t)(sma->current_avg), 10); \
	print_util_dbg_print("\r");

	if (sma->nb_samples < SAMPLING_PERIOD)
	{
		sma->buffer[sma->nb_samples] = sample;
		sma->nb_samples += 1;
		sma->sum += sample;
		sma->current_avg = sma->sum / sma->nb_samples;
	} 
	else 
	{
		sma->sum -= sma->buffer[positive_modulo(sma->nb_samples, sma->period)];
		sma->sum += sample;
		sma->current_avg = sma->sum / sma->period;
		sma->buffer[positive_modulo(sma->nb_samples, sma->period)] = sample;
		sma->nb_samples++;
	}

}
