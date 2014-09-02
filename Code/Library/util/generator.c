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
 * \file generator.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief Analog signal generator
 *
 ******************************************************************************/


#include "generator.h"
#include "math.h"
#include "ads1274.h"


// buffer for DAC output (arbitrary waveform for function generator)
static volatile uint16_t dac_function_buffer[GENERATOR_BUFFER_SIZE];	// TODO: This should not be instanciated by default


void generator_init_dac_buffer_triangle(void) 
{
	int32_t i;
	for (i=0; i<GENERATOR_BUFFER_SIZE/2; i++) 
	{
		dac_function_buffer[i] = (uint16_t)((4095.0f / (float)(GENERATOR_BUFFER_SIZE / 2))*(float)i);
	}

	for (; i<GENERATOR_BUFFER_SIZE; i++) 
	{
		dac_function_buffer[i] = (uint16_t)(4095.0f - (4096.0f / (float)(GENERATOR_BUFFER_SIZE / 2)) * (float)(i-GENERATOR_BUFFER_SIZE / 2));
	}
}


void generator_init_dac_buffer_sine(void) 
{
	int32_t i;
	for (i=0; i<GENERATOR_BUFFER_SIZE; i++) 
	{
		dac_function_buffer[i]=(uint16_t)(4095.0f * (0.5f + 0.5f * sin((float)10 * i * 2.0f * M_PI / (float)GENERATOR_BUFFER_SIZE)));
	}
}


int16_t generator_ramp_generator(int32_t index) 
{
	return index * 4096 / ADC_BUFFER_SIZE;
}

 
int16_t generator_arbitrary_generator(int32_t index) 
{
	return dac_function_buffer[index * GENERATOR_BUFFER_SIZE / ADC_BUFFER_SIZE];
}
