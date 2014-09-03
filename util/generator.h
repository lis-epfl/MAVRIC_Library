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
 * \file generator.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief Analog signal generator
 *
 ******************************************************************************/


#ifndef GENERATOR_H_
#define GENERATOR_H_

#ifdef __cplusplus
extern "C" 
{
#endif

#include <asf.h>

#define GENERATOR_BUFFER_SIZE 512


/**
 * \brief       Initialises the DAC buffer to a triangle
 */
void generator_init_dac_buffer_triangle(void);

 
/**
 * \brief       Initialises the DAC buffer to a sine
 */
void generator_init_dac_buffer_sine(void);


/**
 * \brief       	Generator function to be used as callback with ADC/DAC library
 * 
 * \details     	This function generates a ramp directly without a buffer
 * 					as this is called from an interrupt, it should be very fast and non-blocking
 * 
 * \param index 	Index
 * 
 * \return 			Signal  
 */
int16_t generator_ramp_generator(int32_t index);


/**
 * \brief       	Generator function to be used as callback with ADC/DAC library
 * \details     	This function can generate arbitrary waves based on a lookup buffer
 * 
 * \param index 	Index
 * 
 * \return      	Signal
 */
int16_t generator_arbitrary_generator(int32_t index);


#ifdef __cplusplus
}
#endif

#endif 