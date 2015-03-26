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
 * \file pwm_servos.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 * \author Geraud L'Eplattenier
 * 
 * \brief This file is the driver for pwm servos
 *
 ******************************************************************************/


#include "pwm_servos.h"
#include "print_util.h"
#include "time_keeper.h"
#include <math.h>

#include <stdint.h>
#include <stdbool.h>

const uint32_t servo_timer_freq 	 = 1000000;		///< Timer frequency for the servos
const uint16_t servo_center_pulse_us = 1500;		///< Pulse width in microseconds for neutral servo position
const uint16_t servo_magnitude 		 = 500;			///< Amplitude of variation of the pulse width

bool use_servos_7_8;

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Output a PWM on one channel
 *
 * \param	channel			Corresponding channel
 * \param	pulse_us_a		Pulse a in micro sec
 * \param	pulse_us_b		Pulse b in micro sec
 * \param	frequency		Frequency in Hz
 */
void write_channels(int32_t channel, int32_t pulse_us_a, int32_t pulse_us_b, uint16_t frequency);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void write_channels(int32_t channel, int32_t pulse_us_a, int32_t pulse_us_b, uint16_t frequency)
{
	;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void pwm_servos_init(bool use_servos_7_8_param)
{
	use_servos_7_8 = use_servos_7_8_param;
}


void pwm_servos_write_to_hardware(const servos_t* servos)
{
	;
}

void pwm_servos_calibrate_esc(const servos_t* servos)
{
	;
}