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
 * \file pwm_servos_avr32.hpp
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 * \author Nicolas Dousse
 * 
 * \brief This file is the driver for pwm servos
 *
 ******************************************************************************/


#ifndef PWM_SERVOS_AVR32_H_
#define PWM_SERVOS_AVR32_H_

#include "pwm_servos.hpp"

extern "C"
{
	#include <stdint.h>
	#include <stdbool.h>
	#include "servos.h"
}

class Pwm_servos_avr32: public Pwm_servos
{
public:

	Pwm_servos_avr32();

	/**
	 * \brief						Initialize the hardware line for servos
	 *
	 * \param use_servos_7_8_param	Definition if the line for servos 7 and 8 is used
	 */
	bool pwm_servos_init(bool use_servos_7_8_param);


	/**
	 * \brief						Set servos' values
	 *
	 * \param servos				Pointer to a structure containing the servos' data
	 */
	void pwm_servos_write_to_hardware(const servos_t* servos);


	/**
	 * \brief						Set speed controller set points 
	 *
	 * \param servos				Pointer to a structure containing the servos' data
	 */
	void pwm_servos_calibrate_esc(const servos_t* servos);

private:

	/**
	 * \brief	Output a PWM on one channel
	 *
	 * \param	channel			Corresponding channel
	 * \param	pulse_us_a		Pulse a in micro sec
	 * \param	pulse_us_b		Pulse b in micro sec
	 * \param	frequency		Frequency in Hz
	 */
	void write_channels(int32_t channel, int32_t pulse_us_a, int32_t pulse_us_b, uint16_t frequency);

	bool use_servos_7_8;


};

#endif /* PWM_SERVOS_AVR32_H_ */
