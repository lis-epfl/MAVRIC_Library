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
 * \file manual_control.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of taking the correct input for the control
 * (i.e. the remote or the joystick)
 *
 ******************************************************************************/


#ifndef MANUAL_CONTROL_H_
#define MANUAL_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "remote.h"
#include "joystick_parsing.h"
#include "state.h"
#include "stabilisation.h"

/**
 * \brief The manual control structure
 */
typedef struct
{
	const remote_t* remote;						/// The pointer to the remote structure
	const joystick_parsing_t* joystick;			/// The pointer to the joystick structure
	const state_t* state;						/// The pointer to the state structure
}manual_control_t;

/**
 * \brief					Initialise the manual control module
 *
 * \param	manual_control	The pointer to the manual control structure
 * \param	remote			The pointer to the remote structure
 * \param	joystick		The pointer to the joystick structure
 * \param	state			The pointer to the state structure
 *
 * \return	True if the init succeed, false otherwise
 */
bool manual_control_init(manual_control_t* manual_control, const remote_t* remote, const joystick_parsing_t* joystick, const state_t* state);

/**
 * \brief	Selects the source input for the attitude command
 * 
 * \param	manual_control	The pointer to the manual control structure
 * \param	controls		The pointer to the command structure that will be executed
 */
void manual_control_get_attitude_command(manual_control_t* manual_control, control_command_t* controls);

/**
 * \brief	Selects the source input for the velocity command
 * 
 * \param	manual_control	The pointer to the manual control structure
 * \param	controls		The pointer to the command structure that will be executed
 */
void manual_control_get_velocity_command(manual_control_t* manual_control, control_command_t* controls);

/**
 * \brief	Selects the source input and returns the thrust
 * 
 * \param	manual_control	The pointer to the manual control structure
 *
 * \return 	The value of the thrust depending on the source input
 */
float manual_control_get_thrust(const manual_control_t* manual_control);



#ifdef __cplusplus
}
#endif

#endif // MANUAL_CONTROL_H_
