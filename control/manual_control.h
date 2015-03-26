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
#include "joystick.h"
#include "state.h"
#include "stabilisation.h"


/**
 * \brief	The source mode enum
 */
typedef enum
{
	MODE_SOURCE_GND_STATION  = 0,
	MODE_SOURCE_REMOTE 		= 1,
	MODE_SOURCE_JOYSTICK 		= 2,
} mode_source_t;


/**
 * \brief 	Control source
 */
typedef enum
{
	CONTROL_SOURCE_NONE 			= 0,
	CONTROL_SOURCE_REMOTE 		= 1,
	CONTROL_SOURCE_JOYSTICK	 	= 2,
} control_source_t;


/**
 * \brief Configuration for manual control
 */
typedef struct
{
	mode_source_t 		mode_source;		///< The source mode
	control_source_t 	control_source;		///< Flag to tell whether the remote is active or not
} manual_control_conf_t;


/**
 * \brief The manual control structure
 */
typedef struct
{
	mode_source_t 			mode_source;		///< The source mode
	control_source_t 		control_source;		///< Flag to tell whether the remote is active or not

	remote_t	 			remote;				///< The pointer to the remote structure
	joystick_t 				joystick;			///< The pointer to the joystick structure
} manual_control_t;


/**
 * \brief					Initialise the manual control module
 *
 * \param	manual_control	The pointer to the manual control structure
 * \param	config			The pointer to the configuration structure of the module
 * \param	remote_config	The pointer to the remote structure
 * \param	joystick_config	The pointer to the joystick structure
 *
 * \return	True if the init succeed, false otherwise
 */
bool manual_control_init(manual_control_t* manual_control, manual_control_conf_t* config, remote_conf_t* remote_config);


/**
 * \brief	Selects the source input for the attitude command
 * 
 * \param	manual_control	The pointer to the manual control structure
 * \param	controls		The pointer to the command structure that will be executed
 */
void manual_control_get_control_command(manual_control_t* manual_control, control_command_t* controls);


/**
 * \brief	Selects the source input for the velocity command
 * 
 * \param	manual_control	The pointer to the manual control structure
 * \param	controls		The pointer to the command structure that will be executed
 */
void manual_control_get_velocity_vector(manual_control_t* manual_control, control_command_t* controls);


/**
 * \brief	Selects the source input and returns the thrust
 * 
 * \param	manual_control	The pointer to the manual control structure
 *
 * \return 	The value of the thrust depending on the source input
 */
float manual_control_get_thrust(const manual_control_t* manual_control);


/**
 * \brief	Returns the value of the mode from the desired source input
 *
 * \param	manual_control			The pointer to the manual_control structure
 * \param	mode_current			The current mode of the MAV
 *
 * \return	The value of the mode
 */
mav_mode_t manual_control_get_mode_from_source(manual_control_t* manual_control, mav_mode_t mode_current );


/**
 * \brief	Returns the quality of the strength of the remote receiver
 *
 * \param	manual_control			The pointer to the manual_control structure
 *
 * \return	The current status of the remote controller
 */
signal_quality_t manual_control_get_signal_strength(manual_control_t* manual_control);


#ifdef __cplusplus
}
#endif

#endif // MANUAL_CONTROL_H_
