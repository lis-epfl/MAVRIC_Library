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
 * \file joystick_parsing.c
 * 
 * \author MAV'RIC Team
 *   
 * \brief This file is to decode the set manual command message from MAVLink
 *
 ******************************************************************************/


#include "joystick_parsing.h"
#include "print_util.h"
#include "constants.h"
#include "coord_conventions.h"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief						Arming/Disarming the motors when button 1 is pressed
 *
 * \param	joystick_parsing	The pointer to the joystick parsing structure
 * \param	button_1			The button 1 value
 */
static void joystick_parsing_button_1(joystick_parsing_t* joystick_parsing, button_pressed_t button_1);


/**
 * \brief						Do operations when a button is pressed
 *
 * \param	joystick_parsing	The pointer to the joystick parsing structure
 * \param	button				The value of the button pressed
 * \param	mode_flag			The flag mode to be set
 */
static void joystick_parsing_button(joystick_parsing_t* joystick_parsing, button_pressed_t button, mav_flag_mask_t mode_flag);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void joystick_parsing_button_1(joystick_parsing_t* joystick_parsing, button_pressed_t button_1)
{
	if (button_1 == BUTTON_PRESSED)
	{
		if (joystick_parsing->buttons.button_1 == BUTTON_UNPRESSED)
		{
			if (joystick_parsing->mav_mode_desired.ARMED == ARMED_ON)
			{
				print_util_dbg_print("Disarming from joystick\r\n");
				joystick_parsing->mav_mode_desired.ARMED = ARMED_OFF;
				joystick_parsing->arm_action = ARM_ACTION_DISARMING;
			}
			else
			{
				print_util_dbg_print("Arming from joystick\r\n");
				if ((joystick_parsing->mav_mode_desired.byte&0b01011100) == MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
				{
					joystick_parsing->mav_mode_desired.ARMED = ARMED_ON;
					joystick_parsing->arm_action = ARM_ACTION_ARMING;
				}
			}
			joystick_parsing->buttons.button_1 = BUTTON_PRESSED;
		}
	}
	else
	{
		if (button_1 == BUTTON_UNPRESSED )
		{
			if (joystick_parsing->buttons.button_1 == BUTTON_PRESSED)
			{
				joystick_parsing->buttons.button_1 = BUTTON_UNPRESSED;
			}
		}
	}
}


static void joystick_parsing_button(joystick_parsing_t* joystick_parsing, button_pressed_t button, mav_flag_mask_t mode_flag)
{
	if (button == BUTTON_PRESSED)
	{	
		joystick_parsing->mav_mode_desired.byte &= 0b10100011;
		joystick_parsing->mav_mode_desired.byte += mode_flag;	
	}
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool joystick_parsing_init(joystick_parsing_t* joystick_parsing)
{
	bool init_success = true;
	
	//joystick channels init
	joystick_parsing->channels.x = 0.0f;
	joystick_parsing->channels.y = 0.0f;
	joystick_parsing->channels.z = -1.0f;
	joystick_parsing->channels.r = 0.0f;
	
	//joystick buttons init
	joystick_parsing->buttons.button_mask = 0;
	
	joystick_parsing->mav_mode_desired.byte = MAV_MODE_SAFE;
	joystick_parsing->arm_action = ARM_ACTION_NONE;
	
	print_util_dbg_print("Joystick parsing initialised\r");

	return init_success;
}


float joystick_parsing_get_throttle(const joystick_parsing_t* joystick)
{
	return joystick->channels.z;
}


float joystick_parsing_get_roll(const joystick_parsing_t* joystick)
{
	return joystick->channels.y;
}



float joystick_parsing_get_pitch(const joystick_parsing_t* joystick)
{
	return joystick->channels.x;
}


float joystick_parsing_get_yaw(const joystick_parsing_t* joystick)
{
	return joystick->channels.r;
}


mav_mode_t joystick_parsing_get_mode(joystick_parsing_t* joystick, const mav_mode_t current_mode)
{
	mav_mode_t new_mode = current_mode;
	new_mode.byte = (current_mode.byte & 0b10100000) + (joystick->mav_mode_desired.byte & 0b01011111);
	
	if(joystick->arm_action == ARM_ACTION_ARMING)
	{
		new_mode.ARMED = ARMED_ON;
		joystick->arm_action = ARM_ACTION_NONE;
		print_util_dbg_print("Arming in new fct\r\n");
	}else if(joystick->arm_action == ARM_ACTION_DISARMING)
	{
		new_mode.ARMED = ARMED_OFF;
		joystick->arm_action = ARM_ACTION_NONE;
		print_util_dbg_print("Disarming in new fct\r\n");
	}
	
	return new_mode;
}


void joystick_parsing_get_velocity_vector_from_joystick(joystick_parsing_t* joystick_parsing, control_command_t* controls)
{
	controls->tvel[X] = -10.0f 	* joystick_parsing->channels.x 	* MAX_JOYSTICK_RANGE;
	controls->tvel[Y] =  10.0f	* joystick_parsing->channels.y 	* MAX_JOYSTICK_RANGE;
	controls->tvel[Z] = -1.5f	* joystick_parsing->channels.z;
	
	controls->rpy[YAW] = joystick_parsing->channels.r * MAX_JOYSTICK_RANGE;
}


void joystick_parsing_get_attitude_command_from_joystick(joystick_parsing_t* joystick_parsing, control_command_t* controls)
{
	controls->rpy[ROLL] 	= joystick_parsing->channels.y * MAX_JOYSTICK_RANGE;
	controls->rpy[PITCH] 	= joystick_parsing->channels.x * MAX_JOYSTICK_RANGE;
	controls->rpy[YAW] 		= joystick_parsing->channels.r * MAX_JOYSTICK_RANGE;
	controls->thrust 		= joystick_parsing->channels.z;
}


void joystick_parsing_button_mask(joystick_parsing_t* joystick_parsing, uint16_t buttons)
{
	joystick_button_t button_local;
	button_local.button_mask = buttons;
	
	joystick_parsing_button_1(joystick_parsing, button_local.button_1);
	
	joystick_parsing_button(joystick_parsing, button_local.button_2, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED + MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_GUIDED_ENABLED);  // MAV_MODE_POSITION_HOLD
	joystick_parsing_button(joystick_parsing, button_local.button_5, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED + MAV_MODE_FLAG_STABILIZE_ENABLED); 								// MAV_MODE_VELOCITY_CONTROL
	joystick_parsing_button(joystick_parsing, button_local.button_6, MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_GUIDED_ENABLED + MAV_MODE_FLAG_AUTO_ENABLED); 			// MAV_MODE_GPS_NAVIGATION
	joystick_parsing_button(joystick_parsing, button_local.button_3, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED); 																	// MAV_MODE_ATTITUDE_CONTROL
	
	joystick_parsing->buttons.button_mask = buttons;
}


void joystick_parsing_get_torque_command(const joystick_parsing_t* joystick, torque_command_t * command)
{
	command->xyz[ROLL] 	= joystick_parsing_get_roll(joystick);
	command->xyz[PITCH] = joystick_parsing_get_pitch(joystick);
	command->xyz[YAW] 	= joystick_parsing_get_yaw(joystick);
}


void joystick_parsing_get_rate_command(const joystick_parsing_t* joystick, rate_command_t* command)
{
	command->xyz[ROLL] 	= joystick_parsing_get_roll(joystick);
	command->xyz[PITCH] = joystick_parsing_get_pitch(joystick);
	command->xyz[YAW] 	= joystick_parsing_get_yaw(joystick);
}


void joystick_parsing_get_thrust_command(const joystick_parsing_t* joystick, thrust_command_t* command)
{
	command->thrust = joystick_parsing_get_throttle(joystick);
}


void joystick_parsing_get_attitude_command_absolute_yaw(const joystick_parsing_t* joystick, attitude_command_t* command)
{
	command->rpy[ROLL] 	= joystick_parsing_get_roll(joystick);
	command->rpy[PITCH] = joystick_parsing_get_pitch(joystick);
	command->rpy[YAW]  	= joystick_parsing_get_yaw(joystick);
			
	aero_attitude_t attitude;
	attitude.rpy[ROLL] 	= command->rpy[ROLL]; 
	attitude.rpy[PITCH] = command->rpy[PITCH];
	attitude.rpy[YAW] 	= command->rpy[YAW];
	command->quat = coord_conventions_quaternion_from_aero(attitude);
}


void joystick_parsing_get_attitude_command(const joystick_parsing_t* joystick, const float ki_yaw, attitude_command_t* command)
{		
	command->rpy[ROLL] 	= joystick_parsing_get_roll(joystick);
	command->rpy[PITCH] = joystick_parsing_get_pitch(joystick);
	command->rpy[YAW]  	+= ki_yaw * joystick_parsing_get_yaw(joystick);	
		
	aero_attitude_t attitude;
	attitude.rpy[ROLL] 	= command->rpy[ROLL]; 
	attitude.rpy[PITCH] = command->rpy[PITCH];
	attitude.rpy[YAW] 	= command->rpy[YAW];
	command->quat = coord_conventions_quaternion_from_aero(attitude);
}


void joystick_parsing_get_velocity_command(const joystick_parsing_t* joystick, velocity_command_t* command)
{
	command->xyz[X] = -10.0f 	* joystick_parsing_get_pitch(joystick);
	command->xyz[Y] =  10.0f  	* joystick_parsing_get_roll(joystick);
	command->xyz[Z] = -1.5f 	* joystick_parsing_get_throttle(joystick);
}
