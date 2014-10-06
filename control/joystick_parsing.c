/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 
 
/**
 * \file joystick_parsing.c
 *
 * This file is to decode the set manual command message from MAVLink
 */


#include "joystick_parsing.h"
#include "print_util.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief						Arming/Disarming the motors when button 1 is pressed
 *
 * \param	joystick_parsing	The pointer to the joystick parsing structure
 * \param	button_1			The button 1 value
 */
void joystick_parsing_button_1(joystick_parsing_t* joystick_parsing, button_pressed_t button_1);

/**
 * \brief						Do operations when a button is pressed
 *
 * \param	joystick_parsing	The pointer to the joystick parsing structure
 * \param	button				The value of the button pressed
 * \param	mode_flag			The flag mode to be set
 */
void joystick_parsing_button(joystick_parsing_t* joystick_parsing, button_pressed_t button, mav_flag_mask_t mode_flag);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void joystick_parsing_button_1(joystick_parsing_t* joystick_parsing, button_pressed_t button_1)
{
	if (button_1 == BUTTON_PRESSED)
	{
		if (joystick_parsing->buttons.button_1 == BUTTON_UNPRESSED)
		{
			if (joystick_parsing->state->mav_mode.ARMED == ARMED_ON)
			{
				print_util_dbg_print("Disarming\r");
				joystick_parsing->state->mav_mode.ARMED = ARMED_OFF;
			}
			else
			{
				print_util_dbg_print("Arming\r");
				if ((joystick_parsing->state->mav_mode.byte&0b01011100) == MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
				{
					joystick_parsing->state->mav_mode.ARMED = ARMED_ON;
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

void joystick_parsing_button(joystick_parsing_t* joystick_parsing, button_pressed_t button, mav_flag_mask_t mode_flag)
{
	if (button == BUTTON_PRESSED)
	{
		joystick_parsing->state->mav_mode.byte &= 0b10100011;
		joystick_parsing->state->mav_mode.byte += mode_flag;
	}
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void joystick_parsing_init(joystick_parsing_t* joystick_parsing, control_command_t* controls, state_t* state)
{
	joystick_parsing->controls = controls;
	joystick_parsing->state = state;
	
	joystick_parsing->controls->rpy[ROLL] = 0.0f;
	joystick_parsing->controls->rpy[PITCH] = 0.0f;
	joystick_parsing->controls->rpy[YAW] = 0.0f;
	joystick_parsing->controls->tvel[X] = 0.0f;
	joystick_parsing->controls->tvel[Y] = 0.0f;
	joystick_parsing->controls->tvel[Z] = 0.0f;
	joystick_parsing->controls->theading = 0.0f;
	joystick_parsing->controls->thrust = -1.0f;
	joystick_parsing->controls->control_mode = ATTITUDE_COMMAND_MODE;
	joystick_parsing->controls->yaw_mode = YAW_ABSOLUTE;
	
	joystick_parsing->buttons.button_mask = 0;
	
	
	print_util_dbg_print("Joystick parsing initialised\r");
}

void joystick_parsing_get_velocity_vector_from_joystick(joystick_parsing_t* joystick_parsing, control_command_t* controls)
{
	controls->tvel[X] = - 10.0f * joystick_parsing->controls->rpy[PITCH] * MAX_JOYSTICK_RANGE;
	controls->tvel[Y] = 10.0f * joystick_parsing->controls->rpy[ROLL] * MAX_JOYSTICK_RANGE;
	controls->tvel[Z] = - 1.5f * joystick_parsing->controls->thrust;
	
	controls->rpy[YAW] = joystick_parsing->controls->rpy[YAW] * MAX_JOYSTICK_RANGE;
}

void joystick_parsing_get_attitude_command_from_joystick(joystick_parsing_t* joystick_parsing, control_command_t* controls)
{
	controls->rpy[ROLL] = joystick_parsing->controls->rpy[ROLL] * MAX_JOYSTICK_RANGE;
	controls->rpy[PITCH] = joystick_parsing->controls->rpy[PITCH] * MAX_JOYSTICK_RANGE;
	controls->rpy[YAW] = joystick_parsing->controls->rpy[YAW] * MAX_JOYSTICK_RANGE;
	
	controls->thrust = joystick_parsing->controls->thrust;
}

void joystick_parsing_button_mask(joystick_parsing_t* joystick_parsing, uint16_t buttons)
{
	button_t button_local;
	button_local.button_mask = buttons;
	
	joystick_parsing_button_1(joystick_parsing, button_local.button_1);
	
	joystick_parsing_button(joystick_parsing, button_local.button_2, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED + MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_GUIDED_ENABLED); //MAV_MODE_POSITION_HOLD
	joystick_parsing_button(joystick_parsing, button_local.button_5, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED + MAV_MODE_FLAG_STABILIZE_ENABLED); //MAV_MODE_VELOCITY_CONTROL
	joystick_parsing_button(joystick_parsing, button_local.button_6, MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_GUIDED_ENABLED + MAV_MODE_FLAG_AUTO_ENABLED); //MAV_MODE_GPS_NAVIGATION
	joystick_parsing_button(joystick_parsing, button_local.button_3, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED); //MAV_MODE_ATTITUDE_CONTROL
	
	joystick_parsing->buttons.button_mask = buttons;
	
}