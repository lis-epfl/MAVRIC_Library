/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
* \file joystick_parsing.h
*
* This file is to decode the set manual command message from MAVLink
*/


#ifndef JOYSTICK_PARSING_H__
#define JOYSTICK_PARSING_H__

#ifdef __cplusplus
	extern "C" {
#endif

#include "stabilisation.h"
#include "state.h"

#define MAX_JOYSTICK_RANGE 0.8

typedef enum
{
	BUTTON_UNPRESSED = 0,
	BUTTON_PRESSED = 1,
}button_pressed_t;

/**
 * \brief	The union structure for the bit mask of the joystick buttons
 */
typedef union
{
	uint16_t button_mask;
	// unamed bitfield structure, use to access directly the flags
	struct
	{
		button_pressed_t		button_16	: 1;
		button_pressed_t		button_15	: 1;
		button_pressed_t		button_14	: 1;
		button_pressed_t		button_13	: 1;
		button_pressed_t		button_12	: 1;
		button_pressed_t		button_11	: 1;
		button_pressed_t		button_10	: 1;
		button_pressed_t		button_9	: 1;
		button_pressed_t		button_8	: 1;
		button_pressed_t		button_7	: 1;
		button_pressed_t		button_6	: 1;
		button_pressed_t		button_5	: 1;
		button_pressed_t		button_4	: 1;
		button_pressed_t		button_3	: 1;
		button_pressed_t		button_2	: 1;
		button_pressed_t		button_1	: 1;
	};
	// identical bitfield, but named (useful for initialisation)
	struct
	{
		button_pressed_t		button_16	: 1;
		button_pressed_t		button_15	: 1;
		button_pressed_t		button_14	: 1;
		button_pressed_t		button_13	: 1;
		button_pressed_t		button_12	: 1;
		button_pressed_t		button_11	: 1;
		button_pressed_t		button_10	: 1;
		button_pressed_t		button_9	: 1;
		button_pressed_t		button_8	: 1;
		button_pressed_t		button_7	: 1;
		button_pressed_t		button_6	: 1;
		button_pressed_t		button_5	: 1;
		button_pressed_t		button_4	: 1;
		button_pressed_t		button_3	: 1;
		button_pressed_t		button_2	: 1;
		button_pressed_t		button_1	: 1;
	} button;
}button_t;

/**
 * \brief	The structure for the joystick parsing
 */
typedef struct
{
	button_t buttons;												///< The bit mask of the button pressed
		
	control_command_t* controls;									///< The pointer to the controls structure
	state_t* state;													///< The pointer to the state structure
}joystick_parsing_t;


/** 
 * \brief	Initialisation of the joystick parsing module
 * \param	joystick_parsing		The pointer to the joystick parsing structure
 * \param	controls				The pointer to the control structure
 * \param	state					The pointer to the state structure
 */
void joystick_parsing_init(joystick_parsing_t* joystick_parsing, control_command_t* controls, state_t* state);


/** 
 * \brief	Parse joystick to velocity vector command
 * \param	joystick_parsing		The pointer to the joystick parsing structure
 * \param	controls				The pointer to the control structure
 */
void joystick_parsing_get_velocity_vector_from_joystick(joystick_parsing_t* joystick_parsing, control_command_t* controls);

/** 
 * \brief	Parse joystick to attitude command
 * \param	joystick_parsing		The pointer to the joystick parsing structure
 * \param	controls				The pointer to the control structure
 */
void joystick_parsing_get_attitude_command_from_joystick(joystick_parsing_t* joystick_parsing, control_command_t* controls);

/**
 * \brief						Do operations when buttons are pressed
 *
 * \param	joystick_parsing	The pointer to the joystick parsing structure
 * \param	buttons				The bit mask of the buttons
 */
void joystick_parsing_button_mask(joystick_parsing_t* joystick_parsing, uint16_t buttons);

#ifdef __cplusplus
}
#endif

#endif // JOYSTICK_PARSING_H__