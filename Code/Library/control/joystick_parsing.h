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
* This file is to decode the set manual command message from mavlink
*/


#ifndef JOYSTICK_PARSING_H__
#define JOYSTICK_PARSING_H__

#ifdef __cplusplus
	extern "C" {
#endif

#include "mavlink_communication.h"
#include "stabilisation.h"

#define MAX_JOYSTICK_RANGE 0.8

/**
 * \brief	The structure for the joystick parsing
 */
typedef struct
{
	control_command_t* controls;									///< The pointer to the controls structure
}joystick_parsing_t;


/** 
 * \brief	Initialisation of the joystick parsing module
 * \param	joystick_parsing		The poiner to the joystick parsing structure
 * \param	controls				The pointer to the control structure
 * \param	mavlink_communication	The pointer to the mavlink communication structure
 */
void joystick_parsing_init(joystick_parsing_t* joystick_parsing, control_command_t* controls, mavlink_communication_t* mavlink_communication);


/** 
 * \brief	Parse received mavlink message in structure
 * \param	joystick_parsing		The poiner to the joystick parsing structure
 * \param	rec						The pointer to the mavlink message received
 */
void joystick_parsing_parse_msg(joystick_parsing_t *joystick_parsing, mavlink_received_t* rec);

/** 
 * \brief	Parse joystick to velocity vector
 * \param	joystick_parsing		The poiner to the joystick parsing structure
 * \param	controls				The pointer to the control structure
 */
void joystick_parsing_get_velocity_vector_from_joystick(joystick_parsing_t* joystick_parsing, control_command_t* controls);

#ifdef __cplusplus
}
#endif

#endif // JOYSTICK_PARSING_H__