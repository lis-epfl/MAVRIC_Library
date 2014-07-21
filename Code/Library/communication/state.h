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
 * \file state.h
 *
 *  Place where the state structure is defined
 */


#ifndef STATE_H__
#define STATE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "scheduler.h"
#include "mavlink_communication.h"
#include <stdbool.h>

/**
 * \brief The Hardware-in-the-loop simulation enum typedef
 */
typedef enum
{
	REAL_MODE,												///< HIL off, runs in real mode
	SIMULATION_MODE											///< HIL on, runs in simulation mode
} HIL_mode;

typedef enum
{
	MAV_MODE_PRE = 0,
	MAV_MODE_SAFE = 64,
	MAV_MODE_ATTITUDE_CONTROL = 192,
	MAV_MODE_VELOCITY_CONTROL = 208,
	MAV_MODE_POSITION_HOLD = 216,
	MAV_MODE_GPS_NAVIGATION = 148,
	MAV_MODE_COLLISION_AVOIDANCE = 149	
} mav_mode_t;

typedef enum MAV_MODE_FLAG mav_flag_t;

/**
 * \brief The state structure
 */
typedef struct  
{
	uint8_t mav_mode;						///< The value of the MAV mode (MAV_MODE enum in common.h)
	uint8_t mav_state;							///< The value of the MAV state (MAV_STATE enum in common.h)
		
	uint8_t mav_mode_previous;				///< The value of the MAV mode at previous time step
	uint8_t mav_state_previous;					///< The value of the MAV state at previous time step
		
	HIL_mode simulation_mode;					///< The value of the simulation_mode (0: real, 1: simulation)
	HIL_mode simulation_mode_previous;			///< The value of the simulation_mode at previous time step
	
	uint8_t autopilot_type;
	uint8_t autopilot_name;
} state_structure_t;

/**
 * \brief						Initialise the state of the MAV
 *
 * \param	state_structure		The pointer to the state structure
 * \param	autopilot_type		The pointer to the autopilot type, MAV_TYPE enum
 * \param	autopilot_name		The pointer to the autopilot name, MAV_AUTOPILOT enum
 * \param	mav_state			The pointer to the mav state, MAV_STATE enum
 * \param	mav_mode			The pointer to the mav mode, MAV_MODE enum
 * \param	simu_mode			The pointer to the simulation mode, HIL_Mode enum
 * \param	message_handler		The pointer to the message handler
 */
void state_init(state_structure_t *state_structure, uint8_t autopilot_type, uint8_t autopilot_name, uint8_t mav_state, uint8_t mav_mode, HIL_mode simu_mode, mavlink_message_handler_t *message_handler);

/**
 * \brief	Task to send the mavlink heartbeat message
 * 
 * \return	The status of execution of the task
 */
task_return_t state_send_heartbeat(state_structure_t* state_structure);

/**
 * \brief						Set the state and the mode of the vehicle
 *
 * \param	state_structure		The pointer to the state structure
 * \param	rec					The received mavlink message structure
 */
void state_set_mav_mode(state_structure_t* state_structure, mavlink_received_t* rec);

/**
 * \brief						Test if the mode has the mode flag in it
 *
 * \param	mode				The mode of the vehicle
 * \param	test_flag			The mode flag to test against
 *
 * \return	The boolean value of the test
 */
bool state_test_flag_mode(uint8_t mode, mav_flag_t test_flag);

/**
 * \brief						Enable the mode
 *
 * \param	state_structure		The pointer to the state structure
 * \param	mav_mode_flag		The flag of the MAV mode
 */
void state_enable_mode(state_structure_t *state_structure, mav_flag_t mav_mode_flag);

/**
 * \brief						Disable the mode
 *
 * \param	state_structure		The pointer to the state structure
 * \param	mav_mode_flag		The flag of the MAV mode
 */
void state_disable_mode(state_structure_t *state_structure, mav_flag_t mav_mode_flag);

/**
 * \brief						Test if the mode has the mode flag in it
 *
 * \param	state_structure		The pointer to the state structure
 * \param	mav_mode_flag		The flag of the MAV mode
 *
 * \return	The boolean value of the test
 */
bool state_test_if_in_flag_mode(state_structure_t *state_structure, mav_flag_t mav_mode_flag);

/**
 * \brief						Test if the mode has the mode flag in it
 *
 * \param	state_structure		The pointer to the state structure
 * \param	mav_mode			The MAV mode
 *
 * \return	The boolean value of the test
 */
bool state_test_if_first_time_in_mode(state_structure_t *state_structure, mav_mode_t mav_mode);

/**
 * \brief						Test if the mode has the mode flag in it
 *
 * \param	state_structure		The pointer to the state structure
 * \param	mav_mode			The MAV mode
 */
void state_set_new_mode(state_structure_t *state_structure, mav_mode_t mav_mode);

#ifdef __cplusplus
}
#endif

#endif //STATE_H__