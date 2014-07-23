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
#include "analog_monitor.h"
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
	MAV_MODE_GPS_NAVIGATION = 148
} mav_mode_t;

typedef enum MAV_MODE_FLAG mav_flag_t;

/**
 * \brief The state structure
 */
typedef struct  
{
	uint8_t mav_mode;							///< The value of the MAV mode (MAV_MODE enum in common.h)
	uint8_t mav_state;							///< The value of the MAV state (MAV_STATE enum in common.h)
		
	uint8_t mav_mode_previous;					///< The value of the MAV mode at previous time step
		
	HIL_mode simulation_mode;					///< The value of the simulation_mode (0: real, 1: simulation)
	HIL_mode simulation_mode_previous;			///< The value of the simulation_mode at previous time step
	
	uint8_t autopilot_type;						///< The type of the autopilot (MAV_TYPE enum in common.h)
	uint8_t autopilot_name;						///< The name of the autopilot (MAV_AUTOPILOT enum in common.h)
	
	uint16_t sensor_present;					///< The type of sensors that are present on the autopilot (Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control)
	uint16_t sensor_enabled;					///< The sensors enabled on the autopilot (Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control)
	uint16_t sensor_health;						///< The health of sensors present on the autopilot (Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control)
	
	const analog_monitor_t* adc;				///< The pointer to the analog monitor structure
	const mavlink_stream_t* mavlink_stream;			///< Pointer to the mavlin kstream structure
} state_structure_t;


/**
 * \brief						Initialise the state of the MAV
 *
 * \param	state_structure		The pointer to the state structure
 * \param	state_config		The pointer to the state configuration structure
 * \param   mavlink_stream		The pointer to the mavlink stream structure
 * \param	message_handler		The pointer to the message handler
 */
void state_init(state_structure_t *state_structure, state_structure_t* state_config, const analog_monitor_t* adc, mavlink_stream_t* mavlink_stream, mavlink_message_handler_t *message_handler);


/**
 * \brief	Task to send the mavlink heartbeat message
 * 
 * \param	state_structure		The pointer to the state structure
 *
 * \return	The status of execution of the task
 */
task_return_t state_send_heartbeat(state_structure_t* state_structure);


/**
 * \brief	Task to send the mavlink system status message, project specific message!
 * 
 * \param	state_structure		The pointer to the state structure
 *
 * \return	The status of execution of the task
 */
task_return_t state_send_status(state_structure_t* state_structure);


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
bool state_test_if_in_flag_mode(const state_structure_t *state_structure, mav_flag_t mav_mode_flag);


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