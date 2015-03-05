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
 * \file state.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Holds the current status of the MAV
 *
 ******************************************************************************/


#ifndef STATE_H_
#define STATE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "mav_modes.h"
#include "analog_monitor.h"
#include "battery.h"
#include <stdbool.h>

/**
 * \brief	The source mode enum
 */
typedef enum
{
	GND_STATION = 1,
	REMOTE = 2,
	JOYSTICK = 3,
}source_mode_t;

/**
 * \brief	The critical behavior enum
 */
typedef enum
{
	CLIMB_TO_SAFE_ALT,									///< First critical behavior
	FLY_TO_HOME_WP,										///< Second critical behavior, comes after CLIMB_TO_SAFE_ALT
	HOME_LAND,											///< Third critical behavior, comes after FLY_TO_HOME_WP
	CRITICAL_LAND										///< Fourth critical behavior
} critical_behavior_enum;

/**
 * \brief	The auto-landing enum
 */
typedef enum
{
	DESCENT_TO_SMALL_ALTITUDE,							///< First auto landing behavior
	DESCENT_TO_GND										///< Second auto landing behavior, comes after DESCENT_TO_SMAL_ALTITUDE
} auto_landing_behavior_t;

typedef enum MAV_MODE_FLAG mav_flag_t;

/**
 * \brief The state structure
 */
typedef struct  
{
	mav_mode_t mav_mode;								///< The value of the MAV mode 
	mav_state_t mav_state;								///< The value of the MAV state
	
	mav_mode_custom_t mav_mode_custom;					///< The value of the custom_mode
	
	int32_t simulation_mode;							///< The value of the simulation_mode (0: real, 1: simulation)
	
	uint8_t autopilot_type;								///< The type of the autopilot (MAV_TYPE enum in common.h)
	uint8_t autopilot_name;								///< The name of the autopilot (MAV_AUTOPILOT enum in common.h)
	
	uint16_t sensor_present;							///< The type of sensors that are present on the autopilot (Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control)
	uint16_t sensor_enabled;							///< The sensors enabled on the autopilot (Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control)
	uint16_t sensor_health;								///< The health of sensors present on the autopilot (Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control)

	source_mode_t source_mode;

	bool nav_plan_active;								///< Flag to tell that a flight plan (min 1 waypoint) is active
	bool in_the_air;									///< Flag to tell whether the vehicle is airborne or not
	bool reset_position;								///< Flag to enable the reset of the position estimation
	
	uint32_t remote_active;								///< Flag to tell whether the remote is active or not
	
	battery_t battery;									///< The battery structure
	
	const analog_monitor_t* analog_monitor;				///< The pointer to the analog monitor structure
} state_t;


/**
 * \brief					Initialize the state of the MAV
 *
 * \param	state			The pointer to the state structure
 * \param	state_config	The pointer to the state configuration structure
 * \param	analog_monitor	The pointer to the analog monitor structure
 *
 * \return	True if the init succeed, false otherwise
 */
bool state_init(state_t *state, state_t* state_config, const analog_monitor_t* analog_monitor);

/**
 * \brief					Makes the switch to active mode
 *
 * \param	state			The pointer to the state structure
 * \param	mav_state		The MAV state
 */
void state_switch_to_active_mode(state_t* state,mav_state_t* mav_state);

#ifdef __cplusplus
}
#endif

#endif //STATE_H_