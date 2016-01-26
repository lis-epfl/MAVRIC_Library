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
 * \file throw_recovery_state_machine.h
 *
 * \author MAV'RIC Team
 * \author Dylan Bourgeois
 *   
 * \brief Handles transitions between states defined in paper :
 * 
 *		Automatic Re-Initialization and Failure Recovery for Aggressive
 *		Flight with a Monocular Vision-Based Quadrotor
 *		M. Faessler, F. Fontana, C. Forster, D. Scaramuzza
 *		IEEE International Conference on Robotics and Automation (ICRA), Seattle, 2015.
 * 		http://rpg.ifi.uzh.ch/docs/ICRA15_Faessler.pdf
 *
 ******************************************************************************/


#ifndef THROW_RECOVERY_STATE_MACHINE_H_
#define THROW_RECOVERY_STATE_MACHINE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "ahrs.h"
#include "launch_detection.h"
#include "navigation.h"
#include "position_estimation.h"
#include "remote.h"
#include "scheduler.h"
#include "stabilisation.h"
#include "stabilisation_copter.h"
#include "time_keeper.h"


typedef enum 
{
	STATE_IDLE = 0,
	STATE_LAUNCH_DETECTION = 1,
	STATE_ATTITUDE_CONTROL = 2,
	STATE_VERTICAL_VELOCITY = 3,
	STATE_HEIGHT_CONTROL = 4,
	STATE_HORIZONTAL_VELOCITY = 5,
	STATE_POSITION_LOCKING = 6
} state_custom_t;

typedef struct 
{
	state_custom_t state;

	bool enabled; 			///< True : Throw recovery algorithm is enabled
	bool debug;				///< True : Debug mode activated
	bool is_initialised;	///< True : Throw recovery state machine is ready to run

	uint32_t transition_times[6]; 

	stabilisation_copter_conf_t * stabilisation_copter_conf;

	const remote_t * remote;
	launch_detection_t ld;
	const imu_t * imu;
	const ahrs_t * ahrs;
	const position_estimation_t * pos_est;
	stabilisation_copter_t * stabilisation_copter;
 	navigation_t * navigation;
} throw_recovery_state_machine_t;


/**
 * \brief Initialize the state machine
 *
 * \param state_machine				Pointer to the state machine structure
 * \param remote					Pointer to the remote
 * \param imu 						Pointer to the IMU structure
 * \param ahrs 						Pointer to the AHRS structure
 * \param pos_est 					Pointer to the position estimation structure
 * \param stabilisation_copter 		Pointer to the stabilisation copter structure
 * \param navigation 	 			Pointer to the navigation structure
 *
 * \return	True if the init succeed, false otherwise
 */
bool throw_recovery_state_machine_init(	throw_recovery_state_machine_t * state_machine,
										const remote_t * remote,
										const imu_t * imu, 
										const ahrs_t * ahrs, 
										const position_estimation_t * pos_est, 
										stabilisation_copter_t * stabilisation_copter,
										navigation_t * navigation);


/**
 * \brief Update the state machine
 *
 * \param state_machine				Pointer to the state machine structure
 * \param controls					Pointer to the controls to set
 *
 * \return	Returns the result of the task
 */
task_return_t throw_recovery_state_machine_update(throw_recovery_state_machine_t * state_machine, control_command_t * controls);

/**
 * \brief	Resets the state machine and launch detection
 *
 * \param	state_machine			The pointer to the state_machine structure
 */
void throw_recovery_state_machine_reset(throw_recovery_state_machine_t * state_machine);

/**
 * \brief	Returns true if the state machine should be reset
 *
 * \param	state_machine			The pointer to the state_machine structure
 */
bool throw_recovery_state_machine_should_reset(throw_recovery_state_machine_t * state_machine);

/**
 * \brief	Returns true if the switch enabling the state machine is on
 *
 * \param	state_machine			The pointer to the state_machine structure
 */
bool throw_recovery_state_machine_switch_enabled(throw_recovery_state_machine_t * state_machine);

#ifdef __cplusplus
}
#endif

#endif /* THROW_RECOVERY_STATE_MACHINE_H_ */