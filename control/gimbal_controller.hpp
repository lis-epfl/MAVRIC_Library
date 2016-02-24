/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file gimbal_controller.hpp
 *
 * \author MAV'RIC Team
 * \author Alexandre Cherpillod
 *
 * \brief   Gimbal control management
 *
 ******************************************************************************/


#ifndef GIMBAL_CONTROLLER_H_
#define GIMBAL_CONTROLLER_H_

#include "control/control_command.h"
#include "control/pid_controller.h"
#include "drivers/servo.hpp"

/**
 * \brief Gimbal controller configuration
 */
typedef struct
{
	attitude_command_t   		attitude_command_desired_config;	///< Initial desired attitude command
	attitude_command_t   		attitude_command_range_config[2];  	///< Allowed [min;max] range of the gimbal
	attitude_command_t   		attitude_output_config;  			///< Initial output commands
} gimbal_controller_conf_t;


class Gimbal_controller
{

//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------
private:
	enum RANGE_GIMBAL
	{
	    MIN_RANGE_GIMBAL = 0,              ///< Range of the minimum allowed gimbal angles
	    MAX_RANGE_GIMBAL = 1               ///< Range of the maximum allowed gimabl angles
	};

<<<<<<< HEAD
/**
 * \brief                       Initializes the gimbal controller structure
 *
 * \param   controller          Pointer to data structure
 * \param   config              Pointer to configuration
 */
void gimbal_controller_init(gimbal_controller_t* controller,
								const gimbal_controller_conf_t config);
=======
	attitude_command_t			attitude_command_range[2];	///< Range [min; max] of the attitude commands
	attitude_command_t			attitude_output;			///< Output to PWM (output)
	Servo*						servo_pitch;				///< Gimbal pitch servo
	Servo*						servo_yaw;					///< Gimbal yaw servo
>>>>>>> alex_gimbal


public:
	attitude_command_t			attitude_command_desired;	///< Attitude command (input from head-tracker)

//------------------------------------------------------------------------------
// FUNCTIONS
//------------------------------------------------------------------------------
private:
	/**
	 * \brief                   Sends the output to the servos
	 *
	 * \param   output			Outputs to send
	 * \param	servos			pointer to the servo structure
	 */
	void gimbal_controller_mix_to_servos();

public:
	/**
	 * \brief                       Initializes the gimbal controller structure
	 *
	 * \param   controller          Pointer to data structure
	 * \param   config              Pointer to configuration
	 */
	void gimbal_controller_init(const gimbal_controller_conf_t config,Servo *servo_4,Servo *servo_5);

	/**
	 * \brief                   Main update function - sends gimbal command to two PWM outputs (for pitch and yaw)
	 *
	 * \param   controller      Pointer to data structure
	 */
	bool gimbal_controller_update(Gimbal_controller *not_used);

};

#endif /* GIMBAL_CONTROLLER_H_ */
