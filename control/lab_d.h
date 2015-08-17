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
 * \file lab_d.d
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This file is the file to modify in lab D
 *
 ******************************************************************************/


#include "stabilisation.h"


/**
 * \brief	Computes the velocity vector command from a joystick input
 *
 * \param	velocity_vector			The velocity vector command
 * \param	joystick_input			The joystick input
 */
void lab_d_velocity_vector(float velocity_vector[3], const float joystick_input[3]);


/**
 * \brief	Computes the velocity error and runs the PID controller
 *
 * \param	rpyt_errors				The array of velocity errors
 * \param	velocity_cmd			The desired velocity vector
 * \param	current_velocity		The current velocity vector
 * \param	stabiliser 				The PID stablisier structure
 * \param	dt 						The time interval between last update and now
 */
void lab_d_run_PID(float rpyt_errors[4], const float[3], const float current_velocity[3], stabiliser_t *stabiliser, float dt);


/**
 * \brief	Computes the velocity vector command from a joystick input
 *
 * \param	rpy_cmd					The roll, pitch and yaw command vector
 * \param	thrust_cmd				The thrust command
 * \param	vel_controller_output 	The output of the velocity controller
 * \param	thrust_output			The thrust output of the velocity controller
 * \param	thrust_hover_point		The thrust hover point
 */
void lab_d_velocity_to_attitude(float rpy_cmd[3], float *thrust_cmd, const float velocity_controller_output[3],const float thrust_output, const float thrust_hover_point);


/**
 * \brief	Computes the velocity vector command from the relative position
 *
 * \param	velocity_vector			The velocity vector command
 * \param	joystick_input			The joystick input
 */
void lab_d_direct_to_navigation(float velocity_vector[3], const float goal_position[3], const float current_position[3]);
