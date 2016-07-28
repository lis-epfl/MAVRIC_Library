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
 * \file quad_nonlinear_control.cpp
 *
 * \author MAV'RIC Team
 * \author Pu Bai
 *
 * \brief Stabilization of a quadrotor using nonlinear control law
 *
 ******************************************************************************/


#ifndef QUAD_NONLINEAR_CONTROL_H_
#define QUAD_NONLINEAR_CONTROL_H_

#include "sensing/position_estimation.hpp"
#include "util/matrix.hpp"

extern "C"
{
#include "control/control_command.h"
}

typedef struct
{
    float gain_position;
    float gain_velocity;
    float gain_attitude;
    float gain_angvel;
    float gain_pos_vel;
    float gain_att_agv;

    const Position_estimation* pos_est;                         // The pointer to the position estimation structure
    const ahrs_t* ahrs;                                         // The pointer to the attitude estimation structure
    thrust_command_t* thrust_command;                           // The pointer to the thrust command structure
    torque_command_t* torque_command;                           // The pointer to the torque command structure
} quad_nonlinear_control_struct;

/**
 * \brief                           Initialize module stabilization
 *
 * \param   quad_nonlinear_control      The pointer to the quadrotor nonlinear control structure
 * \param   pos_est                	 	The pointer to the position estimation structure
 * \param   ahrs                    	The pointer to the attitude estimation structure
 * \param   torque_command         	  	The pointer to the torque command values structure
 * \param   thrust_command          	The pointer to the thrust command values structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool quad_nonlinear_control_init(quad_nonlinear_control_struct* quad_nonlinear_control, const Position_estimation* pos_est, const ahrs_t* ahrs, thrust_command_t* thrust, torque_command_t* torque);

/**
 * \brief                           	Main Controller for stabilizing the quadrotor
 *
 * \param   stabilisation_copter    	The quadrotor nonlinear control structure
 */
void quad_nonlinear_control_law(quad_nonlinear_control_struct* quad_nonlinear_control);

/**
 * \brief                           	Covert attitude from quaternion representation into rotation matrix
 *
 * \param   quaternion    				The quadrotor attitude in quaternion form
 * \param   res    						The quadrotor attitude in rotation matrix form
 */
void quat_to_rot(quat_t quaternion, Mat<3,3>& res);

/**
 * \brief   							Convert physical torque into relative form (range from -1 to 1)
 *
 * \param   thrust    					The desired thrust output in newton
 * \param   torque_roll    				The desired torque output for roll in newton meter
 * \param   torque_pitch    			The desired torque output for pitch in newton meter
 * \param   torque_yaw    				The desired torque output for yaw in newton meter
 */
void torque_abs_to_rel(double& thrust, double& torque_roll, double& torque_pitch, double& torque_yaw);

#endif /* QUAD_NONLINEAR_CONTROL_H_ */