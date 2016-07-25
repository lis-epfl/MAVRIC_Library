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
 * \file stabilisation_copter.cpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Nicolas Dousse
 *
 * \brief This file handles the stabilization of the platform
 *
 ******************************************************************************/


#include "control/stabilisation_copter.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/constants.hpp"

extern "C"
{
#include "util/print_util.h"
}


bool stabilisation_copter_init(stabilisation_copter_t* stabilisation_copter, const stabilisation_copter_conf_t stabiliser_conf, control_command_t* controls, const ahrs_t* ahrs, const INS* ins, torque_command_t* torque, thrust_command_t* thrust)
{
    bool init_success = true;

    //init dependencies
    stabilisation_copter->stabiliser_stack = stabiliser_conf.stabiliser_stack;
    stabilisation_copter->motor_layout = stabiliser_conf.motor_layout;
    stabilisation_copter->controls = controls;
    stabilisation_copter->ahrs = ahrs;
    stabilisation_copter->ins = ins;
    stabilisation_copter->torque_command = torque;
    stabilisation_copter->thrust_command = thrust;

    //init controller
    controls->control_mode = ATTITUDE_COMMAND_MODE;
    controls->yaw_mode = YAW_RELATIVE;

    controls->rpy[ROLL] = 0.0f;
    controls->rpy[PITCH] = 0.0f;
    controls->rpy[YAW] = 0.0f;
    controls->tvel[X] = 0.0f;
    controls->tvel[Y] = 0.0f;
    controls->tvel[Z] = 0.0f;
    controls->theading = 0.0f;
    controls->thrust = -1.0f;

    stabilisation_copter->thrust_hover_point = stabiliser_conf.thrust_hover_point;
    stabilisation_copter->dt_s          = 0.0f;
    stabilisation_copter->last_update_s = time_keeper_get_s();

    return init_success;
}


void stabilisation_copter_cascade_stabilise(stabilisation_copter_t* stabilisation_copter)
{
    float rpyt_errors[4];
    control_command_t input;
    int32_t i;
    quat_t qtmp, q_rot;
    aero_attitude_t attitude_yaw_inverse;

    // Update timing
    float now                           = time_keeper_get_s();
    stabilisation_copter->dt_s          = now - stabilisation_copter->last_update_s;
    stabilisation_copter->last_update_s = now;

    // Get up vector in body frame
    quat_t up = {0.0f, {UPVECTOR_X, UPVECTOR_Y, UPVECTOR_Z}};
    quat_t up_vec = quaternions_global_to_local(stabilisation_copter->ahrs->qe,
                    up);

    // Get current velocity
    std::array<float,3> vel_lf = stabilisation_copter->ins->velocity_lf();

    // set the controller input
    input = *stabilisation_copter->controls;
    switch (stabilisation_copter->controls->control_mode)
    {
        case VELOCITY_COMMAND_MODE:

            attitude_yaw_inverse = coord_conventions_quat_to_aero(stabilisation_copter->ahrs->qe);
            attitude_yaw_inverse.rpy[0] = 0.0f;
            attitude_yaw_inverse.rpy[1] = 0.0f;
            attitude_yaw_inverse.rpy[2] = attitude_yaw_inverse.rpy[2];

            //qtmp=quaternions_create_from_vector(input.tvel);
            //quat_t input_global = quaternions_local_to_global(stabilisation_copter->ahrs->qe, qtmp);

            q_rot = coord_conventions_quaternion_from_aero(attitude_yaw_inverse);

            quat_t input_global;
            quaternions_rotate_vector(q_rot, input.tvel, input_global.v);

            input.tvel[X] = input_global.v[X];
            input.tvel[Y] = input_global.v[Y];
            input.tvel[Z] = input_global.v[Z];

            rpyt_errors[X] = input.tvel[X] - vel_lf[X];
            rpyt_errors[Y] = input.tvel[Y] - vel_lf[Y];
            rpyt_errors[3] = -(input.tvel[Z] - vel_lf[Z]);

            if (stabilisation_copter->controls->yaw_mode == YAW_COORDINATED)
            {
                float rel_heading_coordinated;
                if ((maths_f_abs(vel_lf[X]) < 0.001f) && (maths_f_abs(vel_lf[Y]) < 0.001f))
                {
                    rel_heading_coordinated = 0.0f;
                }
                else
                {
                    rel_heading_coordinated = atan2(vel_lf[Y], vel_lf[X]);
                }

                float w = 0.5f * (maths_sigmoid(vectors_norm(vel_lf.data()) - stabilisation_copter->stabiliser_stack.yaw_coordination_velocity) + 1.0f);
                input.rpy[YAW] = (1.0f - w) * input.rpy[YAW] + w * rel_heading_coordinated;
            }

            rpyt_errors[YAW] = input.rpy[YAW];

            // run PID update on all velocity controllers
            stabilisation_run(&stabilisation_copter->stabiliser_stack.velocity_stabiliser, stabilisation_copter->dt_s, rpyt_errors);

            //velocity_stabiliser.output.thrust = maths_f_min(velocity_stabiliser.output.thrust,stabilisation_param.controls->thrust);
            stabilisation_copter->stabiliser_stack.velocity_stabiliser.output.thrust += stabilisation_copter->thrust_hover_point;
            stabilisation_copter->stabiliser_stack.velocity_stabiliser.output.theading = input.theading;
            input = stabilisation_copter->stabiliser_stack.velocity_stabiliser.output;

            qtmp = quaternions_create_from_vector(stabilisation_copter->stabiliser_stack.velocity_stabiliser.output.rpy);
            //quat_t rpy_local = quaternions_global_to_local(stabilisation_copter->ahrs->qe, qtmp);

            quat_t rpy_local;
            quaternions_rotate_vector(quaternions_inverse(q_rot), qtmp.v, rpy_local.v);

            input.rpy[ROLL] = rpy_local.v[Y];
            input.rpy[PITCH] = -rpy_local.v[X];

            // if ((!stabilisation_copter->pos_est->healthy()) || (stabilisation_copter->pos_est->get_fence_violation_state() == Position_estimation::OUTSIDE_FENCE2))
            if (stabilisation_copter->ins->is_healthy(INS::healthy_t::XYZ_VELOCITY) == false)
            {
                input.rpy[ROLL] = 0.0f;
                input.rpy[PITCH] = 0.0f;
            }

        //input.thrust = stabilisation_copter->controls->tvel[Z];

        // -- no break here  - we want to run the lower level modes as well! --

        case ATTITUDE_COMMAND_MODE:
            // run absolute attitude_filter controller
            rpyt_errors[0] = input.rpy[0] + up_vec.v[1];
            rpyt_errors[1] = input.rpy[1] - up_vec.v[0];

            if (stabilisation_copter->controls->yaw_mode == YAW_ABSOLUTE)
            {
                rpyt_errors[2] = maths_calc_smaller_angle(input.theading - coord_conventions_get_yaw(stabilisation_copter->ahrs->qe));
            }
            else
            {
                // relative yaw
                rpyt_errors[2] = input.rpy[2];
            }

            rpyt_errors[3] = input.thrust;      // no feedback for thrust at this level

            // run PID update on all attitude_filter controllers
            stabilisation_run(&stabilisation_copter->stabiliser_stack.attitude_stabiliser, stabilisation_copter->dt_s, rpyt_errors);

            // use output of attitude_filter controller to set rate setpoints for rate controller
            input = stabilisation_copter->stabiliser_stack.attitude_stabiliser.output;

        // -- no break here  - we want to run the lower level modes as well! --

        case RATE_COMMAND_MODE: // this level is always run
            // get rate measurements from ahrs_t (filtered angular rates)
            for (i = 0; i < 3; i++)
            {
                rpyt_errors[i] = input.rpy[i] - stabilisation_copter->ahrs->angular_speed[i];
            }
            rpyt_errors[3] = input.thrust ;  // no feedback for thrust at this level

            // run PID update on all rate controllers
            stabilisation_run(&stabilisation_copter->stabiliser_stack.rate_stabiliser, stabilisation_copter->dt_s, rpyt_errors);
    }

    stabilisation_copter->torque_command->xyz[0] = stabilisation_copter->stabiliser_stack.rate_stabiliser.output.rpy[ROLL];
    stabilisation_copter->torque_command->xyz[1] = stabilisation_copter->stabiliser_stack.rate_stabiliser.output.rpy[PITCH];
    stabilisation_copter->torque_command->xyz[2] = stabilisation_copter->stabiliser_stack.rate_stabiliser.output.rpy[YAW];
    stabilisation_copter->thrust_command->thrust = stabilisation_copter->stabiliser_stack.rate_stabiliser.output.thrust;
}
