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
 * \file stabilisation_wing.cpp
 *
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *
 * \brief This file handles the stabilization of the platform
 *
 ******************************************************************************/


#include "control/stabilisation_wing.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/constants.hpp"

extern "C"
{
#include "util/print_util.h"
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Compute a (global) heading angle corresponding to the given velocity input vector.
 *
 * \param   input_vel   Float array of the velocity command, in global frame
 *
 * \return  (Global) heading angle, from x axis, between -Pi and Pi (radians)
 */
float heading_from_velocity_vector(float *input_vel);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

float heading_from_velocity_vector(float *input_vel)
{
    float heading_global = atan2(input_vel[Y], input_vel[X]);

    return heading_global;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool stabilisation_wing_init(stabilisation_wing_t* stabilisation_wing, const stabilisation_wing_conf_t stabiliser_conf, control_command_t* controls, torque_command_t* torque_command, thrust_command_t* thrust_command, const Imu* imu, const ahrs_t* ahrs, const Position_estimation* pos_est, const Airspeed_analog* airspeed_analog, const Navigation* navigation, const Gps* gps)
{
    bool init_success = true;

    //init dependencies
    stabilisation_wing->stabiliser_stack = stabiliser_conf.stabiliser_stack;
    stabilisation_wing->controls = controls;
    stabilisation_wing->torque_command = torque_command;
    stabilisation_wing->thrust_command = thrust_command;
    stabilisation_wing->imu = imu;
    stabilisation_wing->ahrs = ahrs;
    stabilisation_wing->pos_est = pos_est;
    stabilisation_wing->airspeed_analog = airspeed_analog;
    stabilisation_wing->navigation = navigation;
    stabilisation_wing->gps = gps;
    stabilisation_wing->thrust_apriori  = stabiliser_conf.thrust_apriori;
    stabilisation_wing->pitch_angle_apriori = stabiliser_conf.pitch_angle_apriori;
    stabilisation_wing->pitch_angle_apriori_gain = stabiliser_conf.pitch_angle_apriori_gain;
    stabilisation_wing->max_roll_angle = stabiliser_conf.max_roll_angle;
    stabilisation_wing->take_off_thrust = stabiliser_conf.take_off_thrust;
    stabilisation_wing->take_off_pitch = stabiliser_conf.take_off_pitch;
    stabilisation_wing->landing_pitch = stabiliser_conf.landing_pitch;
    stabilisation_wing->landing_max_roll = stabiliser_conf.landing_max_roll;

    stabilisation_wing->dt_s          = 0.0f;
    stabilisation_wing->last_update_s = time_keeper_get_s();

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

    print_util_dbg_print("[STABILISATION wing] initalised.\r\n");

    return init_success;
}

void stabilisation_wing_cascade_stabilise(stabilisation_wing_t* stabilisation_wing)
{
    float rpyt_errors[4];
    control_command_t input;
    int32_t i;
    float feedforward[4];
    float nav_heading, current_heading, heading_error;
    float gps_speed_global[3], gps_speed_semi_local[3];
    float input_turn_rate;
    float input_roll_angle;
    aero_attitude_t attitude, attitude_yaw;
    quat_t q_rot;
    float airspeed_desired;
    float clipping_factor;

    // Update timing
    float now                         = time_keeper_get_s();
    stabilisation_wing->dt_s          = now - stabilisation_wing->last_update_s;
    stabilisation_wing->last_update_s = now;

    // Get up vector in body frame
    quat_t up = {0.0f, {UPVECTOR_X, UPVECTOR_Y, UPVECTOR_Z}};
    quat_t up_vec = quaternions_global_to_local(stabilisation_wing->ahrs->qe,
                    up);

    // set the controller input
    input= *stabilisation_wing->controls;
    switch (stabilisation_wing->controls->control_mode)
    {
    case VELOCITY_COMMAND_MODE:
        // Get current attitude
        attitude_yaw = coord_conventions_quat_to_aero(stabilisation_wing->ahrs->qe);

        /////////////
        // HEADING //
        /////////////
        // Compute the heading angle corresponding to the given input velocity vector (input from remote/vector field should be in semi-local frame).
        nav_heading = heading_from_velocity_vector(input.tvel);
        // Overwrite command if in remote
        if(stabilisation_wing->controls->yaw_mode == YAW_RELATIVE)
        {
            nav_heading = maths_calc_smaller_angle(input.rpy[YAW] - attitude_yaw.rpy[YAW]);
        }

        // Compute current heading
        gps_speed_global[X] = stabilisation_wing->pos_est->velocity_lf()[0];
        gps_speed_global[Y] = stabilisation_wing->pos_est->velocity_lf()[1];
        gps_speed_global[Z] = stabilisation_wing->pos_est->velocity_lf()[Z];

        // Transform global to semi-local
        attitude_yaw.rpy[0] = 0.0f;
        attitude_yaw.rpy[1] = 0.0f;
        attitude_yaw.rpy[2] = -attitude_yaw.rpy[2];
        q_rot = coord_conventions_quaternion_from_aero(attitude_yaw);
        quaternions_rotate_vector(q_rot, gps_speed_global, gps_speed_semi_local);

        current_heading = heading_from_velocity_vector(gps_speed_semi_local);

        // Compute heading error
        heading_error = maths_calc_smaller_angle(nav_heading - current_heading);


        ///////////////
        // PID INPUT //
        ///////////////
        // Vector field normalize vector in plane x-y to cruise_speed value ==> airspeed should be done only on the x-y composants
        airspeed_desired = sqrtf(input.tvel[X]*input.tvel[X] + input.tvel[Y]*input.tvel[Y]);

        // Compute errors
        rpyt_errors[0] = heading_error;                                                             // Heading
        rpyt_errors[1] = input.tvel[Z] - gps_speed_global[Z];                                       // Vertical speed
        rpyt_errors[2] = 0.0f;
        rpyt_errors[3] = airspeed_desired - stabilisation_wing->airspeed_analog->get_airspeed();    // Airspeed

        // Compute the feedforward
        feedforward[0] = 0.0f;
        feedforward[1] = 0.0f;
        feedforward[2] = 0.0f;
        feedforward[3] = (airspeed_desired - 13.0f)/8.0f + 0.2f;

        // run PID update on all velocity controllers
        stabilisation_run_feedforward(&stabilisation_wing->stabiliser_stack.velocity_stabiliser, stabilisation_wing->dt_s, rpyt_errors, feedforward);


        ////////////////
        // PID OUTPUT //
        ////////////////
        // Get turn rate command and transform it into a roll angle command for next layer
        input_turn_rate = stabilisation_wing->stabiliser_stack.velocity_stabiliser.output.rpy[0];
        // TODO: Fix this in case of bad airspeed readings...
        clipping_factor = stabilisation_wing->max_roll_angle / (PI/2.0f);
        if(clipping_factor == 0.0f)
        {
            input_roll_angle = 0.0f;
        }
        else
        {
            input_roll_angle = clipping_factor * atanf( (1.0f/clipping_factor) * (stabilisation_wing->airspeed_analog->get_airspeed() * input_turn_rate / 9.81f) );
        }

        // Set input for next layer
        input = stabilisation_wing->stabiliser_stack.velocity_stabiliser.output;
        input.rpy[0] = input_roll_angle;
        input.rpy[1] = - stabilisation_wing->stabiliser_stack.velocity_stabiliser.output.rpy[1];
        input.thrust = stabilisation_wing->stabiliser_stack.velocity_stabiliser.output.thrust;

        // Overwrite the commands during different key phases (take-off and landing)
        if(stabilisation_wing->navigation->internal_state() == Navigation::NAV_TAKEOFF)
        {
            // Take-off: fixed 0 roll angle, fixed defined pitch angle and fixed defined constant thrust value.
            input.rpy[0] = 0.0f;
            input.rpy[1] = stabilisation_wing->take_off_pitch;
            input.thrust = stabilisation_wing->take_off_thrust;
        }
        else if(stabilisation_wing->navigation->internal_state() == Navigation::NAV_LANDING)
        {
            // Landing: Limit the roll computed by the velocity layer (navigation), shut down the motor and impose a little pitch down to assure gliding without stall.
            if(input.rpy[0] > stabilisation_wing->landing_max_roll)
            {
                input.rpy[0] = stabilisation_wing->landing_max_roll;
            }
            else if(input.rpy[0] < -stabilisation_wing->landing_max_roll)
            {
                input.rpy[0] = -stabilisation_wing->landing_max_roll;
            }
            input.rpy[1] = stabilisation_wing->landing_pitch;
            input.thrust = -1.0f;
        }

    // -- no break here  - we want to run the lower level modes as well! --

    case ATTITUDE_COMMAND_MODE:
        // Add "a priori on the pitch" to fly horizontally and to compensate for roll angle
        attitude = coord_conventions_quat_to_aero(stabilisation_wing->ahrs->qe);
        input.rpy[1] += stabilisation_wing->pitch_angle_apriori;    // Constant compensation for horizontal
        if(maths_f_abs(attitude.rpy[ROLL]) < PI/2.0f)                       // Compensation for the roll bank angle
        {
            input.rpy[1] += stabilisation_wing->pitch_angle_apriori_gain * maths_f_abs(input.rpy[0]*input.rpy[0]*input.rpy[0]);
        }

        // run absolute attitude_filter controller
        rpyt_errors[0]= sinf(input.rpy[0]) + up_vec.v[1];                               // Roll
        rpyt_errors[1]= sinf(input.rpy[1]) - up_vec.v[0];                               // Pitch
        rpyt_errors[2]= 0.0f;                                                           // Yaw
        rpyt_errors[3]= input.thrust;       // no feedback for thrust at this level

        // run PID update on all attitude_filter controllers
        stabilisation_run(&stabilisation_wing->stabiliser_stack.attitude_stabiliser, stabilisation_wing->dt_s, rpyt_errors);

        // use output of attitude_filter controller to set rate setpoints for rate controller
        input = stabilisation_wing->stabiliser_stack.attitude_stabiliser.output;

    // -- no break here  - we want to run the lower level modes as well! --

    case RATE_COMMAND_MODE: // this level is always run
        // get rate measurements from IMU (filtered angular rates)
        for (i=0; i<3; i++)
        {
            rpyt_errors[i]= input.rpy[i]- stabilisation_wing->ahrs->angular_speed[i];
        }
        rpyt_errors[3] = input.thrust ;  // no feedback for thrust at this level

        // run PID update on all rate controllers
        stabilisation_run(&stabilisation_wing->stabiliser_stack.rate_stabiliser, stabilisation_wing->dt_s, rpyt_errors);
    }

    stabilisation_wing->torque_command->xyz[0] = stabilisation_wing->stabiliser_stack.rate_stabiliser.output.rpy[ROLL];
    stabilisation_wing->torque_command->xyz[1] = stabilisation_wing->stabiliser_stack.rate_stabiliser.output.rpy[PITCH];
    stabilisation_wing->thrust_command->thrust = stabilisation_wing->stabiliser_stack.rate_stabiliser.output.thrust;
}
