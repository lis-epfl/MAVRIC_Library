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
 * \file position_control.cpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 *
 * \brief Basic controller for position
 *
 ******************************************************************************/

#include "control/position_controller.hpp"

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

template<class TVelocity_controller>
Position_controller<TVelocity_controller>::Position_controller(args_t args, const conf_t& config) :
    TVelocity_controller(args.velocity_controller_args, config.velocity_controller_config),
    ins_(args.ins),
    ahrs_(args.ahrs),
    ctrl_mode_(ctrl_mode_t::POS_XYZ),
    max_climb_rate_(config.max_climb_rate),
    kp_yaw_(config.kp_yaw)
{
    pid_controller_init(&pid_controller_, &config.pid_config);

    pos_command_t pos_command;
    pos_command.pos = std::array<float,3>{{0.0f, 0.0f, 0.0f}};
    set_position_command(pos_command);
}

template<class TVelocity_controller>
void Position_controller<TVelocity_controller>::update()
{
    /* check whether this is the highest active level of the cascade */
    if(TVelocity_controller::cascade_command_ == &position_command_)
    {
        /* calculate torque_command and propagate down the cascade */
        typename TVelocity_controller::vel_command_t vel_command = calc_velocity_command(position_command_);
        TVelocity_controller::update_cascade(vel_command);
    }
    else
    {
        /* propagate update() down the cascade until reaching the highest active level */
        TVelocity_controller::update();
    }
}


template<class TVelocity_controller>
bool Position_controller<TVelocity_controller>::set_position_command(const pos_command_t& pos_command)
{
    position_command_ = pos_command;
    /* set ctrl_mode to use position in xyz as input */
    ctrl_mode_ = ctrl_mode_t::POS_XYZ;
    /* set this as input cascade level */
    TVelocity_controller::cascade_command_ = &position_command_;
    return true;
}


template<class TVelocity_controller>
bool Position_controller<TVelocity_controller>::set_xyposition_zvel_command(const xypos_zvel_command_t& command)
{
    position_command_.pos = std::array<float,3>{{command.pos_x, command.pos_y, 0}};
    zvel_command_ = command.vel_z;
    /* set ctrl_mode to use position in xy and velocity in z as input */
    ctrl_mode_ = ctrl_mode_t::POS_XY_VEL_Z;
    /* set this as input cascade level */
    TVelocity_controller::cascade_command_ = &position_command_;
    return true;   
}


//------------------------------------------------------------------------------
// PROTECTED FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

template<class TVelocity_controller>
void Position_controller<TVelocity_controller>::update_cascade(const pos_command_t& pos_command)
{
    position_command_ = pos_command;
    typename TVelocity_controller::vel_command_t vel_command = calc_velocity_command(position_command_);
    TVelocity_controller::update_cascade(vel_command);
}

template<class TVelocity_controller>
typename TVelocity_controller::vel_command_t Position_controller<TVelocity_controller>::calc_velocity_command(const pos_command_t& pos_command)
{
    /* get current vehicle position in local frame */
    local_position_t local_pos = ins_.position_lf();

    // get position relative to the target position (0 in z dir if z is controlled in velocity)
    float rel_goal_pos[3];
    rel_goal_pos[X] = pos_command.pos[X] - local_pos[X];
    rel_goal_pos[Y] = pos_command.pos[Y] - local_pos[Y];
    rel_goal_pos[Z] = ctrl_mode_ == ctrl_mode_t::POS_XY_VEL_Z ?  0.0f : pos_command.pos[Z] - local_pos[Z];

    // Normalize relative goal position to get desired direction (unit vector)
    float goal_distance = vectors_norm(rel_goal_pos);
    float scale = maths_f_min(1.0f/goal_distance, 2000.0f); // avoid division by 0
    float goal_dir[3];
    goal_dir[X] = rel_goal_pos[X] * scale;
    goal_dir[Y] = rel_goal_pos[Y] * scale;
    goal_dir[Z] = rel_goal_pos[Z] * scale;

    /* get desired speed from pid controller */
    float v_desired = pid_controller_update(&pid_controller_, goal_distance);
    // make sure maximal climb rate is not exceeded, reduce v_desired if necessary
    if (v_desired *  maths_f_abs(goal_dir[Z]) > max_climb_rate_)
    {
        v_desired = max_climb_rate_ / maths_f_abs(goal_dir[Z]);
    }

    quat_t q_rot;
    aero_attitude_t attitude_yaw;
    attitude_yaw = coord_conventions_quat_to_aero(ahrs_.qe);
    attitude_yaw.rpy[0] = 0.0f;
    attitude_yaw.rpy[1] = 0.0f;
    attitude_yaw.rpy[2] = -attitude_yaw.rpy[2];
    q_rot = coord_conventions_quaternion_from_aero(attitude_yaw);
	
	typename TVelocity_controller::vel_command_t velocity_command;
    velocity_command.vel[X] = goal_dir[X] * v_desired;
    velocity_command.vel[Y] = goal_dir[Y] * v_desired;
    velocity_command.vel[Z] = ctrl_mode_ == ctrl_mode_t::POS_XY_VEL_Z ? zvel_command_ : goal_dir[Z] * v_desired;

    // if in cruise_mode: calculate heading towards goal; else leave yaw command as is
    float rel_heading;
    rel_heading = maths_calc_smaller_angle(atan2(goal_dir[Y],goal_dir[X]) - coord_conventions_get_yaw(ahrs_.qe));
    velocity_command.yaw = kp_yaw_ * rel_heading;

	return velocity_command;
}