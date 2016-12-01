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

Position_controller::Position_controller(args_t args, const conf_t& config) :
    ins_(args.ins),
    ahrs_(args.ahrs),
    position_command_(position_command_t{{{0.0f, 0.0f, 0.0f}}, 0.0f}),
    velocity_command_(velocity_command_t{{{0.0f, 0.0f, 0.0f}}, 0.0f}),
    cruise_speed_(config.cruise_speed),
    cruise_dist_(config.cruise_dist),
    max_climb_rate_(config.max_climb_rate),
    kp_yaw_(config.kp_yaw)
{
    pid_controller_init(&pid_controller_, &config.pid_config);
}

bool Position_controller::update()
{
    // get current vehicle position in local frame
    local_position_t local_pos = ins_.position_lf();
    float v_current = vectors_norm(ins_.velocity_lf().data());

    // get position relative to the target position (0 in z dir if z is controlled in velocity)
    float rel_goal_pos[3];
    rel_goal_pos[X] = position_command_.xyz[X] - local_pos[X];
    rel_goal_pos[Y] = position_command_.xyz[Y] - local_pos[Y];
    rel_goal_pos[Z] = position_command_.xyz[Z] - local_pos[Z];

    // Normalize relative goal position to get desired direction (unit vector)
    float goal_distance = vectors_norm(rel_goal_pos);
    float scale = maths_f_min(1.0f / goal_distance, 2000.0f); // avoid division by 0
    float goal_dir[3];
    goal_dir[X] = rel_goal_pos[X] * scale;
    goal_dir[Y] = rel_goal_pos[Y] * scale;
    goal_dir[Z] = rel_goal_pos[Z] * scale;

    // get desired speed from pid controller
    float pid_dist     = SQR(cruise_speed_) / (SQR(pid_controller_.p_gain) * cruise_dist_);
    float v_desired;
    if (goal_distance > pid_dist)
    {
        v_desired = maths_clip(cruise_speed_ * maths_fast_sqrt(goal_distance / cruise_dist_), cruise_speed_);

        // do ramp-up or ramp-down
        v_desired = v_current + maths_clip(v_desired - v_current, cruise_speed_ / cruise_dist_);
    }
    else
    {
        v_desired = pid_controller_update(&pid_controller_, goal_distance);
    }

    // make sure maximal climb rate is not exceeded, reduce v_desired if necessary
    if (v_desired * maths_f_abs(goal_dir[Z]) > max_climb_rate_)
    {
        v_desired = max_climb_rate_ / maths_f_abs(goal_dir[Z]);
    }

    // Compute desired velocity command
    velocity_command_.xyz[X]  = goal_dir[X] * v_desired;
    velocity_command_.xyz[Y]  = goal_dir[Y] * v_desired;
    velocity_command_.xyz[Z]  = goal_dir[Z] * v_desired;
    velocity_command_.heading = position_command_.heading;

    // Reduce XY velocity according to yaw error: this gives time to align to next waypoint before accelerating
    float yaw_current = coord_conventions_get_yaw(ahrs_.attitude());
    float yaw_error_factor = 1.0f - maths_f_abs(maths_clip(maths_calc_smaller_angle(velocity_command_.heading - yaw_current), 1.0f));
    velocity_command_.xyz[X] *= yaw_error_factor;
    velocity_command_.xyz[Y] *= yaw_error_factor;

	return true;
}


bool Position_controller::set_command(const position_command_t& pos)
{
    position_command_ = pos;
    return true;
}


bool Position_controller::get_command(position_command_t& pos) const
{
    pos = position_command_;
    return true;
}


bool Position_controller::get_output(velocity_command_t& vel) const
{
    vel = velocity_command_;
    return true;
}


pid_controller_t& Position_controller::get_pid(void)
{
    return pid_controller_;
}
