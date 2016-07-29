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
 * \brief Basic controller for position and yaw
 *
 ******************************************************************************/

#include "control/position_controller.hpp"

Position_controller::Position_controller(control_command_t& vel_command_lf, const INS& ins, const quat_t& qe, const Position_controller::conf_t& config) :
    vel_command_lf_(vel_command_lf),
    cruise_mode_(false),
    ins_(ins),
    qe_(qe),
    max_climb_rate_(config.max_climb_rate),
    max_rel_yaw_(config.max_rel_yaw),
    min_cruise_dist_(config.min_cruise_dist),
    cruise_pid_config_(config.cruise_pid_config),
    hover_pid_config_(config.hover_pid_config)
{
    pos_command_lf_ = ins_.position_lf();
    yaw_command_lf_ = coord_conventions_get_yaw(qe_);
    pid_controller_init(&pid_controller_, &hover_pid_config_);
}


void Position_controller::update()
{
    /* get current vehicle position in local frame */
    local_position_t local_pos = ins_.position_lf();

    // get position relative to the target position
    float rel_pos[3];
    rel_pos[X] = pos_command_lf_[X] - local_pos[X];
    rel_pos[Y] = pos_command_lf_[Y] - local_pos[Y];
    rel_pos[Z] = pos_command_lf_[Z] - local_pos[Z];
    
    // calculate dir_desired in semi local frame (x-axis aligned with vehicle) (turn around -yaw_lf)
    float dir_desired_sg[3];
    float yaw_lf = coord_conventions_get_yaw(qe_);
    aero_attitude_t attitude_yaw;
    attitude_yaw.rpy[0] = 0.0f;
    attitude_yaw.rpy[1] = 0.0f;
    attitude_yaw.rpy[2] = -yaw_lf;
    quat_t q_rot = coord_conventions_quaternion_from_aero(attitude_yaw);
    quaternions_rotate_vector(q_rot, rel_pos, dir_desired_sg);

    // Normalize desired direction to unit vector
    float dist = vectors_norm(dir_desired_sg);
    if(dist < 0.0005f) // Avoiding division by zero
    {
        dist = 0.0005f;
    }
    dir_desired_sg[X] /= dist;
    dir_desired_sg[Y] /= dist;
    dir_desired_sg[Z] /= dist;

    /* if we change from cruise_mode to hover_mode or vice-versa, change pid parameters */
    bool cruise_mode_new = dist > min_cruise_dist_;
    if(cruise_mode_new != cruise_mode_)
    {
        set_cruise_mode(cruise_mode_new);
    }

    /* get desired speed from pid controller */
    float v_desired = pid_controller_update(&pid_controller_, dist);
    // make sure maximal climb rate is not exceeded, reduce v_desired if necessary
    if (v_desired *  maths_f_abs(dir_desired_sg[Z]) > max_climb_rate_)
    {
        v_desired = max_climb_rate_ / maths_f_abs(dir_desired_sg[Z]);
    }

    /* calculate relative yaw and clip at rel_yaw */
    float rel_yaw = maths_calc_smaller_angle(yaw_command_lf_ - yaw_lf);
    if(rel_yaw > max_rel_yaw_)
    {
        rel_yaw = max_rel_yaw_;
    }

    vel_command_lf_.tvel[X] = dir_desired_sg[X] * v_desired;
    vel_command_lf_.tvel[Y] = dir_desired_sg[Y] * v_desired;
    vel_command_lf_.tvel[Z] = dir_desired_sg[Z] * v_desired;
    vel_command_lf_.rpy[YAW] = rel_yaw;
}


void Position_controller::set_command(local_position_t pos_command_lf, float yaw_command_lf)
{
    set_position_command(pos_command_lf);
    set_yaw_command(yaw_command_lf);    
}


void Position_controller::set_position_command(local_position_t pos_command_lf)
{
    pos_command_lf_ = pos_command_lf;
}


void Position_controller::set_yaw_command(float yaw_command_lf)
{
    yaw_command_lf_ = yaw_command_lf;
}


void Position_controller::set_cruise_mode(bool cruise_mode)
{
    /* choose pid_config to apply to controller */
    pid_controller_conf_t* pid_config = cruise_mode_ ? &cruise_pid_config_ : &hover_pid_config_;
    
    /* apply chosen configuration */
    pid_controller_apply_config(&pid_controller_, pid_config);
}

