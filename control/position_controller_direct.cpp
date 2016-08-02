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
 * \file position_control_direct.cpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 *
 * \brief Basic controller for position and yaw
 *
 ******************************************************************************/

#include "control/position_controller_direct.hpp"

Position_controller_direct::Position_controller_direct(control_command_t& vel_command_lf, const INS& ins, const quat_t& qe, const Position_controller_direct::conf_t& config) :
    Position_controller(vel_command_lf, ins, qe),
    cruise_mode_(false),
    cruise_pid_config_(config.cruise_pid_config),
    hover_pid_config_(config.hover_pid_config),
    max_climb_rate_(config.max_climb_rate),
    max_rel_yaw_(config.max_rel_yaw),
    min_cruise_dist_(config.min_cruise_dist)
{
    yaw_command_lf_ = coord_conventions_get_yaw(qe_);
    pid_controller_init(&pid_controller_, &hover_pid_config_);
}


void Position_controller_direct::update()
{
    /* update rel_goal_pos_, goal_distance_ and dir_desired_sg_ */
    update_internal();

    /* set cruise mode in any case since parameters of config might change */
    bool cruise_mode_new = goal_distance_ > min_cruise_dist_;
    set_cruise_mode(cruise_mode_new);

    /* get desired speed from pid controller */
    float v_desired = pid_controller_update(&pid_controller_, goal_distance_);
    // make sure maximal climb rate is not exceeded, reduce v_desired if necessary
    if (v_desired *  maths_f_abs(dir_desired_sg_[Z]) > max_climb_rate_)
    {
        v_desired = max_climb_rate_ / maths_f_abs(dir_desired_sg_[Z]);
    }

    /* calculate yaw */
    float rel_yaw;
    if(yaw_automatic_)
    {
        /* make drone point towards goal */
        rel_yaw = atan2(dir_desired_sg_[Y], dir_desired_sg_[X]);
    }
    else
    {
        /* use yaw_command_lf_ */
        float yaw_lf = coord_conventions_get_yaw(qe_);
        rel_yaw = maths_calc_smaller_angle(yaw_command_lf_ - yaw_lf);
        if(rel_yaw > max_rel_yaw_)
        {
            rel_yaw = max_rel_yaw_;
        }
    }

    vel_command_lf_.tvel[X] = dir_desired_sg_[X] * v_desired;
    vel_command_lf_.tvel[Y] = dir_desired_sg_[Y] * v_desired;
    vel_command_lf_.tvel[Z] = dir_desired_sg_[Z] * v_desired;
    vel_command_lf_.rpy[YAW] = rel_yaw;
}


void Position_controller_direct::set_command(local_position_t pos_command_lf, float yaw_command_lf)
{
    set_position_command(pos_command_lf);
    set_yaw_command(yaw_command_lf);    
}


void Position_controller_direct::set_yaw_command(float yaw_command_lf)
{
    yaw_automatic_ = false;  // set yaw to manual
    yaw_command_lf_ = yaw_command_lf;
}


void Position_controller_direct::set_yaw_automatic()
{
    yaw_automatic_ = true;
}


void Position_controller_direct::set_cruise_mode(bool cruise_mode)
{
    /* choose pid_config to apply to controller */
    pid_controller_conf_t* pid_config = cruise_mode ? &cruise_pid_config_ : &hover_pid_config_;
    
    /* apply chosen configuration */
    pid_controller_apply_config(&pid_controller_, pid_config);

    cruise_mode_ = cruise_mode;
}

pid_controller_conf_t& Position_controller_direct::cruise_pid_config()
{
    return cruise_pid_config_;
}


pid_controller_conf_t& Position_controller_direct::hover_pid_config()
{
    return hover_pid_config_;
}
