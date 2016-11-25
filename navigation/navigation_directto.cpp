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
 * \file navigation_directto.cpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 * \author Julien Lecoeur
 *
 * \brief   Navigation module allowing to navigated to goal position on direct trajectory
 * \details Inherits from Position_controller, which inherits from a Velocity_controller_I
 *          i.e. can take navigation, position and velocity commands
 *
 ******************************************************************************/

#include "navigation/navigation_directto.hpp"

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


Navigation_directto::Navigation_directto(args_t args, const conf_t& config) :
    ins_(args.ins),
    cruise_pid_config_(config.cruise_pid_config),
    hover_pid_config_(config.hover_pid_config),
    // std_pid_config_(config.position_controller_config.pid_config),
    min_cruise_dist_sqr_(config.min_cruise_dist_sqr)
{
    // nav_command_t nav_command;
    // nav_command.pos = std::array<float,3>{{0.0f, 0.0f, 0.0f}};
    // set_navigation_command(pos_command);
}


bool Navigation_directto::update()
{
    position_command_t pos_command;
    pos_command.xyz = navigation_command_.xyz;

    // calculate distance to goal squared
    const local_position_t& pos = ins_.position_lf();
    const local_position_t& goal_pos = navigation_command_.xyz;
    float dist[3] = {pos[X] - goal_pos[X], pos[Y] - goal_pos[Y], pos[Z] - goal_pos[Z]};
    distance_to_goal_sqr_ = vectors_norm_sqr(dist);

    // adjust PID parameters of position controller depending on distance to goal
    //pid_controller_conf_t pid_config = distance_to_goal_sqr_ > min_cruise_dist_sqr_ ? cruise_pid_config_ : hover_pid_config_;
    //pid_controller_apply_config(&(TPosition_controller::pid_controller_), &pid_config);

    return true;
}


bool Navigation_directto::set_navigation_command(const nav_command_t& command)
{
    navigation_command_ = command;
    return true;
}

bool Navigation_directto::set_goal(const nav_command_t& command)
{
    navigation_command_ = command;
    return true;
}

//------------------------------------------------------------------------------
// PROTECTED FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

pid_controller_conf_t& Navigation_directto::cruise_pid_config()
{
    return cruise_pid_config_;
}



pid_controller_conf_t& Navigation_directto::hover_pid_config()
{
    return hover_pid_config_;
}
