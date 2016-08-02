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
 * \file position_controller.cpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 *
 * \brief Interface for position controller
 *
 ******************************************************************************/

#include "control/position_controller.hpp"


Position_controller::Position_controller(control_command_t& vel_command, const INS& ins, const quat_t& qe) : 
                        ins_(ins), 
                        qe_(qe),
                        vel_command_lf_(vel_command),
                        pos_command_lf_(ins_.position_lf())
{
    ;
}

void Position_controller::update_internal()
{
    /* get current vehicle position in local frame */
    local_position_t local_pos = ins_.position_lf();

    // get position relative to the target position
    rel_goal_pos_[X] = pos_command_lf_[X] - local_pos[X];
    rel_goal_pos_[Y] = pos_command_lf_[Y] - local_pos[Y];
    rel_goal_pos_[Z] = pos_command_lf_[Z] - local_pos[Z];
    
    // calculate dir_desired in semi local frame (x-axis aligned with vehicle) (turn around -yaw_lf)
    float yaw_lf = coord_conventions_get_yaw(qe_);
    aero_attitude_t attitude_yaw;
    attitude_yaw.rpy[0] = 0.0f;
    attitude_yaw.rpy[1] = 0.0f;
    attitude_yaw.rpy[2] = -yaw_lf;
    quat_t q_rot = coord_conventions_quaternion_from_aero(attitude_yaw);
    quaternions_rotate_vector(q_rot, rel_goal_pos_, dir_desired_sg_);

    // Normalize desired direction to unit vector
    goal_distance_ = vectors_norm(dir_desired_sg_);
    if(goal_distance_ > 0.0005f) // Avoiding division by zero
    {
        dir_desired_sg_[X] /= goal_distance_;
        dir_desired_sg_[Y] /= goal_distance_;
        dir_desired_sg_[Z] /= goal_distance_;
    }else
    {
        dir_desired_sg_[X] /= 0.0005f;
        dir_desired_sg_[Y] /= 0.0005f;
        dir_desired_sg_[Z] /= 0.0005f;
    }

}
