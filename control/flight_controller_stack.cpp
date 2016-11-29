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
 * \file   flight_controller_stack.cpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 * \author Julien Lecoeur
 *
 * \brief  Base class for cascade style controller hierarchy
 *
 ******************************************************************************/

#include "control/flight_controller_stack.hpp"

Flight_controller_stack::Flight_controller_stack(pos_ctrl_t& pos_ctrl,
                        vel_ctrl_t& vel_ctrl,
                        att_ctrl_t& att_ctrl,
                        rate_ctrl_t& rate_ctrl,
                        mix_ctrl_t& mix_ctrl):
    pos_ctrl_(pos_ctrl),
    vel_ctrl_(vel_ctrl),
    att_ctrl_(att_ctrl),
    rate_ctrl_(rate_ctrl),
    mix_ctrl_(mix_ctrl)
{
    command_.mode     = COMMAND_MODE_THRUST_AND_TORQUE;
    command_.thrust   = thrust_command_t{{{0.0f, 0.0f, 0.0f}}};
    command_.torque   = torque_command_t{{{0.0f, 0.0f, 0.0f}}};
    command_.rate     = rate_command_t{{{0.0f, 0.0f, 0.0f}}};
    command_.attitude = attitude_command_t{1.0f, {0.0f, 0.0f, 0.0f}};
    command_.velocity = velocity_command_t{{0.0f, 0.0f, 0.0f}, 0.0f};
    command_.position = position_command_t{{0.0f, 0.0f, 0.0f}, 0.0f};
};


bool Flight_controller_stack::update(void)
{
    bool ret = true;
    switch (command_.mode)
    {
        case COMMAND_MODE_THRUST_AND_TORQUE:
            ret &= update_in_thrust_and_torque_mode();
        break;

        case COMMAND_MODE_THRUST_AND_RATE:
            ret &= update_in_thrust_and_rate_mode();
        break;

        case COMMAND_MODE_THRUST_AND_ATTITUDE:
            ret &= update_in_thrust_and_attitude_mode();
        break;

        case COMMAND_MODE_VELOCITY:
            ret &= update_in_velocity_mode();
        break;

        case COMMAND_MODE_POSITION:
            ret &= update_in_position_mode();
        break;
    }

    return true;
};


template<typename CTRL1, typename CTRL2, typename MID_COMMAND_T = typename CTRL1::out_command_t>
bool update_cascade_level(CTRL1& ctrl1, CTRL2& ctrl2)
{
    bool ret = true;

    MID_COMMAND_T command;
    ret &= ctrl1.update();
    ret &= ctrl1.get_output(command);
    ret &= ctrl2.set_command(command);

    return ret;
}


template<typename CTRL1,
         typename CTRL2_A, typename CTRL2_B,
         typename MID_COMMAND_A_T = typename CTRL2_A::in_command_t,
         typename MID_COMMAND_B_T = typename CTRL2_B::in_command_t>
bool update_cascade_level(CTRL1& ctrl1, CTRL2_A& ctrl2_a, CTRL2_B& ctrl2_b)
{
    bool ret = true;

    // common part
    ret &= ctrl1.update();

    // A branch
    MID_COMMAND_A_T  a_command;
    ret &= ctrl1.get_output(a_command);
    ret &= ctrl2_a.set_command(a_command);

    // B branch
    MID_COMMAND_B_T  b_command;
    ret &= ctrl1.get_output(b_command);
    ret &= ctrl2_b.set_command(b_command);

    return ret;
}


bool Flight_controller_stack::update_in_thrust_and_torque_mode(void)
{
    // Servo mix
    return mix_ctrl_.update();
}

bool Flight_controller_stack::update_in_thrust_and_rate_mode(void)
{
    // Rate control
    bool ret = true;
    ret &= update_cascade_level(rate_ctrl_, mix_ctrl_);
    ret &= mix_ctrl_.update();
    return ret;
}

bool Flight_controller_stack::update_in_thrust_and_attitude_mode(void)
{
    // Attitude control
    bool ret = true;
    update_cascade_level(att_ctrl_, rate_ctrl_);
    update_cascade_level(rate_ctrl_, mix_ctrl_);
    ret &= mix_ctrl_.update();
    return ret;
}

bool Flight_controller_stack::update_in_velocity_mode(void)
{
    // Velocity control
    bool ret = true;
    ret &= update_cascade_level<vel_ctrl_t, att_ctrl_t, mix_ctrl_t,
                                attitude_command_t, thrust_command_t>(vel_ctrl_, att_ctrl_, mix_ctrl_);
    ret &= update_cascade_level(att_ctrl_,  rate_ctrl_);
    ret &= update_cascade_level(rate_ctrl_, mix_ctrl_);
    ret &= mix_ctrl_.update();
    return ret;
}

bool Flight_controller_stack::update_in_position_mode(void)
{
    // Position control
    bool ret = true;
    ret &= update_cascade_level(pos_ctrl_,  vel_ctrl_);
    ret &= update_cascade_level<vel_ctrl_t, att_ctrl_t, mix_ctrl_t,
                                attitude_command_t, thrust_command_t>(vel_ctrl_, att_ctrl_, mix_ctrl_);
    ret &= update_cascade_level(att_ctrl_,  rate_ctrl_);
    ret &= update_cascade_level(rate_ctrl_, mix_ctrl_);
    ret &= mix_ctrl_.update();
    return ret;
}

bool Flight_controller_stack::failsafe(void)
{
    return mix_ctrl_.failsafe();
}

bool Flight_controller_stack::set_command(const position_command_t& position)
{
    command_.mode = COMMAND_MODE_POSITION;
    return pos_ctrl_.set_command(position);
};

bool Flight_controller_stack::set_command(const velocity_command_t& velocity)
{
    command_.mode = COMMAND_MODE_VELOCITY;
    return vel_ctrl_.set_command(velocity);
};

bool Flight_controller_stack::set_command(const attitude_command_t& attitude)
{
    command_.mode = COMMAND_MODE_THRUST_AND_ATTITUDE;
    return att_ctrl_.set_command(attitude);
};

bool Flight_controller_stack::set_command(const rate_command_t& rate)
{
    command_.mode = COMMAND_MODE_THRUST_AND_RATE;
    return rate_ctrl_.set_command(rate);
};

bool Flight_controller_stack::set_command(const torque_command_t& torque)
{
    command_.mode = COMMAND_MODE_THRUST_AND_TORQUE;
    return mix_ctrl_.set_command(torque);
};

bool Flight_controller_stack::set_command(const thrust_command_t& thrust)
{
    return mix_ctrl_.set_command(thrust);
};

bool Flight_controller_stack::get_command(position_command_t& position) const
{
    return pos_ctrl_.get_command(position);
};

bool Flight_controller_stack::get_command(velocity_command_t& velocity) const
{
    return vel_ctrl_.get_command(velocity);
};

bool Flight_controller_stack::get_command(attitude_command_t& attitude) const
{
    return att_ctrl_.get_command(attitude);
};

bool Flight_controller_stack::get_command(rate_command_t& rate) const
{
    return rate_ctrl_.get_command(rate);
};

bool Flight_controller_stack::get_command(torque_command_t& torque) const
{
    return mix_ctrl_.get_command(torque);
};

bool Flight_controller_stack::get_command(thrust_command_t& thrust) const
{
    return mix_ctrl_.get_command(thrust);
};

bool Flight_controller_stack::get_output(empty_command_t& command) const
{
    return true;
};

const command_t& Flight_controller_stack::command(void) const
{
    return command_;
};
