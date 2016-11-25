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
 * \file controller_stack.hpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 * \author Julien Lecoeur
 *
 * \brief  Base class for cascade style controller hierarchy
 *
 ******************************************************************************/


#ifndef CONTROLLER_STACK_HPP_
#define CONTROLLER_STACK_HPP_

#include "control/control_command.hpp"
#include "control/controller.hpp"
#include "control/flight_controller.hpp"

template<typename POS_CTRL,
         typename VEL_CTRL,
         typename ATT_CTRL,
         typename RATE_CTRL,
         typename MIX_CTRL>
class Controller_stack: public Flight_controller
{
public:

    struct args_t
    {
        typename POS_CTRL::args_t pos_args;
        typename VEL_CTRL::args_t vel_args;
        typename ATT_CTRL::args_t att_args;
        typename RATE_CTRL::args_t rate_args;
        typename MIX_CTRL::args_t mix_args;
    };

    struct conf_t
    {
        typename POS_CTRL::conf_t   pos_config;
        typename VEL_CTRL::conf_t   vel_config;
        typename ATT_CTRL::conf_t   att_config;
        typename RATE_CTRL::conf_t  rate_config;
        typename MIX_CTRL::conf_t        mix_config;
    };

    static conf_t default_config(void)
    {
        conf_t conf;

        conf.pos_config  = POS_CTRL::default_config();
        conf.vel_config  = VEL_CTRL::default_config();
        conf.att_config  = ATT_CTRL::default_config();
        conf.rate_config = RATE_CTRL::default_config();
        conf.mix_config  = MIX_CTRL::default_config();

        return conf;
    };

    Controller_stack(const args_t& args,
                     const conf_t& config = default_config()):
        pos_ctrl_(args.pos_args, config.pos_config),
        vel_ctrl_(args.vel_args, config.vel_config),
        att_ctrl_(args.att_args, config.att_config),
        rate_ctrl_(args.rate_args, config.rate_config),
        mix_ctrl_(args.mix_args, config.mix_config)
    {
        command_.mode   = COMMAND_MODE_THRUST_AND_TORQUE;
        command_.thrust = thrust_command_t{{{0.0f, 0.0f, 0.0f}}};
        command_.torque = torque_command_t{{{0.0f, 0.0f, 0.0f}}};
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

    virtual bool update(void)
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


    bool update_in_thrust_and_torque_mode(void)
    {
        // Servo mix
        return mix_ctrl_.update();
    }

    bool update_in_thrust_and_rate_mode(void)
    {
        // Rate control
        bool ret = true;
        ret &= update_cascade_level(rate_ctrl_, mix_ctrl_);
        ret &= mix_ctrl_.update();
        return ret;
    }

    bool update_in_thrust_and_attitude_mode(void)
    {
        // Attitude control
        bool ret = true;
        update_cascade_level(att_ctrl_, rate_ctrl_);
        update_cascade_level(rate_ctrl_, mix_ctrl_);
        ret &= mix_ctrl_.update();
        return ret;
    }

    bool update_in_velocity_mode(void)
    {
        // Velocity control
        bool ret = true;
        ret &= update_cascade_level<VEL_CTRL, ATT_CTRL, MIX_CTRL,
                                    attitude_command_t, thrust_command_t>(vel_ctrl_, att_ctrl_, mix_ctrl_);
        ret &= update_cascade_level(att_ctrl_,  rate_ctrl_);
        ret &= update_cascade_level(rate_ctrl_, mix_ctrl_);
        ret &= mix_ctrl_.update();
        return ret;
    }

    bool update_in_position_mode(void)
    {
        // Position control
        bool ret = true;
        ret &= update_cascade_level(pos_ctrl_,  vel_ctrl_);
        ret &= update_cascade_level<VEL_CTRL, ATT_CTRL, MIX_CTRL,
                                    attitude_command_t, thrust_command_t>(vel_ctrl_, att_ctrl_, mix_ctrl_);
        ret &= update_cascade_level(att_ctrl_,  rate_ctrl_);
        ret &= update_cascade_level(rate_ctrl_, mix_ctrl_);
        ret &= mix_ctrl_.update();
        return ret;
    }

    bool set_command(const position_command_t& position)
    {
        command_.mode = COMMAND_MODE_POSITION;
        return pos_ctrl_.set_command(position);
    };

    bool set_command(const velocity_command_t& velocity)
    {
        command_.mode = COMMAND_MODE_VELOCITY;
        return vel_ctrl_.set_command(velocity);
    };

    bool set_command(const attitude_command_t& attitude)
    {
        command_.mode = COMMAND_MODE_THRUST_AND_ATTITUDE;
        return att_ctrl_.set_command(attitude);
    };

    bool set_command(const rate_command_t& rate)
    {
        command_.mode = COMMAND_MODE_THRUST_AND_RATE;
        return rate_ctrl_.set_command(rate);
    };

    bool set_command(const torque_command_t& torque)
    {
        command_.mode = COMMAND_MODE_THRUST_AND_TORQUE;
        return mix_ctrl_.set_command(torque);
    };

    bool set_command(const thrust_command_t& thrust)
    {
        return mix_ctrl_.set_command(thrust);
    };

    bool get_command(position_command_t& position) const
    {
        return pos_ctrl_.get_command(position);
    };

    bool get_command(velocity_command_t& velocity) const
    {
        return vel_ctrl_.get_command(velocity);
    };

    bool get_command(attitude_command_t& attitude) const
    {
        return att_ctrl_.get_command(attitude);
    };

    bool get_command(rate_command_t& rate) const
    {
        return rate_ctrl_.get_command(rate);
    };

    bool get_command(torque_command_t& torque) const
    {
        return mix_ctrl_.get_command(torque);
    };

    bool get_command(thrust_command_t& thrust) const
    {
        return mix_ctrl_.get_command(thrust);
    };

    bool get_output(empty_command_t& command) const
    {
        return true;
    };

    const command_t& command(void) const
    {
        return command_;
    }

protected:
    POS_CTRL  pos_ctrl_;
    VEL_CTRL  vel_ctrl_;
    ATT_CTRL  att_ctrl_;
    RATE_CTRL rate_ctrl_;
    MIX_CTRL  mix_ctrl_;

    command_t command_;
};


#endif /* CONTROLLER_STACK_HPP_ */
