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
//
// template<typename POS_CTRL, typename VEL_CTRL, typename ATT_CTRL, typename RATE_CTRL, typename MIX>
// class Controller_stack: public Controller<position_command_t>,
//                         public Controller<velocity_command_t>,
//                         public Controller<attitude_command_t>,
//                         public Controller<rate_command_t>,
//                         public Controller<thrust_command_t>
// {
// public:
//
//     struct conf_t
//     {
//         typename POS_CTRL::conf_t   pos_config;
//         typename VEL_CTRL::conf_t   vel_config;
//         typename ATT_CTRL::conf_t   att_config;
//         typename RATE_CTRL::conf_t  rate_config;
//         typename MIX::conf_t        mix_config;
//     };
//
//
//     static conf_t default_config(void)
//     {
//         conf_t conf;
//
//         conf.pos_config  = POS_CTRL::default_config();
//         conf.vel_config  = VEL_CTRL::default_config();
//         conf.att_config  = ATT_CTRL::default_config();
//         conf.rate_config = RATE_CTRL::default_config();
//         conf.mix_config  = MIX::default_config();
//
//         return conf;
//     };
//
//
//     Controller_stack(typename POS_CTRL::args_t pos_args,
//                      typename VEL_CTRL::args_t vel_args,
//                      typename ATT_CTRL::args_t att_args,
//                      typename RATE_CTRL::args_t rate_args,
//                      typename MIX::args_t mix_args,
//                      conf_t config = default_config()):
//         pos_ctrl_(pos_args, config.pos_config),
//         vel_ctrl_(vel_args, config.vel_config),
//         att_ctrl_(att_args, config.att_config),
//         rate_ctrl_(rate_args, config.rate_config),
//         mix_(mix_args, config.mix_config)
//     {
//         command_.mode   = COMMAND_MODE_THRUST_AND_TORQUE;
//         command_.thrust = {0.0f, 0.0f, 0.0f};
//         // mode_(COMMAND_MODE_THRUST_AND_TORQUE)
//     };
//     //
//     // virtual bool update_rate(void)
//     // {
//     //     torque_command_t torque;
//     //     bool ret = true;
//     //
//     //     ret &= rate_ctrl_.update();
//     //     ret &= rate_ctrl_.get_output(torque);
//     //     ret &= mix_.set_command(torque);
//     //
//     //     return ret;
//     // }
//     //
//     //
//     // virtual bool update_attitude(void)
//     // {
//     //     rate_command_t command;
//     //     bool ret = true;
//     //
//     //     ret &= att_ctrl_.update();
//     //     ret &= att_ctrl_.get_output(command);
//     //     ret &= rate_ctrl_.set_command(command);
//     //
//     //     return ret;
//     // }
//
//     // template<typename CTRL1, typename CTRL2>
//     // bool update_cascade_level(CTRL1& ctrl1, CTRL2& ctrl2)
//     // {
//     //     bool ret = true;
//     //
//     //     typename CTRL2::in_command_t command;
//     //     ret &= ctrl1.update();
//     //     ret &= ctrl1.get_output(command);
//     //     ret &= ctrl2.set_command(command);
//     //
//     //     return ret;
//     // }
//
//
//     virtual bool update(void)
//     {
//         switch (command_.mode)
//         {
//             case COMMAND_MODE_THRUST_AND_TORQUE:
//                 // Servo mix
//                 mix_.update();
//             break;
//
//             case COMMAND_MODE_THRUST_AND_RATE:
//                 // Rate control
//                 // update_cascade_level(rate_ctrl_, mix_);
//                 mix_.update();
//             break;
//
//             case COMMAND_MODE_THRUST_AND_ATTITUDE:
//                 // Attitude control
//                 // update_cascade_level(att_ctrl_,  rate_ctrl_);
//                 // update_cascade_level(rate_ctrl_, mix_);
//                 mix_.update();
//             break;
//
//             case COMMAND_MODE_VELOCITY:
//                 // Velocity control
//                 // update_cascade_level(vel_ctrl_,  att_ctrl_);
//
//                 // thrust_command_t thrust_command;
//                 // vel_ctrl_.get_output(thrust_command);
//                 // mix_.set_command(thrust_command);
//
//                 // update_cascade_level(att_ctrl_,  rate_ctrl_);
//                 // update_cascade_level(rate_ctrl_, mix_);
//                 mix_.update();
//             break;
//
//             case COMMAND_MODE_POSITION:
//                 // Position control
//                 // update_cascade_level(pos_ctrl_,  vel_ctrl_);
//                 // update_cascade_level(vel_ctrl_,  att_ctrl_);
//
//                 // thrust_command_t thrust_command;
//                 // vel_ctrl_.get_output(thrust_command);
//                 // mix_.set_command(thrust_command);
//
//                 // update_cascade_level(att_ctrl_,  rate_ctrl_);
//                 // update_cascade_level(rate_ctrl_, mix_);
//                 mix_.update();
//             break;
//         }
//     };
//
//     bool set_command(const position_command_t& velocity) {return true;};
//     bool set_command(const velocity_command_t& velocity) {return true;};
//     bool set_command(const attitude_command_t& attitude) {return true;};
//     bool set_command(const rate_command_t& rate) {return true;};
//     bool set_command(const thrust_command_t& rate) {return true;};
//
//     bool get_command(position_command_t& velocity) const {return true;};
//     bool get_command(velocity_command_t& velocity) const {return true;};
//     bool get_command(attitude_command_t& attitude) const {return true;};
//     bool get_command(rate_command_t& rate) const {return true;};
//     bool get_command(thrust_command_t& rate) const {return true;};
//
//     bool get_output(empty_command_t& command) const
//     {
//         return true;
//     };
//
// protected:
//     POS_CTRL  pos_ctrl_;
//     VEL_CTRL  vel_ctrl_;
//     ATT_CTRL  att_ctrl_;
//     RATE_CTRL rate_ctrl_;
//     MIX       mix_;
//     // control_command_mode_t mode_;
//     command_t command_;
// };


template<typename POS_CTRL, typename VEL_CTRL, typename ATT_CTRL, typename RATE_CTRL, typename MIX>
class Controller_stack: public Controller<position_command_t>,
                        public Controller<velocity_command_t>,
                        public Controller<attitude_command_t>,
                        public Controller<rate_command_t>,
                        public Controller<thrust_command_t>
{
public:

    struct conf_t
    {
        typename POS_CTRL::conf_t   pos_config;
        typename VEL_CTRL::conf_t   vel_config;
        typename ATT_CTRL::conf_t   att_config;
        typename RATE_CTRL::conf_t  rate_config;
        typename MIX::conf_t        mix_config;
    };


    struct args_t
    {
        typename POS_CTRL::args_t pos_args;
        typename VEL_CTRL::args_t vel_args;
        typename ATT_CTRL::args_t att_args;
        typename RATE_CTRL::args_t rate_args;
        typename MIX::args_t mix_args;
    };

    static conf_t default_config(void)
    {
        conf_t conf;

        conf.pos_config  = POS_CTRL::default_config();
        conf.vel_config  = VEL_CTRL::default_config();
        conf.att_config  = ATT_CTRL::default_config();
        conf.rate_config = RATE_CTRL::default_config();
        conf.mix_config  = MIX::default_config();

        return conf;
    };


    // Controller_stack(const args_t& args,
    //                  const conf_t& config = default_config()):
    //     pos_ctrl_(args.pos_args, config.pos_config),
    //     vel_ctrl_(args.vel_args, config.vel_config),
    //     att_ctrl_(args.att_args, config.att_config),
    //     rate_ctrl_(args.rate_args, config.rate_config),
    //     mix_(args.mix_args, config.mix_config)
    Controller_stack(typename POS_CTRL::args_t pos_args,
                     typename VEL_CTRL::args_t vel_args,
                     typename ATT_CTRL::args_t att_args,
                     typename RATE_CTRL::args_t rate_args,
                     typename MIX::args_t mix_args,
                     const conf_t& config = default_config()):
        pos_ctrl_(pos_args, config.pos_config),
        vel_ctrl_(vel_args, config.vel_config),
        att_ctrl_(att_args, config.att_config),
        rate_ctrl_(rate_args, config.rate_config),
        mix_(mix_args, config.mix_config)
    {
        command_.mode   = COMMAND_MODE_THRUST_AND_TORQUE;
        command_.thrust = {0.0f, 0.0f, 0.0f};
        // mode_(COMMAND_MODE_THRUST_AND_TORQUE)
    };
    //
    // virtual bool update_rate(void)
    // {
    //     torque_command_t torque;
    //     bool ret = true;
    //
    //     ret &= rate_ctrl_.update();
    //     ret &= rate_ctrl_.get_output(torque);
    //     ret &= mix_.set_command(torque);
    //
    //     return ret;
    // }
    //
    //
    // virtual bool update_attitude(void)
    // {
    //     rate_command_t command;
    //     bool ret = true;
    //
    //     ret &= att_ctrl_.update();
    //     ret &= att_ctrl_.get_output(command);
    //     ret &= rate_ctrl_.set_command(command);
    //
    //     return ret;
    // }

    template<typename CTRL1, typename CTRL2>
    bool update_cascade_level(CTRL1& ctrl1, CTRL2& ctrl2)
    {
        bool ret = true;

        typename CTRL1::out_command_t command;
        ret &= ctrl1.update();
        ret &= ctrl1.get_output(command);
        ret &= ctrl2.set_command(command);

        return ret;
    }


    virtual bool update(void)
    {
        switch (command_.mode)
        {
            case COMMAND_MODE_THRUST_AND_TORQUE:
                // Servo mix
                mix_.update();
            break;

            case COMMAND_MODE_THRUST_AND_RATE:
                // Rate control
                // update_cascade_level(rate_ctrl_, mix_);
                mix_.update();
            break;

            case COMMAND_MODE_THRUST_AND_ATTITUDE:
                // Attitude control
                // update_cascade_level(att_ctrl_,  rate_ctrl_);
                // update_cascade_level(rate_ctrl_, mix_);
                mix_.update();
            break;

            case COMMAND_MODE_VELOCITY:
                // Velocity control
                // update_cascade_level(vel_ctrl_,  att_ctrl_);

                // thrust_command_t thrust_command;
                // vel_ctrl_.get_output(thrust_command);
                // mix_.set_command(thrust_command);

                // update_cascade_level(att_ctrl_,  rate_ctrl_);
                // update_cascade_level(rate_ctrl_, mix_);
                mix_.update();
            break;

            case COMMAND_MODE_POSITION:
                // Position control
                // update_cascade_level(pos_ctrl_,  vel_ctrl_);
                // update_cascade_level(vel_ctrl_,  att_ctrl_);

                // thrust_command_t thrust_command;
                // vel_ctrl_.get_output(thrust_command);
                // mix_.set_command(thrust_command);

                // update_cascade_level(att_ctrl_,  rate_ctrl_);
                // update_cascade_level(rate_ctrl_, mix_);
                mix_.update();
            break;
        }
    };

    bool set_command(const position_command_t& velocity) {return true;};
    bool set_command(const velocity_command_t& velocity) {return true;};
    bool set_command(const attitude_command_t& attitude) {return true;};
    bool set_command(const rate_command_t& rate) {return true;};
    bool set_command(const thrust_command_t& rate) {return true;};

    bool get_command(position_command_t& velocity) const {return true;};
    bool get_command(velocity_command_t& velocity) const {return true;};
    bool get_command(attitude_command_t& attitude) const {return true;};
    bool get_command(rate_command_t& rate) const {return true;};
    bool get_command(thrust_command_t& rate) const {return true;};

    bool get_output(empty_command_t& command) const
    {
        return true;
    };

protected:
    POS_CTRL  pos_ctrl_;
    VEL_CTRL  vel_ctrl_;
    ATT_CTRL  att_ctrl_;
    RATE_CTRL rate_ctrl_;
    MIX       mix_;
    // control_command_mode_t mode_;
    command_t command_;
};



// template<typename T_CTRL, typename T_CTRL_NEXT>
// class Cascade_level: public T_CTRL, public T_CTRL_NEXT
// {
// public:
//
// protected:
//     bool update_cascade()
//     {
//
//     };
//
// };








#endif /* CONTROLLER_STACK_HPP_ */
