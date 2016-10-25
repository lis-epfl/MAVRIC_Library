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

template<typename POS_CTRL, typename VEL_CTRL, typename ATT_CTRL, typename RATE_CTRL, typename SERVO_MIX>
class Controller_stack: public Controller<position_command_t>,
                        public Controller<velocity_command_t>,
                        public Controller<attitude_command_t>,
                        public Controller<rate_command_t>,
                        public Controller<thrust_command_t>
{
public:

    struct conf_t
    {

    };

    static conf_t default_config(void)
    {
        conf_t config;
        return config;
    };

    bool update(void){return true;};

    // bool set_command(const command_t& command)
    // {
    //     mode_ = command.mode;
    //
    //     switch (command.mode)
    //     {
    //         case COMMAND_MODE_THRUST_AND_TORQUE:
    //             set_command(command.thrust);
    //             set_command(command.torque);
    //         break;
    //
    //         case COMMAND_MODE_THRUST_AND_RATE:
    //             set_command(command.thrust);
    //             set_command(command.rate);
    //         break;
    //
    //         case COMMAND_MODE_THRUST_AND_ATTITUDE:
    //             set_command(command.thrust);
    //             set_command(command.attitude);
    //         break;
    //
    //         case COMMAND_MODE_VELOCITY:
    //             set_command(command.velocity);
    //         break;
    //
    //         case COMMAND_MODE_POSITION:
    //             set_command(command.position);
    //         break;
    //     }
    //
    //     return true;
    // };

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
    control_command_mode_t mode_;
};

#endif /* CONTROLLER_STACK_HPP_ */
