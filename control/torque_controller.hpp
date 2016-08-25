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
 * \file torque_controller.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Nicolas Dousse
 * \author Basil Huber
 *
 * \brief Abstract class that links between torque commands and servos PWM command for quadcopters
 *
 ******************************************************************************/


#ifndef TORQUE_CONTROLLER_HPP_
#define TORQUE_CONTROLLER_HPP_

#include "drivers/servo.hpp"
#include "util/constants.hpp"

#include "control/itorque_controller.hpp"
#include "control/base_cascade_controller.hpp"


class Torque_controller : public Base_cascade_controller, public ITorque_controller
{
public:
    /**
     * \brief Configuration
     */
    struct conf_t
    {
        float       min_thrust;                 ///< Minimum thrust
        float       max_thrust;                 ///< Maximum thrust
    };


    /**
     * \brief                   Constructor
     *
     * \param config            configuration
     */
    Torque_controller(const conf_t& config);

    /*
     * \brief   Write motor commands to servo structure based on torque command
     */
    virtual void update()=0;

    /*
     * \brief   Set torque command and set controller cascade to "torque mode" 
     * \details Sets the torq_command_ and sets cascade_command_ to point to torq_command, signaling that this is the command mode
     *          This function should NOT be called from higher level controllers if they provide a command, use update_cascade instead
     * \param torq_command  torque command to be set and used for motor commands
     */
    inline bool set_torque_command(const torq_command_t& torq_command){torq_command_ = torq_command; cascade_command_ = &torq_command_; return true;};

protected:
    /*
     * \brief   Write motor commands to servo structure based on given torque command
     * \details Sets the internal torq_command_ to the provided one, without modifiying cascade_command_
     *          This function should be called from higher level controllers if they provide a command
     * \param torq_command  torque command to be set and used for motor commands
     */
    inline void update_cascade(const torq_command_t& torq_command){set_torque_command(torq_command); update();};

    float       min_thrust_;                     ///< Minimum thrust
    float       max_thrust_;                     ///< Maximum thrust

    torq_command_t torq_command_;               ///< torque command (desired torque and thrust)
};

#endif
