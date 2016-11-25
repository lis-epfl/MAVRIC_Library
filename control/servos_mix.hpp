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
 * \file servos_mix.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Nicolas Dousse
 * \author Basil Huber
 *
 * \brief Abstract class that links between torque commands and servos PWM command for quadcopters
 *
 ******************************************************************************/


#ifndef SERVOS_MIX_HPP_
#define SERVOS_MIX_HPP_

#include "control/controller.hpp"
#include "control/control_command.hpp"
#include "util/coord_conventions.hpp"
#include "drivers/servo.hpp"
#include "util/constants.hpp"


class Servos_mix : public Controller<torque_command_t>,
                   public Controller<thrust_command_t>
{
public:

    /*
     * \brief   Write motor commands to servos based on torque command
     */
    virtual bool update()=0;


    /*
     * \brief   Write failsafe motor commands to servos
     */
    virtual bool failsafe()=0;


    /**
     * \brief           sets the torque command
     *
     * \param command   torque command in body frame
     *
     * \return success  whether command was accepted
     */
    virtual bool set_command(const torque_command_t& torque) = 0;


    /**
     * \brief           sets the thrust command
     *
     * \param command   thrust command in body frame
     *
     * \return success  whether command was accepted
     */
    virtual bool set_command(const thrust_command_t& thrust) = 0;


    /**
     * \brief           sets the torque command
     *
     * \param command   torque command in body frame
     *
     * \return success  whether command was accepted
     */
    virtual bool get_command(torque_command_t& torque) const = 0;


    /**
     * \brief           sets the thrust command
     *
     * \param command   thrust command in body frame
     *
     * \return success  whether command was accepted
     */
    virtual bool get_command(thrust_command_t& thrust) const = 0;


    /**
     * \brief   Returns the output command
     *
     * \param   command   output command
     *
     * \return  success
     */
    virtual bool get_output(empty_command_t& command) const
    {
        return true;
    };
};

#endif
