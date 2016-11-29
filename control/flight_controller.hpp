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
 * \file flight_controller.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Interface for flight controllers
 *
 ******************************************************************************/

#ifndef FLIGHT_CONTROLLER_HPP_
#define FLIGHT_CONTROLLER_HPP_

#include "control/controller.hpp"
#include "manual_control/manual_control.hpp"
#include "control/flight_command_source.hpp"

/**
 * \brief   Interface for flight controllers
 */
class Flight_controller: public Controller<position_command_t>,
                         public Controller<velocity_command_t>,
                         public Controller<attitude_command_t>,
                         public Controller<rate_command_t>,
                         public Controller<torque_command_t>,
                         public Controller<thrust_command_t>
{
public:
    /**
     * \brief   Main update function
     */
    virtual bool update(void) = 0;

    /**
     * \brief   Sets actuators in safety mode
     */
    virtual bool failsafe(void) = 0;

    /**
     * \brief   Set of command setters
     */
    virtual bool set_command(const position_command_t& command) = 0;
    virtual bool set_command(const velocity_command_t& command) = 0;
    virtual bool set_command(const attitude_command_t& command) = 0;
    virtual bool set_command(const rate_command_t& command) = 0;
    virtual bool set_command(const torque_command_t& command) = 0;
    virtual bool set_command(const thrust_command_t& command) = 0;

    /**
     * \brief   Set command from manual control in rate mode
     *
     * \param   manual_control  Reference to manual_control
     */
    virtual bool set_manual_rate_command(const Manual_control& manual_control) = 0;


    /**
     * \brief   Set command from manual control in attitude mode
     *
     * \param   manual_control  Reference to manual_control
     */
    virtual bool set_manual_attitude_command(const Manual_control& manual_control) = 0;


    /**
     * \brief   Set command from manual control in velocity mode
     *
     * \param   manual_control  Reference to manual_control
     */
    virtual bool set_manual_velocity_command(const Manual_control& manual_control) = 0;


    /**
     * \brief   Set command from autonomous command source
     *
     * \param   command_source  Reference to source of control command
     */
    virtual bool set_flight_command(const Flight_command_source& command_source)
    {
        return command_source.write_flight_command(*this);
    };
};



#endif  // FLIGHT_CONTROLLER_HPP_
