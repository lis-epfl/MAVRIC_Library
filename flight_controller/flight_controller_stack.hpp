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
 * \file   flight_controller_stack.hpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 * \author Julien Lecoeur
 *
 * \brief  Base class for cascade style controller hierarchy
 *
 ******************************************************************************/


#ifndef FLIGHT_CONTROLLER_STACK_HPP_
#define FLIGHT_CONTROLLER_STACK_HPP_

#include "control/control_command.hpp"
#include "control/controller.hpp"
#include "flight_controller/flight_controller.hpp"
#include "control/servos_mix.hpp"

/**
 * \brief  Base class for cascade style controller hierarchy
 */
class Flight_controller_stack: public Flight_controller
{
public:
    typedef Controller< position_command_t, velocity_command_t>                         pos_ctrl_t;  ///< Alias for required type of position controller
    typedef Controller_1_to_2<velocity_command_t, attitude_command_t, thrust_command_t> vel_ctrl_t;  ///< Alias for required type of velocity controller
    typedef Controller<attitude_command_t, rate_command_t>                              att_ctrl_t;  ///< Alias for required type of attitude controller
    typedef Controller<rate_command_t, torque_command_t>                                rate_ctrl_t; ///< Alias for required type of rate controller
    typedef Servos_mix                                                                  mix_ctrl_t;  ///< Alias for required type of servo mixing controller

    /**
     * \brief Constructor
     *
     * \param   pos_ctrl    Reference to the position controller
     * \param   vel_ctrl    Reference to the velocity controller
     * \param   att_ctrl    Reference to the attitude controller
     * \param   rate_ctrl   Reference to the rate controller
     * \param   mix_ctrl    Reference to the mixing controller
     */
    Flight_controller_stack(pos_ctrl_t& pos_ctrl,
                            vel_ctrl_t& vel_ctrl,
                            att_ctrl_t& att_ctrl,
                            rate_ctrl_t& rate_ctrl,
                            mix_ctrl_t& mix_ctrl);


    /**
     * \brief   Main update function
     *
     * \detail   Updates recursively the layers of control. The highest active layer is determined
     *          by the type of the last command
     */
    virtual bool update(void);


    /**
     * \brief   Sets actuators in safety mode
     */
    bool failsafe(void);


    /**
     * \brief   Set of command setters
     *
     * \detail  Based on the type of the provided command, the control mode will be changed and used
     *          in following calls to update
     */
    bool set_command(const position_command_t& position);
    bool set_command(const velocity_command_t& velocity);
    bool set_command(const attitude_command_t& attitude);
    bool set_command(const rate_command_t& rate);
    bool set_command(const torque_command_t& torque);
    bool set_command(const thrust_command_t& thrust);

    /**
     * \brief   Set of command getters
     */
    bool get_command(position_command_t& position) const;
    bool get_command(velocity_command_t& velocity) const;
    bool get_command(attitude_command_t& attitude) const;
    bool get_command(rate_command_t& rate) const;
    bool get_command(torque_command_t& torque) const;
    bool get_command(thrust_command_t& thrust) const;
    bool get_output(empty_command_t& command) const;

protected:
    /**
     * \brief   Main update function when in thrust + torque mode
     */
    bool update_in_thrust_and_torque_mode(void);

    /**
     * \brief   Main update function when in thrust + rate mode
     */
    bool update_in_thrust_and_rate_mode(void);

    /**
     * \brief   Main update function when in thrust + attitude mode
     */
    bool update_in_thrust_and_attitude_mode(void);

    /**
     * \brief   Main update function when in velocity mode
     */
    bool update_in_velocity_mode(void);

    /**
     * \brief   Main update function when in position mode
     */
    bool update_in_position_mode(void);


protected:
    command_t command_;      ///< Flight commands

private:
    pos_ctrl_t&  pos_ctrl_;  ///< Reference to controller in position
    vel_ctrl_t&  vel_ctrl_;  ///< Reference to controller in velocity
    att_ctrl_t&  att_ctrl_;  ///< Reference to controller in attitude
    rate_ctrl_t& rate_ctrl_; ///< Reference to controller in rate
    mix_ctrl_t&  mix_ctrl_;  ///< Reference to servo mixing
};

#endif /* FLIGHT_CONTROLLER_STACK_HPP_ */
