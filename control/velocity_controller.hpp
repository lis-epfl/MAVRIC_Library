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
 * \file velocity_controller.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief  Velocity controller base class
 *
 ******************************************************************************/


#ifndef VELOCITY_CONTROLLER_HPP_
#define VELOCITY_CONTROLLER_HPP_

#include "control/controller.hpp"
#include "control/pid_controller.hpp"
#include "sensing/ahrs.hpp"
#include "sensing/ins.hpp"

/**
 * \brief Velocity controller for hovering platforms
 */
class Velocity_controller : public Controller_1_to_2<velocity_command_t, attitude_command_t, thrust_command_t>
{
public:

    /**
     * \brief Velocity controller configuration
     */
    struct conf_t
    {
        pid_controller_conf_t   pid_config[3];          ///< Config for PID controller on velocity along X, Y and Z in global frame
    };


    /**
     * \brief   Required arguments
     */
    struct args_t
    {
        const ahrs_t&                ahrs;                      ///< Pointer to attitude estimation (input)
        const INS&                   ins;                       ///< Speed and position estimation (input)
        velocity_command_t&          velocity_command;          ///< Velocity command (input)
        attitude_command_t&          attitude_command;          ///< Attitude command (output)
        thrust_command_t&            thrust_command;            ///< Thrust command (output)
    };


    /**
     * \brief                 Constructor
     *
     * \param   args          Required arguments
     * \param   config        Configuration
     */
    Velocity_controller(const args_t& args, const conf_t& config);


    /**
     * \brief   Main update function
     *
     * \return  success
     */
    bool update(void);


    /**
     * \brief   Sets the input command
     *
     * \param   command   Input command
     *
     * \return  success
     */
    bool set_command(const velocity_command_t& vel);


    /**
     * \brief   Returns the input command
     *
     * \param   command   Input command
     *
     * \return  success
     */
    bool get_command(velocity_command_t& vel) const;


    /**
     * \brief   Returns the output command
     *
     * \param   command   output command
     *
     * \return  success
     */
    bool get_output(attitude_command_t& att) const;


    /**
     * \brief   Returns the output command
     *
     * \param   command   output command
     *
     * \return  success
     */
    bool get_output(thrust_command_t& thrust) const;


protected:

    /**
     * \brief   Computes attitude and thrust command based on desired acceleration. Depends on robot dynamics
     *
     * \param   accel_vector        Desired acceleration vectors (input)
     * \param   attitude_command    Attitude command (output)
     * \param   thrust_command      Thrust command (output)
     *
     * \return  success
     */
    virtual bool compute_attitude_and_thrust_from_desired_accel(const std::array<float,3>& accel_vector,
                                                                attitude_command_t& attitude_command,
                                                                thrust_command_t& thrust_command            ) = 0;

    const ahrs_t&                ahrs_;                      ///< Pointer to attitude estimation (input)
    const INS&                   ins_;                       ///< Speed and position estimation (input)
    velocity_command_t&          velocity_command_;          ///< Velocity command (input)
    attitude_command_t&          attitude_command_;          ///< Attitude command (output)
    thrust_command_t&            thrust_command_;            ///< Thrust command (output)

    pid_controller_t             pid_[3];                    ///< PID controller for velocity along X, Y and Z in global frame
};

#endif /* VELOCITY_CONTROLLER_HPP_ */
