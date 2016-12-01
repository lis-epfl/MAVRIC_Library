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
 * \file velocity_controller_holonomic.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur

 *
 * \brief A velocity controller for holonomic hovering platform capable of generating 3D thrust
 *
 * \details It takes a velocity command as input and computes an attitude
 * command and thrust command as output.
 *
 ******************************************************************************/


#ifndef VELOCITY_CONTROLLER_HOLONOMIC_HPP_
#define VELOCITY_CONTROLLER_HOLONOMIC_HPP_

#include "control/velocity_controller_copter.hpp"

/**
 * \brief Velocity controller for hovering platforms
 */
class Velocity_controller_holonomic : public Velocity_controller_copter
{
public:
    /**
    * \brief   Default Configuration
    *
    * /return  config
    */
    static inline conf_t default_config(void);

    /**
     * \brief                       Constructor
     *
     * \param   ahrs                Reference to estimated attitude
     * \param   ins                 Reference to estimated speed and position
     * \param   config              Configuration
     */
    Velocity_controller_holonomic(const args_t& args, const conf_t& config = default_config());


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
    virtual bool compute_attitude_and_thrust_from_desired_accel(const std::array<float,3>& accel_command,
                                                                attitude_command_t& attitude_command,
                                                                thrust_command_t& thrust_command);

protected:
    uint32_t use_3d_thrust_;         ///< Boolean indicating if 3D thrust is generated instead of banking
};


Velocity_controller_holonomic::conf_t Velocity_controller_holonomic::default_config(void)
{
    conf_t conf = Velocity_controller_copter::default_config();

    return conf;
}

#endif /* VELOCITY_CONTROLLER_HOLONOMIC_HPP_ */
