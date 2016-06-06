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
 * \file servos_mix_6dof.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief Servo mix
 *
 ******************************************************************************/


#ifndef SERVOS_MIX_HPP_
#define SERVOS_MIX_HPP_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/servo.hpp"
#include "control/control_command.h"
#include "util/matrix.hpp"

/**
 * \brief Servos mix
 *
 * \tparam  N   Number of servos
 */
template<uint32_t N>
class Servos_mix_6dof
{
public:
    /**
     * \brief   Configuration structure
     */
    struct conf_t
    {
        float     min_thrust;   ///< Minimum command to apply to the motors
        float     max_thrust;   ///< Maximum command to apply to the motors
        Mat<N,6>  mix;          ///< Mix matrix
    };

    /**
     * \brief   Default configuration
     *
     * \param   sysid       System id (default value = 1)
     *
     * \return  Config structure
     */
    static inline conf_t default_config(void)
    {
        conf_t conf = {};
        conf.min_thrust = -0.95f;
        conf.max_thrust =  1.0f;
        conf.mix        = Mat<N,6>(0.0f);
        return conf;
    };


    /**
     * \brief   Constructor
     */
    Servos_mix_6dof( const torque_command_t& torque_command,
            const thrust3D_command_t& thrust_command,
            std::array<Servo*, N> servos,
            const conf_t& config = default_config()):
        torque_command_(torque_command),
        thrust_command_(thrust_command),
        servos_(servos),
        mix_(config.mix),
        min_thrust_(config.min_thrust),
        max_thrust_(config.max_thrust)
    {};


    /**
     * \brief  Perform conversion from torque/thrust command to servo command
     */
    bool update(void)
    {
        Mat<N,1> servos_cmd = mix_ % Mat<6,1>({torque_command_.xyz[X],
                                               torque_command_.xyz[Y],
                                               torque_command_.xyz[Z],
                                               thrust_command_.xyz[X],
                                               thrust_command_.xyz[Y],
                                               thrust_command_.xyz[Z]});

        for (uint32_t i=0; i<N; ++i)
        {
            // Clip servo command
            if (servos_cmd[i] > max_thrust_)
            {
               servos_cmd[i] = max_thrust_;
            }
            else if (servos_cmd[i] < min_thrust_)
            {
                servos_cmd[i] = min_thrust_;
            }

            // Write to hardware
            servos_[i]->write(servos_cmd[i]);
        }

        return true;
    };

protected:
    const torque_command_t& torque_command_;
    const thrust3D_command_t& thrust_command_;
    std::array<Servo*, N> servos_;
    Mat<N, 6> mix_;
    float min_thrust_;
    float max_thrust_;
};


#endif /* SERVOS_MIX_HPP_ */
