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
 * \file servos_mix_matrix.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief Servo mix
 *
 * Example use for hexhog, an hexacopter with tilted motors:
 *
 * ```cpp
 *     # Some required variables
 *     torque_command_t torque_command;
 *     thrust_command_t thrust_command;
 *     thrust3D_command_t thrust3d_command;
 *
 *     // Configuration structure
 *     Servos_mix_matrix<6>::conf_t servosmix_config = Servos_mix_matrix<6>::default_config();
 *     float s60 = 0.866025f;
 *     servosmix_config.mix = Mat<6, 6>({  0.0f,  1.0f,  1.0f,  1.0f, 0.0f, 1.0f,
 *                                         s60,  0.5f, -1.0f, -0.5f,  s60, 1.0f,
 *                                         s60, -0.5f,  1.0f, -0.5f, -s60, 1.0f,
 *                                        0.0f, -1.0f, -1.0f,  1.0f, 0.0f, 1.0f,
 *                                        -s60, -0.5f,  1.0f, -0.5f,  s60, 1.0f,
 *                                        -s60, 0.5f, -1.0f, -0.5f, -s60, 1.0f});
 *
 *      # Create object
 *      Servos_mix_matrix<6> servosmix( torque_command,
 *                                  thrust3d_command,
 *                                  std::array<Servo*, 6>{&board.servo_0, &board.servo_1,
 *                                                        &board.servo_4, &board.servo_5},
 *                                  servosmix_config);
 * ```
 ******************************************************************************/


#ifndef SERVOS_MIX_MATRIX_HPP_
#define SERVOS_MIX_MATRIX_HPP_

#include "control/servos_mix.hpp"
#include "drivers/servo.hpp"
#include "util/matrix.hpp"

/**
 * \brief Servos mix
 *
 * \tparam  N   Number of servos
 */
template<uint32_t N>
class Servos_mix_matrix: Servos_mix
{
public:
    /**
     * \brief   Configuration structure
     */
    struct conf_t
    {
        Mat<N, 6> mix;          ///< Mix matrix
        Mat<N, 1> trim;         ///< Trim values added to output
        Mat<N, 1> min;          ///< Minimum commands to apply to servo
        Mat<N, 1> max;          ///< Maximum commands to apply to servo
    };

    /**
     * \brief   Default configuration
     *
     * \return  Config structure
     */
    static inline conf_t default_config(void)
    {
        conf_t conf = {};
        conf.mix        = Mat<N,6>(0.0f);
        conf.trim       = Mat<N,1>(-1.0f);  // -1.0f in case some servos are motors
        conf.min        = Mat<N,1>(-0.9f);
        conf.max        = Mat<N,1>(1.0f);
        return conf;
    };


    /**
     * \brief   Required arguments
     */
    struct args_t
    {
        std::array<Servo*, N> servos;   ///< List of servos
    };

    /**
     * \brief   Constructor
     */
    Servos_mix_matrix( const args_t& args,
                       const conf_t& config = default_config()):
        servos_(args.servos),
        torque_command_{std::array<float,3>{{0.0f, 0.0f, 0.0f}}},
        thrust_command_{std::array<float,3>{{0.0f, 0.0f, 0.0f}}},
        mix_(config.mix),
        trim_(config.trim),
        min_(config.min),
        max_(config.max)
    {};


    /**
     * \brief  Perform conversion from torque/thrust command to servo command
     */
    virtual bool update(void)
    {
        // Mix commands into servo values
        Mat<N,1> servos_cmd = mix_ % Mat<6,1>({torque_command_.xyz[X],
                                               torque_command_.xyz[Y],
                                               torque_command_.xyz[Z],
                                               thrust_command_.xyz[X],
                                               thrust_command_.xyz[Y],
                                               thrust_command_.xyz[Z]});

        // Add trim
        servos_cmd += trim_;

        // Clip between min and max actuator values
        servos_cmd.clip(min_, max_);

        // Write to hardware
        for (uint32_t i=0; i<N; ++i)
        {
            servos_[i]->write(servos_cmd[i]);
        }

        return true;
    };


    /**
     * \brief           sets the torque command
     *
     * \param command   torque command in body frame
     *
     * \return success  whether command was accepted
     */
    virtual bool set_command(const torque_command_t& torque)
    {
        torque_command_ = torque;
        return true;
    };


    /**
     * \brief           sets the thrust command
     *
     * \param command   thrust command in body frame
     *
     * \return success  whether command was accepted
     */
    virtual bool set_command(const thrust_command_t& thrust)
    {
        thrust_command_ = thrust;
        return true;
    };


    /**
     * \brief           sets the torque command
     *
     * \param command   torque command in body frame
     *
     * \return success  whether command was accepted
     */
    virtual bool get_command(torque_command_t& torque) const
    {
        torque = torque_command_;
        return true;
    }


    /**
     * \brief           sets the thrust command
     *
     * \param command   thrust command in body frame
     *
     * \return success  whether command was accepted
     */
    virtual bool get_command(thrust_command_t& thrust) const
    {
        thrust = thrust_command_;
        return true;
    };


protected:
    std::array<Servo*, N> servos_;

    torque_command_t torque_command_;
    thrust_command_t thrust_command_;

    Mat<N, 6> mix_;
    Mat<N, 1> trim_;
    Mat<N, 1> min_;
    Mat<N, 1> max_;
};


#endif /* SERVOS_MIX_MATRIX_HPP_ */
