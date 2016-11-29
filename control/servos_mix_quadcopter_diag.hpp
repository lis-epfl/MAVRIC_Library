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
 * \file servos_mix_quadcopter_diag.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Nicolas Dousse
 * \author Basil Huber
 *
 * \brief Links between torque commands and servos PWM command for quadcopters
 * in diagonal configuration
 *
 ******************************************************************************/


#ifndef SERVOS_MIX_QUADCOPTER_DIAG_HPP_
#define SERVOS_MIX_QUADCOPTER_DIAG_HPP_

#include "control/servos_mix_matrix.hpp"

class Servos_mix_quadcopter_diag: public Servos_mix_matrix<4>
{
public:

    /**
     * \brief                   Constructor (calls only super class constructor)
     *
     * \param motor_rear_left   Rear left motor
     * \param motor_front_left  Front left motor
     * \param motor_front_right Front right motor
     * \param motor_rear_right  Rear right motor
     * \param config            configuration
     */
    Servos_mix_quadcopter_diag( Servo& motor_rear_left,
                                Servo& motor_front_left,
                                Servo& motor_front_right,
                                Servo& motor_rear_right,
                                const conf_t& config = default_config()):
        Servos_mix_matrix<4>(Servos_mix_matrix<4>::args_t{{ &motor_rear_left,
                                                            &motor_front_left,
                                                            &motor_front_right,
                                                            &motor_rear_right }},
                             config)
    {};

    /*
     * \brief   Default configuration
     *
     * \return default configuration
     */
    static inline conf_t default_config();
};

Servos_mix_quadcopter_diag::conf_t Servos_mix_quadcopter_diag::default_config()
{
    conf_t conf = Servos_mix_matrix<4>::default_config();

    conf.mix =  Mat<4, 6>({ 1.0f, -1.0f,  1.0f, 0.0f, 0.0f, -1.0f,
                            1.0f,  1.0f, -1.0f, 0.0f, 0.0f, -1.0f,
                           -1.0f,  1.0f,  1.0f, 0.0f, 0.0f, -1.0f,
                           -1.0f, -1.0f, -1.0f, 0.0f, 0.0f, -1.0f});

    return conf;
}

#endif /* SERVOS_MIX_QUADCOPTER_DIAG_HPP_ */
