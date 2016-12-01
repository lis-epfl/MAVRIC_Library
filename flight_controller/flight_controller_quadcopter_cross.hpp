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
 * \file flight_controller_quadcopter_cross.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Full flight controller for quadcopter
 *
 ******************************************************************************/

#ifndef FLIGHT_CONTROLLER_QUADCOPTER_CROSS_HPP_
#define FLIGHT_CONTROLLER_QUADCOPTER_CROSS_HPP_

#include "flight_controller/flight_controller_copter.hpp"

class Flight_controller_quadcopter_cross: public Flight_controller_copter<4>
{
public:
    Flight_controller_quadcopter_cross(  const INS& ins,
                                        const AHRS& ahrs,
                                        Servo& motor_rear,
                                        Servo& motor_left,
                                        Servo& motor_front,
                                        Servo& motor_right,
                                        conf_t config):
        Flight_controller_copter<4>(ins, ahrs, Servos_mix_matrix<4>::args_t{{{&motor_left, &motor_front, &motor_right, &motor_rear}}}, config)
    {};

    static conf_t default_config(void)
    {
        conf_t conf = Flight_controller_copter<4>::default_config();

        conf.mix_config.mix  = Mat<4, 6>({ 0.0f, -1.0f, -1.0f, 0.0f, 0.0f, -1.0f,
                                           1.0f,  0.0f,  1.0f, 0.0f, 0.0f, -1.0f,
                                           0.0f,  1.0f, -1.0f, 0.0f, 0.0f, -1.0f,
                                          -1.0f,  0.0f,  1.0f, 0.0f, 0.0f, -1.0f });

        return conf;
    };
};

#endif  // FLIGHT_CONTROLLER_QUADCOPTER_CROSS_HPP_
