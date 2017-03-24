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
 * \file flight_controller_deltawing.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Full flight controller for flying wing
 *
 ******************************************************************************/

#ifndef FLIGHT_CONTROLLER_DELTAWING_HPP_
#define FLIGHT_CONTROLLER_DELTAWING_HPP_

#include "flight_controller/flight_controller_fixedwing.hpp"

class Flight_controller_deltawing: public Flight_controller_fixedwing<3>
{
public:
    Flight_controller_deltawing(const INS& ins, const AHRS& ahrs, Servo& motor, Servo& elevon_right, Servo& elevon_left, conf_t config):
        Flight_controller_fixedwing<3>(ins, ahrs, Servos_mix_matrix<3>::args_t{{{&motor, &elevon_right, &elevon_left}}}, config)
    {};

    static conf_t default_config(void)
    {
        conf_t conf = Flight_controller_fixedwing<3>::default_config();

        //                                 roll, pitch,   yaw,   X,    Y,     Z
        conf.mix_config.mix  = Mat<3, 6>({ 0.0f,  0.0f,  0.0f, 1.0f, 0.0f,  0.0f,   // motor
                                          -1.0f, -1.0f,  0.0f, 0.0f, 0.0f,  0.0f,   // elevon right
                                           1.0f, -1.0f,  0.0f, 0.0f, 0.0f,  0.0f}); // elevon left

        conf.mix_config.trim = Mat<3,1>({ -1.0f,        // motor
                                           0.0f,        // elevon right
                                           0.0f});      // elevon left

        conf.mix_config.min = Mat<3,1>({ -1.0f,        // motor
                                         -1.0f,        // elevon right
                                         -1.0f});      // elevon left

        conf.mix_config.max = Mat<3,1>({ 1.0f,        // motor
                                         1.0f,        // elevon right
                                         1.0f});      // elevon left

        return conf;
    };
};

#endif  // FLIGHT_CONTROLLER_DELTAWING_HPP_
