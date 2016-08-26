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
 * \file torque_controller_birotor.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Basil Huber
 *
 * \brief Links between torque commands and servos PWM command for birotors
 *
 ******************************************************************************/

#ifndef TORQUE_CONTROLLER_HPP_
#define TORQUE_CONTROLLER_HPP_

#include "control/torque_controller.hpp"

extern "C"
{
#include "drivers/unsupported/daler_dc_motor_ctrl.h"
}

class Torque_controller_birotor : public Torque_controller
{

    /**
     * \brief Configuration structure
     */
    struct conf_t
    {
        rot_dir_t   motor_left_dir;     ///< Left motor rotation direction
        rot_dir_t   motor_right_dir;    ///< Right motor rotation direction
        rot_dir_t   servo_left_dir;     ///< Left servo rotation direction
        rot_dir_t   servo_right_dir;    ///< Right servo rotation direction
        float       min_servo;          ///< Minimum servo command
        float       max_servo;          ///< Maximum servo command
        float       min_thrust;        ///< Minimal thrust
        float       max_thrust;        ///< Maxmal thrust
    } servo_mix_birotor_conf_t;


    /**
     * \brief Constructor arguments
     */
    struct args_t
    {
        Servo& motor_left;
        Servo& motor_right;
        Servo& servo_left;
        Servo& servo_right;
        daler_dc_motor_ctrl_t& dc_motors;
    };

    Torque_controller_birotor(args_t& args, const conf_t& config);

private:
    rot_dir_t   motor_left_dir_;                 ///< Left motor rotation direction
    rot_dir_t   motor_right_dir_;                ///< Right motor rotation direction
    rot_dir_t   servo_left_dir_;                 ///< Left servo rotation direction
    rot_dir_t   servo_right_dir_;                ///< Right servo rotation direction
    float       min_servo_;                      ///< Minimum servo command
    float       max_servo_;                      ///< Maximum servo command
    float       min_thrust_;                     ///< Minimal thrust
    float       max_thrust_;                     ///< Maxmal thrust
    Servo& motor_left_;                          ///< Servo (output)
    Servo& motor_right_;                         ///< Servo (output)
    Servo& servo_left_;                          ///< Servo (output)
    Servo& servo_right_;                         ///< Servo (output)
    daler_dc_motor_ctrl_t&  dc_motors_;          ///< DC motor controller (output)
};

#endif // TORQUE_CONTROLLER_HPP_
