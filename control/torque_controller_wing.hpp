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
 * \file torque_controller_wing.hpp
 *
 * \author MAV'RIC Team
 * \author Simon Pyroth
 * \author Basil Huber
 *
 * \brief Links between regulation output and PWM commands for a wing aircraft
 *
 ******************************************************************************/


#ifndef SERVOS_MIX_WING_HPP_
#define SERVOS_MIX_WING_HPP_

#include "drivers/servo.hpp"

#include "control/torque_controller.hpp"

class Torque_controller_wing : public Torque_controller
{
public:
    /**
     * \brief The servo mix structure for a wing
     */
    struct conf_t
    {
        uint8_t             servo_right;        ///< Right aileron servo index
        uint8_t             servo_left;         ///< Left aileron servo index
        uint8_t             motor;              ///< Propulsion motor index

        flap_dir_t          servo_right_dir;    ///< Right aileron servo direction
        flap_dir_t          servo_left_dir;     ///< Left aileron servo direction

        float               min_amplitude;      ///< Minimum value which can be put on servo
        float               max_amplitude;      ///< Maximum value which can be put on servo

        float               trim_roll;          ///< Trim value for roll
        float               trim_pitch;         ///< Trim value for pitch

        float               min_thrust;        ///< Minimal thrust
        float               max_thrust;        ///< Maxmal thrust
    };

    /**
     * \brief Constructor arguments
     */
    struct args_t
    {
        Servo& servo_left;
        Servo& servo_right;
        Servo& motor;
    };

    Torque_controller_wing(args_t& args, const conf_t& config = default_config());

    virtual void update();

    static inline conf_t default_config();

private:
    conf_t   config_;                        ///< Configuration of the mix
    Servo& servo_left_;                      ///< Left servo
    Servo& servo_right_;                     ///< Right servo
    Servo& motor_;                           ///< Motor
};

Torque_controller_wing::conf_t Torque_controller_wing::default_config()
{
    conf_t conf;

    conf.servo_right = 2;
    conf.servo_left = 1;
    conf.motor = 0;

    conf.servo_right_dir = FLAP_INVERTED;
    conf.servo_left_dir = FLAP_NORMAL;

    conf.min_amplitude = -1.0f;
    conf.max_amplitude = 1.0f;

    conf.trim_roll = 0.252273f;
    conf.trim_pitch = 0.0090908f;

    conf.min_thrust = -0.9f;
    conf.max_thrust = 1.0f;

    return conf;
};

#endif
