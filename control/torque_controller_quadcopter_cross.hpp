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
 * \file torque_controller_quadcopter_cross.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Nicolas Dousse
 * \author Basil Huber
 *
 * \brief Links between torque commands and servos PWM command for quadcopters
 * in cross configuration
 *
 ******************************************************************************/


#ifndef TORQUE_CONTROLLER_QUADCOPTER_CROSS_HPP_
#define TORQUE_CONTROLLER_QUADCOPTER_CROSS_HPP_

#include "control/torque_controller.hpp"


class Torque_controller_quadcopter_cross : public Torque_controller
{
public:

    /**
     * \brief Configuration
     */
    struct conf_t
    {
        rot_dir_t   motor_front_dir;              ///< Front  motor turning direction
        rot_dir_t   motor_left_dir;               ///< Left  motor turning direction
        rot_dir_t   motor_right_dir;              ///< Front motor turning direction
        rot_dir_t   motor_rear_dir;               ///< Right motor turning direction
        float       min_thrust;                   ///< Minimal thrust
        float       max_thrust;                   ///< Maxmal thrust
    };

    /**
     * \brief Constructor arguments
     */
    struct args_t
    {
        Servo& motor_front;
        Servo& motor_left;
        Servo& motor_right;
        Servo& motor_rear;
    };   

    /**
     * \brief                   Constructor
     *
     * \param args              Constructor arguments
     * \param config            configuration
     */
    Torque_controller_quadcopter_cross(args_t args, const conf_t& config);


    /*
     * \brief   Write motor commands to servo structure based on torque command
     */
    virtual void update();

    static inline conf_t default_config();

private:
    rot_dir_t   motor_front_dir_;              ///< Front  motor turning direction
    rot_dir_t   motor_left_dir_;               ///< Left  motor turning direction
    rot_dir_t   motor_right_dir_;              ///< Right motor turning direction
    rot_dir_t   motor_rear_dir_;               ///< Rear motor turning direction
    float       min_thrust_;                   ///< Minimal thrust
    float       max_thrust_;                   ///< Maxmal thrust
    Servo&      motor_front_;                  ///< Servo for front motor
    Servo&      motor_left_;                   ///< Servo for left left motor
    Servo&      motor_right_;                  ///< Servo for right right motor
    Servo&      motor_rear_;                   ///< Servo for rear right motor
};

#endif /* TORQUE_CONTROLLER_QUADCOPTER_CROSS_HPP_ */
