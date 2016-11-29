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
 * \file flight_controller_copter.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Full flight controller for copter
 *
 ******************************************************************************/

#ifndef FLIGHT_CONTROLLER_COPTER_HPP_
#define FLIGHT_CONTROLLER_COPTER_HPP_

#include "control/flight_controller_stack.hpp"
#include "control/position_controller.hpp"
#include "control/velocity_controller_copter.hpp"
#include "control/attitude_controller.hpp"
#include "control/rate_controller.hpp"
#include "control/servos_mix_matrix.hpp"

template<uint32_t N_ROTORS>
class Flight_controller_copter: public Flight_controller_stack
{
public:

    struct conf_t
    {
        Position_controller::conf_t         pos_config;
        Velocity_controller_copter::conf_t  vel_config;
        Attitude_controller::conf_t         att_config;
        Rate_controller::conf_t             rate_config;
        typename Servos_mix_matrix<N_ROTORS>::conf_t mix_config;
    };

    static conf_t default_config(void)
    {
        conf_t conf;

        conf.pos_config  = Position_controller::default_config();
        conf.vel_config  = Velocity_controller_copter::default_config();
        conf.att_config  = Attitude_controller::default_config();
        conf.rate_config = Rate_controller::default_config();
        conf.mix_config  = Servos_mix_matrix<N_ROTORS>::default_config();

        return conf;
    };

    Flight_controller_copter(const INS& ins, const ahrs_t& ahrs, typename Servos_mix_matrix<N_ROTORS>::args_t mix_args, conf_t config):
        Flight_controller_stack(pos_ctrl_, vel_ctrl_, att_ctrl_, rate_ctrl_, mix_ctrl_),
        pos_ctrl_({ahrs, ins}, config.pos_config),
        vel_ctrl_({{ahrs, ins, command_.velocity, command_.attitude, command_.thrust}}, config.vel_config),
        att_ctrl_({ahrs, command_.attitude, command_.rate}, config.att_config),
        rate_ctrl_({ahrs, command_.rate, command_.torque}, config.rate_config),
        mix_ctrl_(mix_args, config.mix_config),
        ahrs_(ahrs)
    {};


    bool set_manual_rate_command(const Manual_control& manual_control)
    {
        bool ret = true;
        rate_command_t rate_command;
        thrust_command_t thrust_command;
        manual_control.get_rate_command(rate_command);
        manual_control.get_thrust_command_copter(thrust_command);
        ret &= set_command(rate_command);
        ret &= set_command(thrust_command);
        return ret;
    };

    bool set_manual_attitude_command(const Manual_control& manual_control)
    {
        bool ret = true;
        attitude_command_t att_command;
        thrust_command_t thrust_command;
        manual_control.get_attitude_command(att_command, ahrs_.qe);
        manual_control.get_thrust_command_copter(thrust_command);
        ret &= set_command(att_command);
        ret &= set_command(thrust_command);
        return ret;
    };

    bool set_manual_velocity_command(const Manual_control& manual_control)
    {
        bool ret = true;
        velocity_command_t new_vel_command;
        manual_control.get_velocity_command_copter(new_vel_command, ahrs_.qe, command_.velocity);
        ret &= set_command(new_vel_command);
        return ret;
    };

public:
    Position_controller         pos_ctrl_;
    Velocity_controller_copter  vel_ctrl_;
    Attitude_controller         att_ctrl_;
    Rate_controller             rate_ctrl_;
    Servos_mix_matrix<N_ROTORS> mix_ctrl_;

private:
    const ahrs_t& ahrs_;
};


class Flight_controller_quadcopter_diag: public Flight_controller_copter<4>
{
public:
    Flight_controller_quadcopter_diag(const INS& ins, const ahrs_t& ahrs, Servo& motor_rl, Servo& motor_fl, Servo& motor_fr, Servo& motor_rr, conf_t config):
        Flight_controller_copter<4>(ins, ahrs, Servos_mix_matrix<4>::args_t{{{&motor_rl, &motor_fl, &motor_fr, &motor_rr}}}, config)
    {};

    static conf_t default_config(void)
    {
        conf_t conf = Flight_controller_copter<4>::default_config();

        conf.mix_config.mix  = Mat<4, 6>({ 1.0f, -1.0f,  1.0f, 0.0f, 0.0f, -1.0f,
                                           1.0f,  1.0f, -1.0f, 0.0f, 0.0f, -1.0f,
                                          -1.0f,  1.0f,  1.0f, 0.0f, 0.0f, -1.0f,
                                          -1.0f, -1.0f, -1.0f, 0.0f, 0.0f, -1.0f});

        return conf;
    };
};

class Flight_controller_quadcopter_cross: public Flight_controller_copter<4>
{
public:
    Flight_controller_quadcopter_cross(  const INS& ins,
                                        const ahrs_t& ahrs,
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


class Flight_controller_hexacopter: public Flight_controller_copter<6>
{
public:
    Flight_controller_hexacopter(   const INS& ins,
                                    const ahrs_t& ahrs,
                                    Servo& motor_rear,
                                    Servo& motor_rear_left,
                                    Servo& motor_front_left,
                                    Servo& motor_front,
                                    Servo& motor_front_right,
                                    Servo& motor_rear_right,
                                    conf_t config):
        Flight_controller_copter<6>(ins, ahrs, Servos_mix_matrix<6>::args_t{{{&motor_rear, &motor_rear_left, &motor_front_left, &motor_front, &motor_front_right, &motor_rear_right}}}, config)
    {};

    static conf_t default_config(void)
    {
        conf_t conf = Flight_controller_copter<6>::default_config();
        float s60 = 0.866025f;
        conf.mix_config.mix  = Mat<6, 6>({ 0.0f, -1.0f, -1.0f, 0.0f, 0.0f, -1.0f,   // rear
                                            s60, -0.5f,  1.0f, 0.0f, 0.0f, -1.0f,   // rear left
                                            s60,  0.5f, -1.0f, 0.0f, 0.0f, -1.0f,   // front left
                                           0.0f,  1.0f,  1.0f, 0.0f, 0.0f, -1.0f,   // front
                                           -s60,  0.5f, -1.0f, 0.0f, 0.0f, -1.0f,   // front right
                                           -s60, -0.5f,  1.0f, 0.0f, 0.0f, -1.0f}); // rear right

        return conf;
    };
};

// conf.mix_config.mix  = Mat<6, 6>({ 0.0f, -1.0f, -1.0f,  1.0f, 0.0f, 1.0f,   // rear
//                                     s60, -0.5f,  1.0f, -0.5f, -s60, 1.0f,   // rear left
//                                     s60,  0.5f, -1.0f, -0.5f,  s60, 1.0f,   // front left
//                                    0.0f,  1.0f,  1.0f,  1.0f, 0.0f, 1.0f,   // front
//                                    -s60,  0.5f, -1.0f, -0.5f, -s60, 1.0f,   // front right
//                                    -s60, -0.5f,  1.0f, -0.5f,  s60, 1.0f}); // rear right

#endif  // FLIGHT_CONTROLLER_COPTER_HPP_
