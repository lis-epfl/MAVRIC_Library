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
 * \file hexhog.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/


#ifndef HEXHOG_HPP_
#define HEXHOG_HPP_

#include <stdbool.h>
#include <stdint.h>

#include "sample_projects/LEQuad/lequad.hpp"
#include "control/servos_mix_6dof.hpp"

/**
 * \brief MAV class
 */
class Hexhog: public LEQuad
{
public:
    /**
     * \brief   Configuration structure
     */
     struct conf_t
    {
        LEQuad::conf_t lequad_config;
        Servos_mix_6dof<6>::conf_t servos_mix_config;
    };

    /**
     * \brief   Default configuration
     *
     * \param   sysid       System id (default value = 1)
     *
     * \return  Config structure
     */
    static inline conf_t default_config(uint8_t sysid = 1);


    /**
     * \brief   Constructor
     */
    Hexhog( Imu& imu,
            Barometer& barometer,
            Gps& gps,
            Sonar& sonar,
            Serial& serial_mavlink,
            Satellite& satellite,
            Led& led,
            File& file_flash,
            Battery& battery,
            Servo& servo_0,
            Servo& servo_1,
            Servo& servo_2,
            Servo& servo_3,
            Servo& servo_4,
            Servo& servo_5,
            Servo& servo_6,
            Servo& servo_7,
            File& file1,
            File& file2,
            const conf_t& config = default_config()):
        LEQuad(imu, barometer, gps, sonar, serial_mavlink, satellite, led, file_flash, battery,
              servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7,
              file1, file2, config.lequad_config),
        servos_mix_(command.torque, command.thrust3D,
                    std::array<Servo*, 6>{{&servo_0, &servo_1, &servo_2, &servo_3, &servo_4, &servo_5}},
                    config.servos_mix_config)
    {};


protected:
    virtual bool main_task(void);
    Servos_mix_6dof<6> servos_mix_;
};


bool Hexhog::main_task(void)
{
    // Update estimation
    imu.update();
    ahrs_ekf.update();
    position_estimation.update();

    bool failsafe = false;

    // Do control
    if (state.is_armed())
    {
        switch (state.mav_mode().ctrl_mode())
        {
            case Mav_mode::GPS_NAV:
                controls = controls_nav;
                controls.control_mode = VELOCITY_COMMAND_MODE;

                // if no waypoints are set, we do position hold therefore the yaw mode is absolute
                if ((((state.nav_plan_active || (navigation.navigation_strategy == Navigation::strategy_t::DUBIN)) && (navigation.internal_state_ == Navigation::NAV_NAVIGATING)) || (navigation.internal_state_ == Navigation::NAV_STOP_THERE))
              	   || ((state.mav_state_ == MAV_STATE_CRITICAL) && (navigation.critical_behavior == Navigation::FLY_TO_HOME_WP)))
            	{
                    controls.yaw_mode = YAW_RELATIVE;
                }
                else
                {
                    controls.yaw_mode = YAW_ABSOLUTE;
                }
                break;

            case Mav_mode::POSITION_HOLD:
                controls = controls_nav;
                controls.control_mode = VELOCITY_COMMAND_MODE;

                if ( ((state.mav_state_ == MAV_STATE_CRITICAL) && (navigation.critical_behavior == Navigation::FLY_TO_HOME_WP))  || (navigation.navigation_strategy == Navigation::strategy_t::DUBIN))
                {
                    controls.yaw_mode = YAW_RELATIVE;
                }
                else
                {
                    controls.yaw_mode = YAW_ABSOLUTE;
                }
                break;

            case Mav_mode::VELOCITY:
                manual_control.get_velocity_vector(&controls);
                controls.control_mode = VELOCITY_COMMAND_MODE;
                controls.yaw_mode = YAW_RELATIVE;
                break;

            case Mav_mode::ATTITUDE:
                manual_control.get_control_command(&controls);
                controls.control_mode = ATTITUDE_COMMAND_MODE;
                controls.yaw_mode = YAW_RELATIVE;

                if (state.is_custom())
                {
                    command.thrust3D.xyz[Y] = controls.rpy[ROLL];
                    controls.rpy[ROLL] = 0.0f;

                    command.thrust3D.xyz[X] = -controls.rpy[PITCH];
                    controls.rpy[PITCH] = 0.0f;
                }
                else
                {
                    command.thrust3D.xyz[Y] = 0.0f;
                    command.thrust3D.xyz[X] = 0.0f;
                }

                break;

            //case Mav_mode::RATE:
            //    manual_control.get_rate_command(&controls);
            //    controls.control_mode = RATE_COMMAND_MODE;
            //    break;

            default:
                failsafe = true;    // undefined behaviour -> failsafe
        }
    }
    else    // !state.is_armed()
    {
        failsafe = true;    // undefined behaviour -> failsafe
    }

    // if behaviour defined, execute controller and mix; otherwise: set servos to failsafe
    if(!failsafe)
    {
        stabilisation_copter_cascade_stabilise(&stabilisation_copter);
        // servos_mix_quadcopter_diag_update(&servo_mix);
        command.thrust3D.xyz[Z] = command.thrust.thrust;
        servos_mix_.update();
    }
    else
    {
        servo_0.failsafe();
        servo_1.failsafe();
        servo_2.failsafe();
        servo_3.failsafe();
        servo_4.failsafe();
        servo_5.failsafe();
    }

    return true;
}


Hexhog::conf_t Hexhog::default_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf.lequad_config    = LEQuad::default_config();

    conf.servos_mix_config = Servos_mix_6dof<6>::default_config();
    float s60 = 0.866025f;
    conf.servos_mix_config.mix = Mat<6, 6>({  0.0f,  1.0f,  1.0f,  1.0f, 0.0f, 1.0f,
                                               s60,  0.5f, -1.0f, -0.5f,  s60, 1.0f,
                                               s60, -0.5f,  1.0f, -0.5f, -s60, 1.0f,
                                              0.0f, -1.0f, -1.0f,  1.0f, 0.0f, 1.0f,
                                              -s60, -0.5f,  1.0f, -0.5f,  s60, 1.0f,
                                              -s60,  0.5f, -1.0f, -0.5f, -s60, 1.0f});

    return conf;
};

#endif /* HEXHOG_HPP_ */
