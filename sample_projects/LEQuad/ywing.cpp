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
 * \file ywing.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/


#include "sample_projects/LEQuad/ywing.hpp"

Ywing::Ywing(Imu& imu, Barometer& barometer, Gps& gps, Sonar& sonar, Serial& serial_mavlink, Satellite& satellite, Led& led, File& file_flash, Battery& battery, Servo& servo_0, Servo& servo_1, Servo& servo_2, Servo& servo_3, Servo& servo_4, Servo& servo_5, Servo& servo_6, Servo& servo_7, File& file1, File& file2, const conf_t& config):
    LEQuad(imu, barometer, gps, sonar, serial_mavlink, satellite, led, file_flash, battery, servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7, file1, file2, config.lequad_config),
    config_(config)
{
  servos_mix_ywing_init(&servos_mix_ywing,
                              &config_.servos_mix_ywing_config,
                              &command.torque, &command.thrust,
                              &servo_0, &servo_1, &servo_2, &servo_3);
}


// -------------------------------------------------------------------------
// Main task
// -------------------------------------------------------------------------
bool Ywing::main_task(void)
{
    // Update estimation
    imu.update();
    ahrs_ekf.update();
    position_estimation.update();

    // Do control
    if (state.is_armed())
    {
        if (state.is_auto())
        {
            controls = controls_nav;
            controls.control_mode = VELOCITY_COMMAND_MODE;

            // if no waypoints are set, we do position hold therefore the yaw mode is absolute
            if (((state.nav_plan_active && (navigation.internal_state_ == Navigation::NAV_NAVIGATING)) || (navigation.internal_state_ == Navigation::NAV_STOP_THERE))
              || ((state.mav_state_ == MAV_STATE_CRITICAL) && (navigation.critical_behavior == Navigation::FLY_TO_HOME_WP)))
            {
                controls.yaw_mode = YAW_RELATIVE;
            }
            else
            {
                controls.yaw_mode = YAW_ABSOLUTE;
            }

            //if (state.in_the_air || navigation.auto_takeoff)
            if (true)//navigation.internal_state_ > NAV_ON_GND)
            {
                stabilisation_copter_cascade_stabilise(&stabilisation_copter);
                servos_mix_ywing_update(&servos_mix_ywing);
            }
        }
        else if (state.is_guided())
        {
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

            //if (state.in_the_air || navigation.auto_takeoff)
            if (true)//navigation.internal_state_ > NAV_ON_GND)
            {
                stabilisation_copter_cascade_stabilise(&stabilisation_copter);
                servos_mix_ywing_update(&servos_mix_ywing);
            }
        }
        else if (state.is_stabilize())
        {
            manual_control.get_velocity_vector(&controls);

            controls.control_mode = VELOCITY_COMMAND_MODE;
            controls.yaw_mode = YAW_RELATIVE;

            //if (state.in_the_air || navigation.auto_takeoff)
            if (true)//navigation.internal_state_ > NAV_ON_GND)
            {
                stabilisation_copter_cascade_stabilise(&stabilisation_copter);
                servos_mix_ywing_update(&servos_mix_ywing);
            }
        }
        else if (state.is_manual())
        {
            manual_control.get_control_command(&controls);
            // manual_control.get_control_command_absolute_yaw(&controls);

            controls.control_mode = ATTITUDE_COMMAND_MODE;
            controls.yaw_mode = YAW_ABSOLUTE;

            stabilisation_copter_cascade_stabilise(&stabilisation_copter);
            servos_mix_ywing_update(&servos_mix_ywing);
        }
        else
        {
            servo_0.failsafe();
            servo_1.failsafe();
            servo_2.failsafe();
            servo_3.failsafe();
        }
    }
    else
    {
        servo_0.failsafe();
        servo_1.failsafe();
        servo_2.failsafe();
        servo_3.failsafe();
    }

    return true;
}


  // // Attitude controller
  // ret &= attitude_controller_init(&attitude_controller,
  //                                 config_.attitude_controller_config,
  //                                 &ahrs,
  //                                 &command.attitude,
  //                                 &command.rate,
  //                                 &command.torque);
  //
  // // Velocity controller
  // ret &= velocity_controller_copter_init(&velocity_controller,
  //                                        config_.velocity_controller_copter_config,
  //                                        &ahrs,
  //                                        &position_estimation,
  //                                        &command.velocity,
  //                                        &command.attitude,
  //                                        &command.thrust);
  //
  // // Vector field
  // ret &= vector_field_waypoint_init(&vector_field_waypoint,
  //                                   {},
  //                                   &waypoint_handler,
  //                                   &position_estimation,
  //                                   &command.velocity);
