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
 * \file lequad_collision_avoidance.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class for indoor use
 *
 ******************************************************************************/

#include "sample_projects/LEQuad/lequad_collision_avoidance.hpp"
#include "automatic_navigation/collision_avoidance.hpp"

/**
 * \brief Central data for indoor use
 */
LEQuad_collision_avoidance::LEQuad_collision_avoidance( Imu& imu,
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
                      LEQuad::conf_t config):
          LEQuad(imu, barometer, gps, sonar, serial_mavlink, satellite, led, file_flash,
                     battery, servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7,
                     file1, file2, config),
          collision_avoidance(neighbors, state, navigation, position_estimation, &ahrs, &controls_nav)
{};

// -------------------------------------------------------------------------
// Navigation
// -------------------------------------------------------------------------
bool LEQuad_collision_avoidance::init_navigation(void)
{
  bool ret = true;

  // Parameters
  ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.dist2vel_gain,                           "NAV_DIST2VEL"    );
  ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.cruise_speed,                            "NAV_CRUISESPEED" );
  ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.max_climb_rate,                          "NAV_CLIMBRATE"   );
  ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.takeoff_altitude,                        "NAV_TAKEOFF_ALT" );
  ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.minimal_radius,                          "NAV_MINI_RADIUS" );
  ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.soft_zone_size,                          "NAV_SOFTZONE"    );
  ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.hovering_controller.p_gain,              "NAV_HOVER_PGAIN" );
  ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.hovering_controller.differentiator.gain, "NAV_HOVER_DGAIN" );
  ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.wpt_nav_controller.p_gain,               "NAV_WPT_PGAIN"   );
  ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.wpt_nav_controller.differentiator.gain,  "NAV_WPT_DGAIN"   );
  ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.kp_yaw,                                  "NAV_YAW_KPGAIN"  );

  //collision_avoidance_conf_t collision_avoidance_config = collision_avoidance_default_config();

 /* ret &= collision_avoidance_init(  &collision_avoidance,
                                    collision_avoidance_config,
                                    &neighbors,
                                    &state,
                                    &navigation,
                                    &position_estimation,
                                    &ahrs,
                                    &controls_nav);*/

  ret &= collision_avoidance_telemetry_init(&collision_avoidance,
                                            mavlink_communication.p_message_handler());

  // Task
  ret &= scheduler.add_task(10000, (Scheduler_task::task_function_t)&Navigation::update,               (Scheduler_task::task_argument_t)&navigation,       Scheduler_task::PRIORITY_HIGH);
  ret &= scheduler.add_task(10000, (Scheduler_task::task_function_t)&Mavlink_waypoint_handler::update, (Scheduler_task::task_argument_t)&waypoint_handler, Scheduler_task::PRIORITY_HIGH);


  print_util_dbg_init_msg("[COllISION AVOIDANCE]", ret);

  return ret;
}