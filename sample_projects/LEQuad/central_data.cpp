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
 * \file central_data.c
 *
 * \author MAV'RIC Team
 *
 * \brief Place where the central data is stored and initialized
 *
 ******************************************************************************/


#include "sample_projects/LEQuad/central_data.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"

#include "util/print_util.h"
}


Central_data::Central_data(uint8_t sysid, Imu& imu, Barometer& barometer, Gps& gps, Sonar& sonar, Serial& serial_mavlink, Serial& raspi_serial_mavlink, Satellite& satellite, Led& led, File& file_flash, Battery& battery, Servo& servo_0, Servo& servo_1, Servo& servo_2, Servo& servo_3, File& file1, File& file2,  Offboard_Camera& ob_camera, central_data_conf_t config):
    imu(imu),
    barometer(barometer),
    gps(gps),
    sonar(sonar),
    serial_mavlink(serial_mavlink),
    raspi_serial_mavlink(raspi_serial_mavlink),
    satellite(satellite),
    led(led),
    file_flash(file_flash),
    battery(battery),
    servo_0(servo_0),
    servo_1(servo_1),
    servo_2(servo_2),
    servo_3(servo_3),
    state(mavlink_communication.mavlink_stream, battery, config.state_config),
    data_logging(file1, state, config.data_logging_config),
    data_logging2(file2, state, config.data_logging_config2),
    offboard_camera(ob_camera),
    altitude_estimation_(sonar, barometer, ahrs, altitude_),
    altitude_controller_(command.position, altitude_, command.thrust),
    sysid_(sysid),
    config_(config)
{}


bool Central_data::init(void)
{
    bool init_success = true;
    bool ret;

    print_util_dbg_sep('%');
    time_keeper_delay_ms(50);
    print_util_dbg_sep('-');
    time_keeper_delay_ms(50);
    print_util_dbg_print("[CENTRAL_DATA] ...\r\n");
    time_keeper_delay_ms(50);
    print_util_dbg_sep('-');


    // -------------------------------------------------------------------------
    // Init main sheduler
    // -------------------------------------------------------------------------
    ret = scheduler_init(&scheduler, config_.scheduler_config);
    print_util_dbg_init_msg("[SCHEDULER]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init mavlink communication
    // -------------------------------------------------------------------------
    mavlink_communication_conf_t mavlink_communication_config = config_.mavlink_communication_config;
    mavlink_communication_config.mavlink_stream_config.sysid = sysid_;
    mavlink_communication_config.message_handler_config.debug = true;
    mavlink_communication_config.onboard_parameters_config.debug = true;
    mavlink_communication_config.mavlink_stream_config.debug = true;
    ret = mavlink_communication_init(&mavlink_communication,
                                     mavlink_communication_config,
                                     &serial_mavlink,
                                     &state,
                                     &file_flash);
    print_util_dbg_init_msg("[MAVLINK]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init raspberry pi mavlink communication
    // -------------------------------------------------------------------------
    /*mavlink_communication_conf_t raspi_mavlink_communication_config = mavlink_communication_default_config();
    raspi_mavlink_communication_config.mavlink_stream_config.sysid = sysid_;
    raspi_mavlink_communication_config.message_handler_config.debug = true;
    raspi_mavlink_communication_config.onboard_parameters_config.debug = true;
    raspi_mavlink_communication_config.mavlink_stream_config.debug = true;*/
    ret = mavlink_communication_init(&raspi_mavlink_communication,
                                     mavlink_communication_config,
                                     &raspi_serial_mavlink,
                                     &state,
                                     &file_flash);
    print_util_dbg_init_msg("[RASPI MAVLINK]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    //Init state_machine
    // -------------------------------------------------------------------------
    ret = state_machine_init(&state_machine,
                             &state,
                             &gps,
                             &imu,
                             &manual_control);
    print_util_dbg_init_msg("[STATE MACHINE]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init ahrs
    // -------------------------------------------------------------------------
    ret = ahrs_init(&ahrs);
    print_util_dbg_init_msg("[AHRS]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init qfilter
    // -------------------------------------------------------------------------
    ret = qfilter_init(&attitude_filter,
                       config_.qfilter_config,
                       &imu,
                       &ahrs);
    print_util_dbg_init_msg("[QFILTER]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init position_estimation_init
    // -------------------------------------------------------------------------
    ret = position_estimation_init(&position_estimation,
                                   config_.position_estimation_config,
                                   &state,
                                   &barometer,
                                   &sonar,
                                   &gps,
                                   &ahrs);
    print_util_dbg_init_msg("[POS EST]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init navigation
    // -------------------------------------------------------------------------
    ret = navigation_init(&navigation,
                          config_.navigation_config,
                          &controls_nav,
                          &ahrs.qe,
                          &position_estimation,
                          &state,
                          &mavlink_communication);/*,
                            &sonar_i2cxl);*/
    print_util_dbg_init_msg("[NAV]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init waypoint handler
    // -------------------------------------------------------------------------
    ret = waypoint_handler_init(&waypoint_handler,
                                &position_estimation,
                                &navigation,
                                &ahrs,
                                &state,
                                &manual_control,
                                &mavlink_communication,
                                &mavlink_communication.mavlink_stream);
    waypoint_handler_init_homing_waypoint(&waypoint_handler);
    waypoint_handler_nav_plan_init(&waypoint_handler);
    print_util_dbg_init_msg("[WAYPOINT]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init stabilisers
    // -------------------------------------------------------------------------
    ret = stabilisation_copter_init(&stabilisation_copter,
                                    config_.stabilisation_copter_config,
                                    &controls,
                                    &ahrs,
                                    &position_estimation,
                                    &command.torque,
                                    &command.thrust);
    print_util_dbg_init_msg("[STABILISATION COPTER]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init controls
    // -------------------------------------------------------------------------
    ret = stabilisation_init(&controls);
    print_util_dbg_init_msg("[CONTROLS]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init hud
    // -------------------------------------------------------------------------
    ret = hud_telemetry_init(&hud_structure,
                             &position_estimation,
                             &controls,
                             &ahrs);
    print_util_dbg_init_msg("[HUD]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init servo mixing
    // -------------------------------------------------------------------------
    ret = servos_mix_quadcotper_diag_init(&servo_mix,
                                          config_.servos_mix_quadcopter_diag_config,
                                          &command.torque,
                                          &command.thrust,
                                          &servo_0,
                                          &servo_1,
                                          &servo_2,
                                          &servo_3);
    print_util_dbg_init_msg("[SERVOS MIX]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);

    // -------------------------------------------------------------------------
    // Init servo telemetry
    // -------------------------------------------------------------------------
    servos_telemetry_init(&servos_telemetry,
                          &servo_0,
                          &servo_1,
                          &servo_2,
                          &servo_3);

    // -------------------------------------------------------------------------
    // Init manual control
    // -------------------------------------------------------------------------
    ret = manual_control_init(&manual_control,
                              &satellite,
                              config_.manual_control_config,
                              config_.remote_config);
    print_util_dbg_init_msg("[MANUAL CTRL]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);

    //--------------------------------------------------------------------------
    // Init attitude controller
    //--------------------------------------------------------------------------
    attitude_controller_init(&attitude_controller,
                             config_.attitude_controller_config,
                             &ahrs,
                             &command.attitude,
                             &command.rate,
                             &command.torque);

    //--------------------------------------------------------------------------
    // Init altitude estimation
    //--------------------------------------------------------------------------
    altitude_estimation_.init();


    //--------------------------------------------------------------------------
    // Init altitude controller
    //--------------------------------------------------------------------------
    altitude_controller_.init();

    //--------------------------------------------------------------------------
    // Init velocity controller
    //--------------------------------------------------------------------------
    velocity_controller_copter_init(&velocity_controller,
                                    config_.velocity_controller_copter_config,
                                    &ahrs,
                                    &position_estimation,
                                    &command.velocity,
                                    &command.attitude,
                                    &command.thrust);

    //--------------------------------------------------------------------------
    // Init vector field navigation
    //--------------------------------------------------------------------------
    vector_field_waypoint_conf_t vector_field_config;
    vector_field_waypoint_init(&vector_field_waypoint,
                               &vector_field_config,
                               &waypoint_handler,
                               &position_estimation,
                               &command.velocity);

    print_util_dbg_sep('-');
    time_keeper_delay_ms(50);
    print_util_dbg_init_msg("[CENTRAL_DATA]", init_success);
    time_keeper_delay_ms(50);
    print_util_dbg_sep('-');
    time_keeper_delay_ms(50);

    return init_success;
}
