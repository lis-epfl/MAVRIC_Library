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


#include "sample_projects/LEWing/central_data.hpp"
#include "control/stabilisation_wing_default_config.hpp"
#include "communication/mavlink_communication_default_config.hpp"

#include "sensing/position_estimation_default_config.hpp"
#include "sensing/ahrs_madgwick_default_config.hpp"
#include "communication/remote_default_config.hpp"
#include "control/manual_control_default_config.hpp"
#include "control/stabilisation_wing.hpp"
#include "control/servos_mix_wing_default_config.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "control/navigation_default_config.h"
#include "sensing/qfilter_default_config.h"
#include "runtime/scheduler_default_config.h"
#include "util/print_util.h"
}


Central_data::Central_data(uint8_t sysid, Imu& imu, Barometer& barometer, Gps& gps, Sonar& sonar, Serial& serial_mavlink, Satellite& satellite, Led& led, File& file_flash, Battery& battery, Servo& servo_0, Servo& servo_1, Servo& servo_2, Servo& servo_3, Airspeed_analog& airspeed_analog, File& file1, File& file2):
    imu(imu),
    barometer(barometer),
    gps(gps),
    sonar(sonar),
    serial_mavlink(serial_mavlink),
    satellite(satellite),
    led(led),
    file_flash(file_flash),
    battery(battery),
    servo_0(servo_0),
    servo_1(servo_1),
    servo_2(servo_2),
    servo_3(servo_3),
    airspeed_analog(airspeed_analog),
    state(battery, state_default_config()),
    data_logging(file1, state, data_logging_default_config()),
    data_logging2(file2, state, data_logging_default_config()),
    sysid_(sysid)
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
    ret = scheduler_init(&scheduler, scheduler_default_config());
    print_util_dbg_init_msg("[SCHEDULER]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init mavlink communication
    // -------------------------------------------------------------------------
    mavlink_communication_conf_t mavlink_communication_config = mavlink_communication_default_config();
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
    // Init attitude estimation filter
    // -------------------------------------------------------------------------
    ret = ahrs_madgwick_init(&attitude_filter,
                             ahrs_madgwick_default_config(),
                             &imu,
                             &ahrs,
                             &airspeed_analog);
    print_util_dbg_init_msg("[MADGWICK]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init position_estimation_init
    // -------------------------------------------------------------------------
    ret = position_estimation_init(&position_estimation,
                                   position_estimation_default_config(),
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
    navigation_config_t nav_config = navigation_default_config();
    nav_config.navigation_type = DUBIN;
    ret = navigation_init(&navigation,
                          nav_config,
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
    ret = stabilisation_wing_init(&stabilisation_wing,
                                    stabilisation_wing_default_config(),
                                    &controls,
                                    &command.torque,
                                    &command.thrust,
                                    &imu,
                                    &ahrs,
                                    &position_estimation,
                                    &airspeed_analog,
                                    &navigation);
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
    ret = servos_mix_wing_init(  &servo_mix,
                                servos_mix_wing_default_config(),
                                &command.torque,
                                &command.thrust,
                                &servo_1,
                                &servo_2,
                                &servo_0);
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
                              manual_control_default_config(),
                              remote_default_config());
    print_util_dbg_init_msg("[MANUAL CTRL]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);

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
