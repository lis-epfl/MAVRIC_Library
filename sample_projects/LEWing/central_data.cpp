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

#include "control/stabilisation_wing.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
}


Central_data::Central_data(Imu& imu, Barometer& barometer, Gps& gps, Sonar& sonar, Serial& serial_mavlink, Satellite& satellite, Led& led, File& file_flash, Battery& battery, Servo& servo_0, Servo& servo_1, Servo& servo_2, Servo& servo_3, Airspeed_analog& airspeed_analog, File& file1, File& file2, const conf_t& config):
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
    manual_control(&satellite, config.manual_control_config, config.remote_config),    
    state(mavlink_communication.mavlink_stream(), battery, config.state_config),
    airspeed_analog(airspeed_analog),
    scheduler(Scheduler::default_config()),
    mavlink_communication(serial_mavlink, state, file_flash, config.mavlink_communication_config),
    ahrs(ahrs_initialized()),
    position_estimation(state, barometer, sonar, gps, ahrs),
    navigation(controls_nav, ahrs.qe, position_estimation, state, mavlink_communication.mavlink_stream(), config.navigation_config),
    waypoint_handler(position_estimation, navigation, ahrs, state, manual_control, mavlink_communication.message_handler(), mavlink_communication.mavlink_stream()),
    state_machine(state, position_estimation, imu, manual_control),
    data_logging(file1, state, data_logging_default_config()),
    data_logging2(file2, state, data_logging_default_config()),
    sysid_(mavlink_communication.sysid()),
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
    // Init attitude estimation filter
    // -------------------------------------------------------------------------
    ret = ahrs_madgwick_init(&attitude_filter,
                             config_.ahrs_madgwick_config,
                             &imu,
                             &ahrs,
                             &airspeed_analog);
    print_util_dbg_init_msg("[MADGWICK]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);

    // -------------------------------------------------------------------------
    // Init stabilisers
    // -------------------------------------------------------------------------
    ret = stabilisation_wing_init(&stabilisation_wing,
                                    config_.stabilisation_wing_config,
                                    &controls,
                                    &command.torque,
                                    &command.thrust,
                                    &imu,
                                    &ahrs,
                                    &position_estimation,
                                    &airspeed_analog,
                                    &navigation,
                                    &gps);
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
                                config_.servos_mix_wing_config,
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

    //--------------------------------------------------------------------------
    // Init vector field navigation
    //--------------------------------------------------------------------------
    /*vector_field_waypoint_conf_t vector_field_config;
    vector_field_waypoint_init(&vector_field_waypoint,
                               &vector_field_config,
                               &waypoint_handler,
                               &position_estimation,
                               &command.velocity);*/

    print_util_dbg_sep('-');
    time_keeper_delay_ms(50);
    print_util_dbg_init_msg("[CENTRAL_DATA]", init_success);
    time_keeper_delay_ms(50);
    print_util_dbg_sep('-');
    time_keeper_delay_ms(50);

    return init_success;
}
