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
#include "control/stabilisation_copter_default_config.hpp"
#include "communication/remote_default_config.hpp"
#include "control/manual_control_default_config.hpp"
#include "control/attitude_controller_default_config.h"
#include "control/velocity_controller_copter_default_config.h"
#include "control/servos_mix_quadcopter_diag_default_config.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "sensing/qfilter_default_config.h"

#include "util/print_util.h"
}


Central_data::Central_data(Imu& imu, Barometer& barometer, Gps& gps, Sonar& sonar, Serial& serial_mavlink, Satellite& satellite, Led& led, File& file_flash, Battery& battery, Servo& servo_0, Servo& servo_1, Servo& servo_2, Servo& servo_3, File& file1, File& file2, const central_data_conf_t& config):
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
    state(battery, state_default_config()),
    scheduler(Scheduler::default_config()),
    mavlink_communication(serial_mavlink, state, file_flash, config.mavlink_communication_config),
    ahrs(ahrs_initialized()),
    manual_control(&satellite, manual_control_default_config(), remote_default_config()),
    position_estimation(state, barometer, sonar, gps, ahrs),
    navigation(&controls_nav, &ahrs.qe, &position_estimation, &state, &mavlink_communication),
    waypoint_handler(position_estimation, navigation, ahrs, state, manual_control, mavlink_communication, mavlink_communication.get_mavlink_stream()),
    data_logging(file1, state, data_logging_default_config()),
    data_logging2(file2, state, data_logging_default_config()),
    altitude_estimation_(sonar, barometer, ahrs, altitude_),
    altitude_controller_(command.position, altitude_, command.thrust),
    sysid_(mavlink_communication.get_sysid())
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
    print_util_dbg_init_msg("[SCHEDULER]", true);
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
    // Init qfilter
    // -------------------------------------------------------------------------
    ret = qfilter_init(&attitude_filter,
                       qfilter_default_config(),
                       &imu,
                       &ahrs);
    print_util_dbg_init_msg("[QFILTER]", ret);
    init_success &= ret;
    time_keeper_delay_ms(50);


    // -------------------------------------------------------------------------
    // Init stabilisers
    // -------------------------------------------------------------------------
    ret = stabilisation_copter_init(&stabilisation_copter,
                                    stabilisation_copter_default_config(),
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
                                          servos_mix_quadcopter_diag_default_config(),
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

    //--------------------------------------------------------------------------
    // Init attitude controller
    //--------------------------------------------------------------------------
    attitude_controller_init(&attitude_controller,
                             attitude_controller_default_config(),
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
    velocity_controller_copter_conf_t velocity_controller_copter_config = velocity_controller_copter_default_config();
    velocity_controller_copter_init(&velocity_controller,
                                    velocity_controller_copter_config,
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
