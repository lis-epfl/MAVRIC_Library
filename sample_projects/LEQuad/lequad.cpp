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
 * \file lequad.c
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/


#include "sample_projects/LEQuad/lequad.hpp"

#include "communication/data_logging_telemetry.hpp"
#include "communication/state_telemetry.hpp"

#include "drivers/barometer_telemetry.hpp"
#include "drivers/gps_telemetry.hpp"

#include "sensing/imu_telemetry.hpp"
#include "sensing/ahrs_telemetry.hpp"
#include "sensing/position_estimation_telemetry.hpp"

#include "control/manual_control_telemetry.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
}



LEQuad::LEQuad(Imu& imu, Barometer& barometer, Gps& gps, Sonar& sonar, Serial& serial_mavlink, Satellite& satellite, Led& led, File& file_flash, Battery& battery, Servo& servo_0, Servo& servo_1, Servo& servo_2, Servo& servo_3, Servo& servo_4, Servo& servo_5, Servo& servo_6, Servo& servo_7, File& file1, File& file2, const conf_t& config):
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
    servo_4(servo_4),
    servo_5(servo_5),
    servo_6(servo_6),
    servo_7(servo_7),
    manual_control(&satellite, config.manual_control_config, config.remote_config),
    state(mavlink_communication.mavlink_stream(), battery, config.state_config),
    scheduler(Scheduler::default_config()),
    mavlink_communication(serial_mavlink, state, file_flash, config.mavlink_communication_config),
    ahrs(ahrs_initialized()),
    ahrs_ekf(imu, ahrs, config.ahrs_ekf_config),
    position_estimation(state, barometer, sonar, gps, ahrs),
    navigation(controls_nav, ahrs.qe, position_estimation, state, mavlink_communication.mavlink_stream(), config.navigation_config),
    waypoint_handler(position_estimation, navigation, ahrs, state, manual_control, mavlink_communication.message_handler(), mavlink_communication.mavlink_stream(), config.waypoint_handler_config),
    state_machine(state, position_estimation, imu, ahrs, manual_control),
    data_logging_continuous(file1, state, config.data_logging_continuous_config),
    data_logging_stat(file2, state, config.data_logging_stat_config),
    sysid_(mavlink_communication.sysid()),
    config_(config)
{}


bool LEQuad::init(void)
{
    bool ret = true;
    print_util_dbg_sep('%');
    time_keeper_delay_ms(50);
    print_util_dbg_sep('-');
    time_keeper_delay_ms(50);
    print_util_dbg_print("[CENTRAL_DATA] ...\r\n");
    time_keeper_delay_ms(50);
    print_util_dbg_sep('-');

    // Init al modules
    init_state();
    init_data_logging();
    init_gps();
    init_imu();
    init_attitude_estimation();
    init_position_estimation();
    init_stabilisers();
    init_navigation();
    init_hud();
    init_servos();
    init_barometer();
    init_manual_control();

    // sort communication tasks
    ret &= mavlink_communication.scheduler().sort_tasks();

    // sort main tasks
    ret &= scheduler.sort_tasks();

    print_util_dbg_sep('-');
    time_keeper_delay_ms(50);
    print_util_dbg_init_msg("[CENTRAL_DATA]", ret);
    time_keeper_delay_ms(50);
    print_util_dbg_sep('-');
    time_keeper_delay_ms(50);

    return ret;
}


// -------------------------------------------------------------------------
// State
// -------------------------------------------------------------------------
bool LEQuad::init_state(void)
{
    bool ret = true;

    // UP telemetry
    ret &= state_telemetry_init(&state, mavlink_communication.p_message_handler());

    // DOWN telemetry
    ret &= mavlink_communication.add_msg_send(1000000,
                                              Scheduler_task::RUN_REGULAR,
                                              Scheduler_task::PERIODIC_ABSOLUTE,
                                              Scheduler_task::PRIORITY_NORMAL,
                                              (Mavlink_communication::send_msg_function_t)&state_telemetry_send_heartbeat,
                                              &state,
                                              MAVLINK_MSG_ID_HEARTBEAT);   // ID 0
    ret &= mavlink_communication.add_msg_send(1000000,
                                              Scheduler_task::RUN_REGULAR,
                                              Scheduler_task::PERIODIC_ABSOLUTE,
                                              Scheduler_task::PRIORITY_NORMAL,
                                              (Mavlink_communication::send_msg_function_t)&state_telemetry_send_status,
                                              &state,
                                              MAVLINK_MSG_ID_SYS_STATUS);   // ID 1

    // Data logging
    ret &= data_logging_stat.add_field((uint32_t*)&state.mav_state_, "mav_state");
    ret &= data_logging_stat.add_field(&state.mav_mode_,             "mav_mode");


    return ret;
}


// -------------------------------------------------------------------------
// Data logging
// -------------------------------------------------------------------------
bool LEQuad::init_data_logging(void)
{
    bool ret = true;

    // Module
    ret &= data_logging_continuous.create_new_log_file("Log_file", true, mavlink_communication.sysid());
    ret &= data_logging_stat.create_new_log_file("Log_Stat", false, mavlink_communication.sysid());

    // UP telemetry
    ret &= data_logging_telemetry_init(&data_logging_continuous, mavlink_communication.p_message_handler());
    ret &= data_logging_telemetry_init(&data_logging_stat, mavlink_communication.p_message_handler());

    return ret;
}


// -------------------------------------------------------------------------
// GPS
// -------------------------------------------------------------------------
bool LEQuad::init_gps(void)
{
    bool ret = true;

    // UP telemetry
    ret &= gps_telemetry_init(&gps, mavlink_communication.p_message_handler());

    // DOWN telemetry
    ret &= mavlink_communication.add_msg_send(1000000,
                                              Scheduler_task::RUN_REGULAR,
                                              Scheduler_task::PERIODIC_ABSOLUTE,
                                              Scheduler_task::PRIORITY_NORMAL,
                                              (Mavlink_communication::send_msg_function_t)&gps_telemetry_send_raw,
                                              &gps,
                                              MAVLINK_MSG_ID_GPS_RAW_INT);   // ID 24


    return ret;
}


// -------------------------------------------------------------------------
// IMU
// -------------------------------------------------------------------------
bool LEQuad::init_imu(void)
{
    bool ret = true;

    // UP telemetry
    ret &= imu_telemetry_init(&imu, mavlink_communication.p_message_handler());

    // DOWN telemetry
    ret &= mavlink_communication.add_msg_send(250000,
                                              Scheduler_task::RUN_REGULAR,
                                              Scheduler_task::PERIODIC_ABSOLUTE,
                                              Scheduler_task::PRIORITY_NORMAL,
                                              (Mavlink_communication::send_msg_function_t)&imu_telemetry_send_scaled,
                                              &imu,
                                              MAVLINK_MSG_ID_SCALED_IMU);   // ID 26


    // Parameters
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&imu.get_config()->gyroscope.bias[X],     "BIAS_GYRO_X");
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&imu.get_config()->gyroscope.bias[Y],     "BIAS_GYRO_Y");
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&imu.get_config()->gyroscope.bias[Z],     "BIAS_GYRO_Z");
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&imu.get_config()->accelerometer.bias[X], "BIAS_ACC_X");
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&imu.get_config()->accelerometer.bias[Y], "BIAS_ACC_Y");
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&imu.get_config()->accelerometer.bias[Z], "BIAS_ACC_Z");
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&imu.get_config()->magnetometer.bias[X],  "BIAS_MAG_X");
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&imu.get_config()->magnetometer.bias[Y],  "BIAS_MAG_Y");
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&imu.get_config()->magnetometer.bias[Z],  "BIAS_MAG_Z");
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&imu.get_config()->magnetic_north[X],     "NORTH_MAG_X");
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&imu.get_config()->magnetic_north[Y],     "NORTH_MAG_Y");
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&imu.get_config()->magnetic_north[Z],     "NORTH_MAG_Z");

    return ret;
}


// -------------------------------------------------------------------------
// Attitude estimation
// -------------------------------------------------------------------------
bool LEQuad::init_attitude_estimation(void)
{
    bool ret = true;
    ret &= mavlink_communication.add_msg_send(200000,
                                              Scheduler_task::RUN_REGULAR,
                                              Scheduler_task::PERIODIC_ABSOLUTE,
                                              Scheduler_task::PRIORITY_NORMAL,
                                              (Mavlink_communication::send_msg_function_t)&ahrs_telemetry_send_attitude,
                                              &ahrs,
                                              MAVLINK_MSG_ID_ATTITUDE);    // ID 30
    ret &= mavlink_communication.add_msg_send(500000,
                                              Scheduler_task::RUN_REGULAR,
                                              Scheduler_task::PERIODIC_ABSOLUTE,
                                              Scheduler_task::PRIORITY_NORMAL,
                                              (Mavlink_communication::send_msg_function_t)&ahrs_telemetry_send_attitude_quaternion,
                                              &ahrs,
                                              MAVLINK_MSG_ID_ATTITUDE_QUATERNION); // ID 31

    return ret;
}


// -------------------------------------------------------------------------
// Position estimation
// -------------------------------------------------------------------------
bool LEQuad::init_position_estimation(void)
{
    bool ret = true;
    // UP telemetry
    ret &= position_estimation_telemetry_init(&position_estimation, mavlink_communication.p_message_handler());

    // DOWN telemetry
    ret &= mavlink_communication.add_msg_send(500000,
                                              Scheduler_task::RUN_REGULAR,
                                              Scheduler_task::PERIODIC_ABSOLUTE,
                                              Scheduler_task::PRIORITY_NORMAL,
                                              (Mavlink_communication::send_msg_function_t)&position_estimation_telemetry_send_position,
                                              &position_estimation,
                                              MAVLINK_MSG_ID_LOCAL_POSITION_NED); // ID 32
    ret &= mavlink_communication.add_msg_send(250000,
                                              Scheduler_task::RUN_REGULAR,
                                              Scheduler_task::PERIODIC_ABSOLUTE,
                                              Scheduler_task::PRIORITY_NORMAL,
                                              (Mavlink_communication::send_msg_function_t)&position_estimation_telemetry_send_global_position,
                                              &position_estimation,
                                              MAVLINK_MSG_ID_GLOBAL_POSITION_INT); // ID 33

    // Parameters
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&position_estimation.kp_alt_baro,   "POS_KP_ALT_BARO" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&position_estimation.kp_vel_baro,   "POS_KP_VELB"     );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&position_estimation.kp_pos_gps[0], "POS_KP_POS0"     );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&position_estimation.kp_pos_gps[1], "POS_KP_POS1"     );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&position_estimation.kp_pos_gps[2], "POS_KP_POS2"     );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&position_estimation.kp_vel_gps[0], "POS_KP_VEL0"     );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&position_estimation.kp_vel_gps[1], "POS_KP_VEL1"     );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&position_estimation.kp_vel_gps[2], "POS_KP_VEL2"     );

    // Data logging
    ret &= data_logging_continuous.add_field(&position_estimation.local_position.pos[0], "local_x", 3);
    ret &= data_logging_continuous.add_field(&position_estimation.local_position.pos[1], "local_y", 3);
    ret &= data_logging_continuous.add_field(&position_estimation.local_position.pos[2], "local_z", 3);
    ret &= data_logging_stat.add_field(&position_estimation.local_position.origin.latitude,  "origin_lat", 7);
    ret &= data_logging_stat.add_field(&position_estimation.local_position.origin.longitude, "origin_lon", 7);
    ret &= data_logging_stat.add_field(&position_estimation.local_position.origin.altitude,  "origin_alt", 3);

    return ret;
}

// -------------------------------------------------------------------------
// Stabilisers
// -------------------------------------------------------------------------
bool LEQuad::init_stabilisers(void)
{
    bool ret = true;

    // -------------------------------------------------------------------------
    // Stabilisation copter
    // -------------------------------------------------------------------------
    // Module
    ret &= stabilisation_copter_init(&stabilisation_copter,
                                    config_.stabilisation_copter_config,
                                    &controls,
                                    &ahrs,
                                    &position_estimation,
                                    &command.torque,
                                    &command.thrust);
    ret &= stabilisation_init(&controls);

    // Parameters
    Onboard_parameters& op            = mavlink_communication.onboard_parameters();
    stabiliser_t* rate_stabiliser     = &stabilisation_copter.stabiliser_stack.rate_stabiliser;
    stabiliser_t* attitude_stabiliser = &stabilisation_copter.stabiliser_stack.attitude_stabiliser;
    stabiliser_t* velocity_stabiliser = &stabilisation_copter.stabiliser_stack.velocity_stabiliser;
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].p_gain,                    "ROLL_R_KP");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.clip,           "ROLL_R_I_CLIP");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.gain,           "ROLL_R_KI");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.clip,       "ROLL_R_D_CLIP");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.gain,       "ROLL_R_KD");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].p_gain,                "ROLL_A_KP");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.clip,       "ROLL_A_I_CLIP");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.gain,       "ROLL_A_KI");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.clip,   "ROLL_A_D_CLIP");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.gain,   "ROLL_A_KD");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].p_gain,                   "PITCH_R_KP");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.clip,          "PITCH_R_I_CLIP");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.gain,          "PITCH_R_KI");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.clip,      "PITCH_R_D_CLIP");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.gain,      "PITCH_R_KD");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].p_gain,               "PITCH_A_KP");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.clip,      "PITCH_A_I_CLIP");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.gain,      "PITCH_A_KI");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.clip,  "PITCH_A_D_CLIP");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.gain,  "PITCH_A_KD");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[YAW].p_gain,                     "YAW_R_KP");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[YAW].clip_max,                   "YAW_R_P_CLMX");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[YAW].clip_min,                   "YAW_R_P_CLMN");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.clip,            "YAW_R_I_CLIP");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.gain,            "YAW_R_KI");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.clip,        "YAW_R_D_CLIP");
    ret &= op.add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.gain,        "YAW_R_KD");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].p_gain,                 "YAW_A_KP");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].clip_max,               "YAW_A_P_CLMX");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].clip_min,               "YAW_A_P_CLMN");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.clip,        "YAW_A_I_CLIP");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.gain,        "YAW_A_KI");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.clip,    "YAW_A_D_CLIP");
    ret &= op.add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.gain,    "YAW_A_KD");
    ret &= op.add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].p_gain,                "ROLL_V_KP");
    ret &= op.add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].integrator.clip_pre,   "ROLL_V_I_CLPRE");
    ret &= op.add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].integrator.gain,       "ROLL_V_KI");
    ret &= op.add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].integrator.clip,       "ROLL_V_I_CLIP");
    ret &= op.add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].differentiator.gain,   "ROLL_V_KD");
    ret &= op.add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].p_gain,               "PITCH_V_KP");
    ret &= op.add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].integrator.clip_pre,  "PITCH_V_I_CLPRE");
    ret &= op.add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].integrator.gain,      "PITCH_V_KI");
    ret &= op.add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].integrator.clip,      "PITCH_V_I_CLIP");
    ret &= op.add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].differentiator.gain,  "PITCH_V_KD");
    ret &= op.add_parameter_float(&velocity_stabiliser->thrust_controller.p_gain,                   "THRV_KP");
    ret &= op.add_parameter_float(&velocity_stabiliser->thrust_controller.integrator.clip_pre,      "THRV_I_PREG");
    ret &= op.add_parameter_float(&velocity_stabiliser->thrust_controller.differentiator.gain,      "THRV_KD");
    ret &= op.add_parameter_float(&velocity_stabiliser->thrust_controller.soft_zone_width,          "THRV_SOFT");

    return ret;
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


// -------------------------------------------------------------------------
// Navigation
// -------------------------------------------------------------------------
bool LEQuad::init_navigation(void)
{
    bool ret = true;

    // Parameters
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.dist2vel_gain,                           "NAV_DIST2VEL"    );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.cruise_speed,                            "NAV_CRUISESPEED" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.max_climb_rate,                          "NAV_CLIMBRATE"   );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.soft_zone_size,                          "NAV_SOFTZONE"    );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.hovering_controller.p_gain,              "NAV_HOVER_PGAIN" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.hovering_controller.differentiator.gain, "NAV_HOVER_DGAIN" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.wpt_nav_controller.p_gain,               "NAV_WPT_PGAIN"   );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.wpt_nav_controller.differentiator.gain,  "NAV_WPT_DGAIN"   );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&navigation.kp_yaw,                                  "NAV_YAW_KPGAIN"   );

    return ret;
}


// -------------------------------------------------------------------------
// HUD
// -------------------------------------------------------------------------
bool LEQuad::init_hud(void)
{
    bool ret = true;

    // Module
    ret &= hud_telemetry_init(&hud_structure, &position_estimation, &controls, &ahrs);

    // DOWN telemetry
    ret &= mavlink_communication.add_msg_send(500000,
                                              Scheduler_task::RUN_REGULAR,
                                              Scheduler_task::PERIODIC_ABSOLUTE,
                                              Scheduler_task::PRIORITY_NORMAL,
                                              (Mavlink_communication::send_msg_function_t)&hud_telemetry_send_message,
                                              &hud_structure,
                                              MAVLINK_MSG_ID_VFR_HUD);    // ID 74

    return ret;
}

// -------------------------------------------------------------------------
// Servos
// -------------------------------------------------------------------------
bool LEQuad::init_servos(void)
{
    bool ret = true;

    // Module
    ret = servos_mix_quadcotper_diag_init(&servo_mix,
                                          config_.servos_mix_quadcopter_diag_config,
                                          &command.torque, &command.thrust,
                                          &servo_0, &servo_1, &servo_2, &servo_3);

    // DOWN telemetry
    servos_telemetry_init(&servos_telemetry,
                          &servo_0, &servo_1, &servo_2, &servo_3,
                          &servo_4, &servo_5, &servo_6, &servo_7);
    ret &= mavlink_communication.add_msg_send(1000000,
                                              Scheduler_task::RUN_REGULAR,
                                              Scheduler_task::PERIODIC_ABSOLUTE,
                                              Scheduler_task::PRIORITY_NORMAL,
                                              (Mavlink_communication::send_msg_function_t)&servos_telemetry_mavlink_send,
                                              &servos_telemetry,
                                              MAVLINK_MSG_ID_SERVO_OUTPUT_RAW);  // ID 36

    return ret;
}

// -------------------------------------------------------------------------
// Barometer
// -------------------------------------------------------------------------
bool LEQuad::init_barometer(void)
{
    bool ret = true;

    // DOWN telemetry
    ret &= mavlink_communication.add_msg_send(500000,
                                              Scheduler_task::RUN_REGULAR,
                                              Scheduler_task::PERIODIC_ABSOLUTE,
                                              Scheduler_task::PRIORITY_NORMAL,
                                              (Mavlink_communication::send_msg_function_t)&barometer_telemetry_send,
                                              &barometer,
                                              MAVLINK_MSG_ID_SCALED_PRESSURE);  // ID 29
    return ret;
}

// -------------------------------------------------------------------------
// Manual control
// -------------------------------------------------------------------------
bool LEQuad::init_manual_control(void)
{
    bool ret = true;

    // UP telemetry
    ret &= manual_control_telemetry_init(&manual_control, mavlink_communication.p_message_handler());

    // DOWN telemetry
    ret &= mavlink_communication.add_msg_send(500000,
                                              Scheduler_task::RUN_REGULAR,
                                              Scheduler_task::PERIODIC_ABSOLUTE,
                                              Scheduler_task::PRIORITY_NORMAL,
                                              (Mavlink_communication::send_msg_function_t)&manual_control_telemetry_send,
                                              &manual_control,
                                              MAVLINK_MSG_ID_MANUAL_CONTROL); // ID 34

    // Parameters
    /* WARNING the following 2 cast are necessary on stm32 architecture, otherwise it leads to execution error */
    ret &= mavlink_communication.onboard_parameters().add_parameter_int32((int32_t*) &manual_control.control_source_, "CTRL_CTRL_SRC");
    ret &= mavlink_communication.onboard_parameters().add_parameter_int32((int32_t*) &manual_control.mode_source_,    "COM_RC_IN_MODE");

    return ret;
}
