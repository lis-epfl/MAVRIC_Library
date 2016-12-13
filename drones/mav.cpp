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
 * \file lequad.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/


#include "drones/lequad.hpp"

#include "communication/data_logging_telemetry.hpp"
#include "status/state_telemetry.hpp"

#include "drivers/barometer_telemetry.hpp"
#include "drivers/gps_telemetry.hpp"
#include "drivers/sonar_telemetry.hpp"
#include "drivers/px4flow_telemetry.hpp"

#include "sensing/imu_telemetry.hpp"
#include "sensing/ins_telemetry.hpp"
#include "sensing/ahrs_telemetry.hpp"

#include "manual_control/manual_control_telemetry.hpp"

#include "runtime/scheduler_telemetry.hpp"

#include "hal/common/time_keeper.hpp"

#include "util/print_util.hpp"


MAV::MAV(  Imu& imu,
           Barometer& barometer,
           Gps& gps,
           Sonar& sonar,
           PX4Flow& flow,
           Serial& serial_mavlink,
           Satellite& satellite,
           State_display& state_display,
           File& file_flash,
           Battery& battery,
           File& file1,
           File& file2,
           Flight_controller& flight_controller,
           const conf_t& config):
    imu(imu),
    barometer(barometer),
    gps(gps),
    sonar(sonar),
    flow(flow),
    serial_mavlink(serial_mavlink),
    satellite(satellite),
    state_display_(state_display),
    file_flash(file_flash),
    battery(battery),
    gps_mocap(communication.handler()),
    gps_hub(std::array<Gps*,2>{{&gps, &gps_mocap}}),
    ahrs_ekf_mocap(communication.handler(), ahrs_ekf),
    manual_control(&satellite, config.manual_control_config, config.remote_config),
    state(communication.mavlink_stream(), battery, config.state_config),
    scheduler(config.scheduler_config),
    communication(serial_mavlink, state, file_flash, config.mavlink_communication_config),
    ahrs_(ahrs_ekf),
    ahrs_ekf(imu, config.ahrs_ekf_config),
    ahrs_qfilter(imu, config.qfilter_config),
    ins_(ins_complementary),
    ins_complementary(state, barometer, sonar, gps_hub, flow, ahrs_, config.ins_complementary_config),
    ins_kf(state, gps, gps_mocap, barometer, sonar, flow, ahrs_),
    flight_controller_(flight_controller),
    waypoint_handler(ins_, communication.handler(), communication.mavlink_stream(), mission_handler_registry, config.waypoint_handler_config),
    mission_handler_registry(),
    hold_position_handler(ins_),
    landing_handler(ins_, state),
    navigating_handler(ins_, communication.mavlink_stream(), waypoint_handler),
    on_ground_handler(),
    manual_ctrl_handler(),
    takeoff_handler(ins_, state),
    critical_landing_handler(ins_, state),
    critical_navigating_handler(ins_, communication.mavlink_stream(), waypoint_handler),
    mission_planner_(ins_, ahrs_, state, manual_control, safety_geofence_, communication.handler(), communication.mavlink_stream(), waypoint_handler, mission_handler_registry),
    safety_geofence_(config.safety_geofence_config),
    emergency_geofence_(config.emergency_geofence_config),
    state_machine(state, ins_, imu, ahrs_, manual_control, safety_geofence_, emergency_geofence_, state_display_),
    data_logging_continuous(file1, state, config.data_logging_continuous_config),
    data_logging_stat(file2, state, config.data_logging_stat_config),
    sysid_(communication.sysid()),
    config_(config)
{}


bool MAV::init(void)
{
    bool success = true;

    // Init main task first
    success &= init_main_task();

    // Init all modules
    success &= init_state();
    success &= init_communication();
    success &= init_data_logging();
    success &= init_gps();
    success &= init_imu();
    success &= init_barometer();
    success &= init_sonar();
    success &= init_ahrs();
    success &= init_ins();
    success &= init_mocap();
    success &= init_flow();
    success &= init_mission_planning();
    success &= init_controller();
    success &= init_hud();
    // success &= init_servos();
    success &= init_ground_control();

    return success;
}

void MAV::loop(void)
{
    // Sort tasks
    scheduler.sort_tasks();
    communication.telemetry().sort();

    // Try to read from flash, if unsuccessful, write to flash
    if (communication.parameters().read_from_storage() == false)
    {
        communication.parameters().write_to_storage();
    }

    // Create log files
    data_logging_continuous.create_new_log_file("log", communication.sysid());
    data_logging_stat.create_new_log_file("stat", communication.sysid());

    // Init mav state
    state.mav_state_ = MAV_STATE_STANDBY;  // TODO check if this is necessary

    while (1)
    {
        scheduler.update();
    }
}

// -------------------------------------------------------------------------
// Main task
// -------------------------------------------------------------------------
bool MAV::init_main_task(void)
{
    bool ret = true;

    // Task
    ret &= scheduler.add_task(4000, &MAV::main_task_func, this, Scheduler_task::PRIORITY_HIGHEST);

    // DOWN link
    ret &= communication.telemetry().add<Scheduler>(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,  5000000, &scheduler_telemetry_send_rt_stats, &scheduler);
    ret &= communication.telemetry().add<Scheduler>(MAVLINK_MSG_ID_BIG_DEBUG_VECT,  5000000, &scheduler_telemetry_send_rt_stats_all, &scheduler);

    return ret;
}

// -------------------------------------------------------------------------
// State
// -------------------------------------------------------------------------
bool MAV::init_state(void)
{
    bool ret = true;

    // UP telemetry
    ret &= state_telemetry_init(&state_machine, &communication.handler());

    // DOWN telemetry
    ret &= communication.telemetry().add(MAVLINK_MSG_ID_HEARTBEAT,  1000000, &state_telemetry_send_heartbeat, &state);
    ret &= communication.telemetry().add(MAVLINK_MSG_ID_SYS_STATUS, 1000000, &state_telemetry_send_status,    &state);

    // Data logging
    ret &= data_logging_stat.add_field((uint32_t*)&state.mav_state_,   "mav_state");
    ret &= data_logging_stat.add_field((state.mav_mode_.bits_ptr()),   "mav_mode");

    // Task
    ret &= scheduler.add_task(200000, &State_machine::update_task, &state_machine);
    // Leds blinks at 1, 3 and 6Hz, then smallest half period is 83333us
    ret &= scheduler.add_task( 83333, &task_state_display_update, &state_display_);

    return ret;
}


// -------------------------------------------------------------------------
// Communication
// -------------------------------------------------------------------------
bool MAV::init_communication(void)
{
    bool ret = true;

    // Task
    ret &= scheduler.add_task(4000,  &Mavlink_communication::update_task, &communication);

    return ret;
}


// -------------------------------------------------------------------------
// Data logging
// -------------------------------------------------------------------------
bool MAV::init_data_logging(void)
{
    bool ret = true;

    // UP telemetry
    ret &= data_logging_telemetry_init(&data_logging_continuous, &communication.handler());
    ret &= data_logging_telemetry_init(&data_logging_stat, &communication.handler());

    // Task
    ret &= scheduler.add_task<Data_logging>(10000, &task_data_logging_update, &data_logging_continuous);
    ret &= scheduler.add_task<Data_logging>(10000, &task_data_logging_update, &data_logging_stat);

    return ret;
}


// -------------------------------------------------------------------------
// GPS
// -------------------------------------------------------------------------
bool MAV::init_gps(void)
{
    bool ret = true;

    // UP telemetry
    ret &= gps_telemetry_init(&gps_hub, &communication.handler());

    // DOWN telemetry
    ret &= communication.telemetry().add<Gps>(MAVLINK_MSG_ID_GPS_RAW_INT, 1000000, &gps_telemetry_send_raw,  &gps_hub);

    // Task
    ret &= scheduler.add_task<Gps>(100000, &task_gps_update, &gps_hub, Scheduler_task::PRIORITY_HIGH);

    return ret;
}


// -------------------------------------------------------------------------
// IMU
// -------------------------------------------------------------------------
bool MAV::init_imu(void)
{
    bool ret = true;

    // UP telemetry
    ret &= imu_telemetry_init(&imu, &communication.handler());

    // DOWN telemetry
    ret &= communication.telemetry().add(MAVLINK_MSG_ID_SCALED_IMU, 250000, &imu_telemetry_send_scaled, &imu);

    // Parameters
    ret &= communication.parameters().add(&imu.get_config()->gyroscope.bias[X],     "BIAS_GYRO_X");
    ret &= communication.parameters().add(&imu.get_config()->gyroscope.bias[Y],     "BIAS_GYRO_Y");
    ret &= communication.parameters().add(&imu.get_config()->gyroscope.bias[Z],     "BIAS_GYRO_Z");
    ret &= communication.parameters().add(&imu.get_config()->accelerometer.bias[X], "BIAS_ACC_X");
    ret &= communication.parameters().add(&imu.get_config()->accelerometer.bias[Y], "BIAS_ACC_Y");
    ret &= communication.parameters().add(&imu.get_config()->accelerometer.bias[Z], "BIAS_ACC_Z");
    ret &= communication.parameters().add(&imu.get_config()->magnetometer.bias[X],  "BIAS_MAG_X");
    ret &= communication.parameters().add(&imu.get_config()->magnetometer.bias[Y],  "BIAS_MAG_Y");
    ret &= communication.parameters().add(&imu.get_config()->magnetometer.bias[Z],  "BIAS_MAG_Z");
    ret &= communication.parameters().add(&imu.get_config()->magnetic_north[X],     "NORTH_MAG_X");
    ret &= communication.parameters().add(&imu.get_config()->magnetic_north[Y],     "NORTH_MAG_Y");
    ret &= communication.parameters().add(&imu.get_config()->magnetic_north[Z],     "NORTH_MAG_Z");

    return ret;
}


// -------------------------------------------------------------------------
// Barometer
// -------------------------------------------------------------------------
bool MAV::init_barometer(void)
{
    bool ret = true;

    // DOWN telemetry
    ret &= communication.telemetry().add(MAVLINK_MSG_ID_SCALED_PRESSURE, 250000, &barometer_telemetry_send, &barometer);

    // Task
    ret &= scheduler.add_task(15000, &task_barometer_update, &barometer, Scheduler_task::PRIORITY_HIGH, Scheduler_task::PERIODIC_RELATIVE);

    return ret;
}


// -------------------------------------------------------------------------
// Sonar
// -------------------------------------------------------------------------
bool MAV::init_sonar(void)
{
    bool ret = true;

    // DOWN telemetry
    ret &= communication.telemetry().add(MAVLINK_MSG_ID_DISTANCE_SENSOR, 250000, &sonar_telemetry_send, &sonar);


    // Task
    ret &= scheduler.add_task(100000, &task_sonar_update, &sonar, Scheduler_task::PRIORITY_HIGH);

    return ret;
}


// -------------------------------------------------------------------------
// ahrs_
// -------------------------------------------------------------------------
bool MAV::init_ahrs(void)
{
    bool ret = true;

    // DOWN telemetry
    ret &= communication.telemetry().add(MAVLINK_MSG_ID_ATTITUDE,            500000, &ahrs_telemetry_send_attitude,            &ahrs_);
    ret &= communication.telemetry().add(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 500000, &ahrs_telemetry_send_attitude_quaternion, &ahrs_);

    // Parameters
    ret &= communication.parameters().add(&ahrs_ekf.config_.use_accelerometer,  "AHRS_USE_ACC"  );
    ret &= communication.parameters().add(&ahrs_ekf.config_.use_magnetometer,   "AHRS_USE_MAG"  );
    ret &= communication.parameters().add(&ahrs_ekf.config_.sigma_w_sqr,        "AHRS_SIG_GYRO" );
    ret &= communication.parameters().add(&ahrs_ekf.config_.sigma_r_sqr,        "AHRS_SIG_QUAT" );
    ret &= communication.parameters().add(&ahrs_ekf.config_.R_acc,              "AHRS_R_ACC"    );
    ret &= communication.parameters().add(&ahrs_ekf.config_.R_acc_norm,         "AHRS_R_ACCNORM");
    ret &= communication.parameters().add(&ahrs_ekf.config_.R_mag,              "AHRS_R_MAG");

    return ret;
}


// -------------------------------------------------------------------------
// INS
// -------------------------------------------------------------------------
bool MAV::init_ins(void)
{
    bool ret = true;

    // -------------------------------------------------------------------------
    // Via ins_ alias
    // -------------------------------------------------------------------------
    // DOWN telemetry
    ret &= communication.telemetry().add<INS>(MAVLINK_MSG_ID_LOCAL_POSITION_NED,  250000, &ins_telemetry_send_local_position_ned,  &ins_);
    ret &= communication.telemetry().add<INS>(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 250000, &ins_telemetry_send_global_position_int, &ins_);

    // -------------------------------------------------------------------------
    // Position estimation specfic
    // -------------------------------------------------------------------------
    // UP telemetry
    ret &= ins_telemetry_init(&ins_complementary, &communication.handler());
    // Parameters
    ret &= communication.parameters().add(&ins_complementary.config_.kp_gps_XY_pos,     "POS_K_GPS_XY"    );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_gps_Z_pos,      "POS_K_GPS_Z"     );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_gps_XY_vel,     "POS_K_GPS_V_XY"  );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_gps_Z_vel,      "POS_K_GPS_V_Z"   );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_gps_XY_pos_rtk, "POS_K_RTK_XY"    );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_gps_Z_pos_rtk,  "POS_K_RTK_Z"     );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_gps_XY_vel_rtk, "POS_K_RTK_V_XY"  );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_gps_Z_vel_rtk,  "POS_K_RTK_V_Z"   );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_baro_alt,       "POS_K_BARO_Z"    );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_baro_vel,       "POS_K_BARO_V_Z"  );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_sonar_alt,      "POS_K_SONAR_Z"   );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_sonar_vel,      "POS_K_SONAR_V_Z" );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_flow_vel,       "POS_K_OF_V_XY"   );
    ret &= communication.parameters().add(&ins_complementary.config_.kp_acc_bias,       "POS_K_ACC_BIAS"  );
    ret &= communication.parameters().add(&ins_complementary.config_.use_gps,           "POS_USE_GPS"     );
    ret &= communication.parameters().add(&ins_complementary.config_.use_baro,          "POS_USE_BARO"    );
    ret &= communication.parameters().add(&ins_complementary.config_.use_sonar,         "POS_USE_SONAR"   );
    ret &= communication.parameters().add(&ins_complementary.config_.use_flow,          "POS_USE_FLOW"    );
    ret &= communication.parameters().add(&ins_complementary.config_.use_acc_bias,      "POS_USE_ACBIAS"  );

    // -------------------------------------------------------------------------
    // Kalman INS specific
    // -------------------------------------------------------------------------
    // Parameters
    ret &= communication.parameters().add(&ins_kf.config_.sigma_z_gnd,      "INS_X_Z_GND"       );
    ret &= communication.parameters().add(&ins_kf.config_.sigma_bias_acc,   "INS_X_BIAS_ACC"    );
    ret &= communication.parameters().add(&ins_kf.config_.sigma_bias_baro,  "INS_X_BIAS_BARO"   );
    ret &= communication.parameters().add(&ins_kf.config_.sigma_acc,        "INS_U_ACC"         );
    ret &= communication.parameters().add(&ins_kf.config_.sigma_gps_xy,     "INS_Z_POS_XY"      );
    ret &= communication.parameters().add(&ins_kf.config_.sigma_gps_z,      "INS_Z_POS_Z"       );
    ret &= communication.parameters().add(&ins_kf.config_.sigma_gps_velxy,  "INS_Z_VEL_XY"      );
    ret &= communication.parameters().add(&ins_kf.config_.sigma_gps_velz,   "INS_Z_VEL_Z"       );
    ret &= communication.parameters().add(&ins_kf.config_.sigma_gps_mocap,  "INS_Z_MOCAP"       );
    ret &= communication.parameters().add(&ins_kf.config_.sigma_baro,       "INS_Z_BARO"        );
    ret &= communication.parameters().add(&ins_kf.config_.sigma_flow,       "INS_Z_FLOW"        );
    ret &= communication.parameters().add(&ins_kf.config_.sigma_sonar,      "INS_Z_SONAR"       );
    ret &= communication.parameters().add(&ins_kf.init_flag,                "INS_INIT"          );

    return ret;
}


// -------------------------------------------------------------------------
// MOCAP
// -------------------------------------------------------------------------
bool MAV::init_mocap(void)
{
    bool ret = true;

    ret &= gps_mocap.init();
    ret &= ahrs_ekf_mocap.init();

    return ret;
}



// -------------------------------------------------------------------------
// FLOW
// -------------------------------------------------------------------------
bool MAV::init_flow(void)
{
    bool ret = true;

    // DOWN telemetry
    ret &= communication.telemetry().add(MAVLINK_MSG_ID_OPTICAL_FLOW, 200000, &px4flow_telemetry_send, &flow);

    // Task
    ret &= scheduler.add_task(10000, &PX4Flow::update_task, &flow, Scheduler_task::PRIORITY_HIGH);


    return ret;
}


// -------------------------------------------------------------------------
// Navigation
// -------------------------------------------------------------------------
bool MAV::init_mission_planning(void)
{
    bool ret = true;

    // Initialize
    ret &= mission_handler_registry.register_mission_handler(hold_position_handler);
    ret &= mission_handler_registry.register_mission_handler(landing_handler);
    ret &= mission_handler_registry.register_mission_handler(manual_ctrl_handler);
    ret &= mission_handler_registry.register_mission_handler(navigating_handler);
    ret &= mission_handler_registry.register_mission_handler(on_ground_handler);
    ret &= mission_handler_registry.register_mission_handler(takeoff_handler);
    ret &= mission_handler_registry.register_mission_handler(critical_landing_handler);
    ret &= mission_handler_registry.register_mission_handler(critical_navigating_handler);
    ret &= waypoint_handler.init();
    ret &= mission_planner_.init();

    // Parameters
    ret &= communication.parameters().add(&mission_planner_.takeoff_altitude(),                 "NAV_TAKEOFF_ALT" );

    // ret &= communication.parameters().add(&navigation.cruise_speed,                            "NAV_CRUISESPEED" );
    // ret &= communication.parameters().add(&navigation.max_climb_rate,                          "NAV_CLIMBRATE"   );
    // ret &= communication.parameters().add(&navigation.minimal_radius,                          "NAV_MINI_RADIUS" );
    // ret &= communication.parameters().add(&navigation.hovering_controller.p_gain,              "NAV_HOVER_PGAIN" );
    // ret &= communication.parameters().add(&navigation.hovering_controller.differentiator.gain, "NAV_HOVER_DGAIN" );
    // ret &= communication.parameters().add(&navigation.wpt_nav_controller.p_gain,               "NAV_WPT_PGAIN"   );
    // ret &= communication.parameters().add(&navigation.wpt_nav_controller.differentiator.gain,  "NAV_WPT_DGAIN"   );
    // ret &= communication.parameters().add(&navigation.kp_yaw,                                  "NAV_YAW_KPGAIN"  );

    // Task
    ret &= scheduler.add_task(10000, &Mission_planner::update, &mission_planner_, Scheduler_task::PRIORITY_HIGH);

    return ret;
}


// -------------------------------------------------------------------------
// HUD
// -------------------------------------------------------------------------
bool MAV::init_hud(void)
{
    bool ret = true;

    // Module
    ret &= hud_telemetry_init(&hud, &ins_, &flight_controller_.command(), &ahrs_);

    // DOWN telemetry
    ret &= communication.telemetry().add(MAVLINK_MSG_ID_VFR_HUD, 500000, &hud_telemetry_send_message, &hud);

    return ret;
}

// -------------------------------------------------------------------------
// Servos
// -------------------------------------------------------------------------
// bool MAV::init_servos(void)
// {
//     bool ret = true;
//
//     // DOWN telemetry
//     ret &= servos_telemetry_init(&servos_telemetry,
//                                  &servo_0, &servo_1, &servo_2, &servo_3,
//                                  &servo_4, &servo_5, &servo_6, &servo_7);
//     ret &= communication.telemetry().add(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 1000000, &servos_telemetry_mavlink_send, &servos_telemetry);
//
//     return ret;
// }


// -------------------------------------------------------------------------
// Manual control
// -------------------------------------------------------------------------
bool MAV::init_ground_control(void)
{
    bool ret = true;

    // UP telemetry
    ret &= manual_control_telemetry_init(&manual_control, &communication.handler());

    // DOWN telemetry
    ret &= communication.telemetry().add(MAVLINK_MSG_ID_MANUAL_CONTROL, 500000, &manual_control_telemetry_send, &manual_control);

    // Parameters
    /* WARNING the following 2 cast are necessary on stm32 architecture, otherwise it leads to execution error */
    ret &= communication.parameters().add((int32_t*) &manual_control.control_source_, "CTRL_CTRL_SRC");
    ret &= communication.parameters().add((int32_t*) &manual_control.mode_source_,    "COM_RC_IN_MODE");

    // Task
    ret &= scheduler.add_task(20000, &remote_update, &manual_control.remote, Scheduler_task::PRIORITY_HIGH);

    return ret;
}

// -------------------------------------------------------------------------
// Main task
// -------------------------------------------------------------------------
bool MAV::main_task(void)
{
    // Update estimation
    imu.update();
    ahrs_.update();
    ins_.update();

    bool failsafe = false;

    // Get control command
    if (state.is_armed())
    {
        switch (state.mav_mode().ctrl_mode())
        {
            case Mav_mode::AUTO:
            case Mav_mode::POSITION_HOLD:
                flight_controller_.set_flight_command(mission_planner_);
            break;

            case Mav_mode::VELOCITY:
                flight_controller_.set_manual_velocity_command(manual_control);
            break;

            case Mav_mode::ATTITUDE:
                flight_controller_.set_manual_attitude_command(manual_control);
            break;

            case Mav_mode::RATE:
                flight_controller_.set_manual_rate_command(manual_control);
            break;

            default:
                // undefined behaviour -> failsafe
                failsafe = true;
            break;
        }
    }
    else
    {
        // not armed -> failsafe
        failsafe = true;
    }

    // if behaviour defined, execute controller and mix; otherwise: set servos to failsafe
    if(!failsafe)
    {
        // update controller cascade down to the motors
        flight_controller_.update();
    }
    else
    {
        flight_controller_.failsafe();
    }

    return true;
}
