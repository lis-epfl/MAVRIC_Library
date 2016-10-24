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
 * \file lequad.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/


#ifndef LEQUAD_HPP_
#define LEQUAD_HPP_

#include <cstdbool>
#include <cstdint>


#include "communication/data_logging.hpp"
#include "communication/hud_telemetry.hpp"

#include "communication/mavlink_communication.hpp"
#include "communication/mavlink_stream.hpp"
#include "communication/mavlink_waypoint_handler.hpp"
#include "communication/onboard_parameters.hpp"
#include "manual_control/remote_default_config.hpp"

#include "control/altitude_controller.hpp"
#include "control/position_controller.hpp"
#include "control/velocity_controller_copter.hpp"
#include "control/attitude_controller.hpp"
#include "manual_control/manual_control.hpp"
#include "navigation/navigation_directto.hpp"
#include "control/rate_controller.hpp"
#include "control/servos_mix_quadcopter_diag.hpp"
#include "manual_control/manual_control.hpp"
#include "control/stabilisation.hpp"

#include "navigation/vector_field_waypoint.hpp"

#include "drivers/battery.hpp"
#include "drivers/gps.hpp"
#include "drivers/gps_mocap.hpp"
#include "drivers/gps_hub.hpp"
#include "drivers/sonar.hpp"
#include "drivers/servos_telemetry.hpp"
#include "drivers/state_display.hpp"

#include "hal/common/file.hpp"
#include "hal/common/led.hpp"

#include "mission/mission_planner.hpp"
#include "mission/mission_handler_critical_landing.hpp"
#include "mission/mission_handler_critical_navigating.hpp"
#include "mission/mission_handler_hold_position.hpp"
#include "mission/mission_handler_landing.hpp"
#include "mission/mission_handler_manual.hpp"
#include "mission/mission_handler_navigating.hpp"
#include "mission/mission_handler_on_ground.hpp"
#include "mission/mission_handler_takeoff.hpp"

#include "simulation/simulation.hpp"

#include "sensing/ahrs.hpp"
#include "sensing/ahrs_ekf.hpp"
#include "sensing/altitude_estimation.hpp"
#include "sensing/imu.hpp"
#include "sensing/ins_complementary.hpp"
#include "sensing/ins_kf.hpp"
#include "sensing/qfilter.hpp"
#include "sensing/qfilter_default_config.hpp"
#include "sensing/ahrs_ekf_mocap.hpp"

#include "status/geofence.hpp"
#include "status/geofence_cylinder.hpp"
#include "status/state.hpp"
#include "status/state_machine.hpp"

#include "util/coord_conventions.hpp"
#include "util/print_util.hpp"

extern "C"
{
#include "sensing/altitude.h"
}

typedef Navigation_directto<Position_controller<Velocity_controller_copter<Attitude_controller<Rate_controller<Servos_mix_quadcopter_diag> > > > > Cascade_controller;

/**
 * \brief MAV class
 */
class LEQuad
{
public:
    static const uint32_t N_TELEM  = 30;
    static const uint32_t N_MSG_CB = 20;
    static const uint32_t N_CMD_CB = 20;
    static const uint32_t N_PARAM  = 120;
    typedef Mavlink_communication_T<N_TELEM, N_MSG_CB, N_CMD_CB, N_PARAM> Mavlink_communication;


    /**
     * \brief   Configuration structure
     */
     struct conf_t
    {
        State::conf_t state_config;
        Data_logging::conf_t data_logging_continuous_config;
        Data_logging::conf_t data_logging_stat_config;
        Scheduler::conf_t scheduler_config;
        Mavlink_communication::conf_t mavlink_communication_config;
        Mavlink_waypoint_handler::conf_t waypoint_handler_config;
        Mission_planner::conf_t mission_planner_config;
        Mission_handler_landing<Navigation_controller_I, XYposition_Zvel_controller_I>::conf_t mission_handler_landing_config;
        qfilter_conf_t qfilter_config;
        Ahrs_ekf::conf_t ahrs_ekf_config;
        INS_complementary::conf_t ins_complementary_config;
        Manual_control::conf_t manual_control_config;
        remote_conf_t remote_config;
        Cascade_controller::conf_t cascade_controller_config;
        Geofence_cylinder::conf_t safety_geofence_config;
        Geofence_cylinder::conf_t emergency_geofence_config;
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
     * \brief   Configuration for use in drone dome
     *
     * \param   sysid       System id (default value = 1)
     *
     * \return  Config structure
     */
    static inline conf_t dronedome_config(uint8_t sysid = 1);


    /**
     * \brief   Constructor
     */
    LEQuad( Imu& imu,
            Barometer& barometer,
            Gps& gps,
            Sonar& sonar,
            Px4flow_i2c& flow,
            Serial& serial_mavlink,
            Satellite& satellite,
            State_display& state_display,
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
            const conf_t& config = default_config());

    /*
     * \brief   Initializes LEQuad
     * \details  Calls all init functions (init_*());
     *
     * \return  success
     */
    virtual bool init(void);

    /**
     *  \brief    Main update function (infinite loop)
     *  \details  Performs last operations before flight, then loops on scheduler updates
     */
    void loop(void);


    /**
     * \brief   Returns non-const reference to MAVLINK Communication
     * \details This is used to add simulation telemetry from the main function
     *
     * \return  MAVLINK Communication module
     */
     inline Mavlink_communication& mavlink_communication(){return communication;};


protected:

    virtual bool init_main_task(void);
    virtual bool init_state(void);
    virtual bool init_communication(void);
    virtual bool init_data_logging(void);
    virtual bool init_gps(void);
    virtual bool init_imu(void);
    virtual bool init_barometer(void);
    virtual bool init_sonar(void);
    virtual bool init_ahrs(void);
    virtual bool init_ins(void);
    virtual bool init_mocap(void);
    virtual bool init_flow(void);
    virtual bool init_stabilisers(void);
    virtual bool init_mission_planning(void);
    virtual bool init_hud(void);
    virtual bool init_servos(void);
    virtual bool init_ground_control(void);

    virtual bool main_task(void);
    static inline bool main_task_func(LEQuad* mav)
    {
        return mav->main_task();
    };

    Imu&            imu;                ///< Reference to IMU
    Barometer&      barometer;          ///< Reference to barometer
    Gps&            gps;                ///< Reference to GPS
    Sonar&          sonar;              ///< Reference to sonar
    Px4flow_i2c&    flow;              ///< Optic flow sensor
    Serial&         serial_mavlink;     ///< Reference to telemetry serial
    Satellite&      satellite;          ///< Reference to remote control satellite
    State_display&  state_display_;     ///< Reference to the state display
    File&           file_flash;         ///< Reference to flash storage
    Battery&        battery;            ///< Reference to battery
    Servo&          servo_0;            ///< Reference to servos structure
    Servo&          servo_1;            ///< Reference to servos structure
    Servo&          servo_2;            ///< Reference to servos structure
    Servo&          servo_3;            ///< Reference to servos structure
    Servo&          servo_4;            ///< Reference to servos structure
    Servo&          servo_5;            ///< Reference to servos structure
    Servo&          servo_6;            ///< Reference to servos structure
    Servo&          servo_7;            ///< Reference to servos structure


    // Motion capture, position
    Gps_mocap       gps_mocap;          ///< Position measure using mocap information
    Gps_hub<2>      gps_hub;            ///< Gps hub

    // Motion capture, orientation
    Ahrs_ekf_mocap  ahrs_ekf_mocap;     ///< Attitude measure from mocap information

    Manual_control manual_control;                              ///< The joystick parsing structure

    State state;                                                ///< The structure with all state information

    Scheduler_T<20>       scheduler;
    Mavlink_communication   communication;

    ahrs_t ahrs;                                                ///< The attitude estimation structure
    Ahrs_ekf ahrs_ekf;

    INS*                ins_;                                   ///< Alias for the position filter in use
    INS_complementary   ins_complementary;                      ///< The position estimaton structure
    INS_kf              ins_kf;                                 ///< The Kalman INS structure, used for position estimation

    control_command_t controls;                                 ///< The control structure used for rate and attitude modes

    Cascade_controller cascade_controller_;

    Mission_handler_registry mission_handler_registry;          ///< The class for registring and obtaining mission handlers
    Mavlink_waypoint_handler waypoint_handler;                  ///< The handler for the waypoints
    Mission_handler_hold_position<Navigation_controller_I> hold_position_handler;
    Mission_handler_landing<Navigation_controller_I, XYposition_Zvel_controller_I> landing_handler;
    Mission_handler_navigating<Navigation_controller_I> navigating_handler;
    Mission_handler_on_ground on_ground_handler;
    Mission_handler_manual manual_ctrl_handler;
    Mission_handler_takeoff<Navigation_controller_I> takeoff_handler;
    Mission_handler_critical_landing<Navigation_controller_I, XYposition_Zvel_controller_I> critical_landing_handler;
    Mission_handler_critical_navigating<Navigation_controller_I> critical_navigating_handler;
    Mission_planner mission_planner;                            ///< Controls the mission plan

    Geofence_cylinder safety_geofence_;                         ///< Geofence
    Geofence_cylinder emergency_geofence_;                      ///< Geofence

    State_machine state_machine;                                ///< The structure for the state machine

    hud_telemetry_t hud;                                        ///< The HUD structure
    servos_telemetry_t servos_telemetry;

    Data_logging_T<10>    data_logging_continuous;
    Data_logging_T<10>    data_logging_stat;

    command_t                       command;
    // attitude_controller_t           attitude_controller;
    // velocity_controller_copter_t    velocity_controller;
    // vector_field_waypoint_t         vector_field_waypoint;

    uint8_t sysid_;    ///< System ID
    conf_t config_;    ///< Configuration
};


LEQuad::conf_t LEQuad::default_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf.state_config = State::default_config();

    conf.data_logging_continuous_config                  = Data_logging::default_config();
    conf.data_logging_continuous_config.continuous_write = true;
    conf.data_logging_continuous_config.log_data         = 0;

    conf.data_logging_stat_config                  = Data_logging::default_config();
    conf.data_logging_stat_config.continuous_write = false;
    conf.data_logging_stat_config.log_data         = 0;

    conf.scheduler_config = Scheduler::default_config();

    conf.waypoint_handler_config = Mavlink_waypoint_handler::default_config();

    conf.mission_planner_config = Mission_planner::default_config();
    conf.mission_handler_landing_config = Mission_handler_landing<Navigation_controller_I, XYposition_Zvel_controller_I>::default_config();

    conf.qfilter_config = qfilter_default_config();

    conf.ahrs_ekf_config = Ahrs_ekf::default_config();

    conf.ins_complementary_config = INS_complementary::default_config();

    conf.manual_control_config = Manual_control::default_config();

    conf.remote_config = remote_default_config();

    conf.cascade_controller_config = Cascade_controller::default_config();

    conf.safety_geofence_config     = Geofence_cylinder::default_config();
    conf.emergency_geofence_config  = Geofence_cylinder::default_config();
    conf.emergency_geofence_config.radius = 1000.0f;
    conf.emergency_geofence_config.height = 500.0f;

    /* Mavlink communication config */
    conf.mavlink_communication_config                        = Mavlink_communication::default_config(sysid);
    conf.mavlink_communication_config.handler.debug          = false;
    conf.mavlink_communication_config.parameters.debug       = true;
    conf.mavlink_communication_config.mavlink_stream.debug   = false;

    return conf;
};


LEQuad::conf_t LEQuad::dronedome_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf = LEQuad::default_config(sysid);

    // adapt gains for the drone dome
    conf.ins_complementary_config.kp_baro_alt = 0.0f;
    conf.ins_complementary_config.kp_baro_vel = 0.0f;
    conf.ins_complementary_config.kp_sonar_alt = 0.0f;
    conf.ins_complementary_config.kp_sonar_vel = 0.0f;

    conf.mission_handler_landing_config.desc_to_ground_altitude = -1.0f;

    conf.mission_planner_config.safe_altitude                   =  -3.0f;
    conf.mission_planner_config.critical_landing_altitude       =  -2.0f;
    conf.mission_planner_config.takeoff_altitude                =  -2.0f;

    return conf;
}

#endif /* LEQUAD_HPP_ */
