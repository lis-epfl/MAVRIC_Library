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
 * \file mav.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/


#ifndef MAV_HPP_
#define MAV_HPP_

#include <cstdbool>
#include <cstdint>


#include "communication/data_logging.hpp"
#include "communication/hud_telemetry.hpp"
#include "communication/mavlink_communication.hpp"
#include "communication/mavlink_stream.hpp"
#include "communication/mavlink_waypoint_handler.hpp"
#include "communication/onboard_parameters.hpp"

#include "flight_controller/flight_controller_copter.hpp"
#include "control/position_controller.hpp"
#include "control/velocity_controller_copter.hpp"
#include "control/attitude_controller.hpp"
#include "control/rate_controller.hpp"
#include "control/servos_mix_matrix.hpp"

#include "drivers/battery.hpp"
#include "drivers/gps.hpp"
#include "drivers/gps_mocap.hpp"
#include "drivers/gps_hub.hpp"
#include "drivers/sonar.hpp"
#include "drivers/servos_telemetry.hpp"
#include "drivers/state_display.hpp"
#include "drivers/px4flow.hpp"

#include "hal/common/file.hpp"
#include "hal/common/led.hpp"

#include "manual_control/manual_control.hpp"
#include "manual_control/remote_default_config.hpp"

#include "mission/mission_planner.hpp"
#include "mission/mission_handler_critical_landing.hpp"
#include "mission/mission_handler_critical_navigating.hpp"
#include "mission/mission_handler_hold_position.hpp"
#include "mission/mission_handler_landing.hpp"
#include "mission/mission_handler_manual.hpp"
#include "mission/mission_handler_navigating.hpp"
#include "mission/mission_handler_on_ground.hpp"
#include "mission/mission_handler_takeoff.hpp"

#include "navigation/navigation_directto.hpp"
#include "navigation/vector_field_waypoint.hpp"

#include "sensing/ahrs.hpp"
#include "sensing/ahrs_ekf.hpp"
#include "sensing/ahrs_qfilter.hpp"
#include "sensing/altitude_estimation.hpp"
#include "sensing/imu.hpp"
#include "sensing/ins_complementary.hpp"
#include "sensing/ins_kf.hpp"
#include "sensing/ahrs_ekf_mocap.hpp"

#include "simulation/simulation.hpp"

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


/**
 * \brief MAV class
 */
class MAV
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
        State_machine::conf_t state_machine_config;
        bool do_data_logging;
        Data_logging::conf_t data_logging_continuous_config;
        Data_logging::conf_t data_logging_stat_config;
        Scheduler::conf_t scheduler_config;
        Mavlink_communication::conf_t mavlink_communication_config;
        Mavlink_waypoint_handler::conf_t waypoint_handler_config;
        Mission_planner::conf_t mission_planner_config;
        Mission_handler_landing::conf_t mission_handler_landing_config;
        AHRS_qfilter::conf_t qfilter_config;
        AHRS_ekf::conf_t ahrs_ekf_config;
        INS_complementary::conf_t ins_complementary_config;
        Manual_control::conf_t manual_control_config;
        remote_conf_t remote_config;
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
    MAV(    Imu& imu,
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
            const conf_t& config = default_config());

    /*
     * \brief   Initializes MAV
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
    inline Mavlink_communication& get_communication(){return communication;};


     /**
      * \brief   Returns non-const reference to scheduler
      * \details This is used to add sleep task from the main function
      *
      * \return  Scheduler module
      */
    inline Scheduler& get_scheduler(){return scheduler;};


    /**
     * \brief   Returns non-const reference to data_logging stat
     * \details This is used to optionnally run data logging in a separate thread
     *
     * \return  Data logging stat module
     */
   inline Data_logging& get_data_logging_stat(){return data_logging_stat;};


   /**
    * \brief   Returns non-const reference to data_logging continuous
    * \details This is used to optionnally run data logging in a separate thread
    *
    * \return  Data logging continuous module
    */
  inline Data_logging& get_data_logging_continuous(){return data_logging_continuous;};


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
    virtual bool init_controller(void) = 0;
    virtual bool init_mission_planning(void);
    virtual bool init_hud(void);
    // virtual bool init_servos(void);
    virtual bool init_ground_control(void);

    virtual bool main_task(void);
    static inline bool main_task_func(MAV* mav)
    {
        return mav->main_task();
    };

    Imu&            imu;                ///< Reference to IMU
    Barometer&      barometer;          ///< Reference to barometer
    Gps&            gps;                ///< Reference to GPS
    Sonar&          sonar;              ///< Reference to sonar
    PX4Flow&        flow;               ///< Optic flow sensor
    Serial&         serial_mavlink;     ///< Reference to telemetry serial
    Satellite&      satellite;          ///< Reference to remote control satellite
    State_display&  state_display_;     ///< Reference to the state display
    File&           file_flash;         ///< Reference to flash storage
    Battery&        battery;            ///< Reference to battery

    // Motion capture, position
    Gps_mocap       gps_mocap;          ///< Position measure using mocap information
    Gps_hub<2>      gps_hub;            ///< Gps hub
    Ahrs_ekf_mocap  ahrs_ekf_mocap;     ///< Attitude measure from mocap information

    Manual_control manual_control;                              ///< The joystick parsing structure

    State state;                                                ///< The structure with all state information

    Scheduler_T<20>       scheduler;
    Mavlink_communication   communication;

    AHRS&           ahrs_;              ///< The attitude estimation structure
    AHRS_ekf        ahrs_ekf;
    AHRS_qfilter    ahrs_qfilter;

    INS&                ins_;                                   ///< Alias for the position filter in use
    INS_complementary   ins_complementary;                      ///< The position estimaton structure
    INS_kf              ins_kf;                                 ///< The Kalman INS structure, used for position estimation

    Flight_controller&    flight_controller_;

    Mavlink_waypoint_handler waypoint_handler;                  ///< The handler for the waypoints

    Mission_handler_registry mission_handler_registry;          ///< The class for registring and obtaining mission handlers

    Mission_handler_hold_position       hold_position_handler;
    Mission_handler_landing             landing_handler;
    Mission_handler_navigating          navigating_handler;
    Mission_handler_on_ground           on_ground_handler;
    Mission_handler_manual              manual_ctrl_handler;
    Mission_handler_takeoff             takeoff_handler;
    Mission_handler_critical_landing    critical_landing_handler;
    Mission_handler_critical_navigating critical_navigating_handler;

    Mission_planner mission_planner_;                            ///< Controls the mission plan

    Geofence_cylinder safety_geofence_;                         ///< Geofence
    Geofence_cylinder emergency_geofence_;                      ///< Geofence

    State_machine state_machine;                                ///< The structure for the state machine

    hud_telemetry_t hud;                                        ///< The HUD structure
    servos_telemetry_t servos_telemetry;

    Data_logging_T<100> data_logging_continuous;
    Data_logging_T<10>  data_logging_stat;

public:
    uint8_t sysid_;    ///< System ID
    conf_t config_;    ///< Configuration
};


MAV::conf_t MAV::default_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf.state_config = State::default_config();

    conf.state_machine_config = State_machine::default_config();

    conf.do_data_logging = true;
    conf.data_logging_continuous_config                  = Data_logging::default_config();
    conf.data_logging_continuous_config.continuous_write = true;
    conf.data_logging_continuous_config.log_data         = 1;
    conf.data_logging_stat_config                  = Data_logging::default_config();
    conf.data_logging_stat_config.continuous_write = false;
    conf.data_logging_stat_config.log_data         = 1;

    conf.scheduler_config = Scheduler::default_config();

    conf.waypoint_handler_config = Mavlink_waypoint_handler::default_config();

    conf.mission_planner_config = Mission_planner::default_config();
    conf.mission_handler_landing_config = Mission_handler_landing::default_config();

    conf.ahrs_ekf_config = AHRS_ekf::default_config();
    conf.qfilter_config = AHRS_qfilter::default_config();

    conf.ins_complementary_config = INS_complementary::default_config();

    conf.manual_control_config = Manual_control::default_config();

    conf.remote_config = remote_default_config();

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


MAV::conf_t MAV::dronedome_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf = MAV::default_config(sysid);

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

#endif /* MAV_HPP_ */
