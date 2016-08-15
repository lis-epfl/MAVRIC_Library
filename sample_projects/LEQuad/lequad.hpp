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
#include "communication/state.hpp"
#include "communication/state_machine.hpp"
#include "communication/remote_default_config.hpp"

#include "control/altitude_controller.hpp"
#include "control/attitude_controller.hpp"
#include "control/manual_control.hpp"
#include "control/navigation.hpp"
#include "control/servos_mix_quadcopter_diag.hpp"
#include "control/servos_mix_quadcopter_diag_default_config.hpp"
#include "control/stabilisation.hpp"
#include "control/stabilisation_copter.hpp"
#include "control/stabilisation_copter_default_config.hpp"
#include "control/velocity_controller_copter.hpp"
#include "control/vector_field_waypoint.hpp"

#include "drivers/battery.hpp"
#include "drivers/gps.hpp"
#include "drivers/sonar.hpp"
#include "drivers/servos_telemetry.hpp"
#include "drivers/state_display.hpp"

#include "hal/common/file.hpp"
#include "hal/common/led.hpp"

#include "simulation/simulation.hpp"

#include "sensing/ahrs.hpp"
#include "sensing/ahrs_ekf.hpp"
#include "sensing/altitude_estimation.hpp"
#include "sensing/imu.hpp"
#include "sensing/position_estimation.hpp"
#include "sensing/qfilter.hpp"
#include "sensing/qfilter_default_config.hpp"

#include "util/coord_conventions.hpp"

extern "C"
{
#include "sensing/altitude.h"
#include "util/print_util.hpp"
}



/**
 * \brief MAV class
 */
class LEQuad
{
public:
    /**
     * \brief   Configuration structure
     */
     struct conf_t
    {
        State::conf_t state_config;
        data_logging_conf_t data_logging_continuous_config;
        data_logging_conf_t data_logging_stat_config;
        Scheduler::conf_t scheduler_config;
        Mavlink_communication::conf_t mavlink_communication_config;
        Navigation::conf_t navigation_config;
        Mavlink_waypoint_handler::conf_t waypoint_handler_config;
        qfilter_conf_t qfilter_config;
        Ahrs_ekf::conf_t ahrs_ekf_config;
        Position_estimation::conf_t position_estimation_config;
        stabilisation_copter_conf_t stabilisation_copter_config;
        servos_mix_quadcopter_diag_conf_t servos_mix_quadcopter_diag_config;
        Manual_control::conf_t manual_control_config;
        remote_conf_t remote_config;
        Attitude_controller::conf_t attitude_controller_config;
        Velocity_controller_copter::conf_t velocity_controller_copter_config;
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
     * \brief   Constructor
     */
    LEQuad( Imu& imu,
                  Barometer& barometer,
                  Gps& gps,
                  Sonar& sonar,
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

    /**
     *  \brief    Main update function (infinite loop)
     *  \details  Performs last operations before flight, then loops on scheduler updates
     */
    void loop(void);

// protected:

    virtual bool init_main_task(void);
    virtual bool init_state(void);
    virtual bool init_communication(void);
    virtual bool init_data_logging(void);
    virtual bool init_gps(void);
    virtual bool init_imu(void);
    virtual bool init_barometer(void);
    virtual bool init_sonar(void);
    virtual bool init_attitude_estimation(void);
    virtual bool init_position_estimation(void);
    virtual bool init_stabilisers(void);
    virtual bool init_navigation(void);
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

    Manual_control manual_control;                              ///< The joystick parsing structure

    State state;                                                ///< The structure with all state information

    Scheduler_tpl<20> scheduler;
    Mavlink_communication mavlink_communication;

    servos_mix_quadcotper_diag_t servo_mix;

    ahrs_t ahrs;                                                ///< The attitude estimation structure
    Ahrs_ekf ahrs_ekf;

    Position_estimation position_estimation;                    ///< The position estimaton structure

    control_command_t controls;                                 ///< The control structure used for rate and attitude modes
    control_command_t controls_nav;                             ///< The control nav structure used for velocity modes

    stabilisation_copter_t stabilisation_copter;                ///< The stabilisation structure for copter

    Navigation navigation;                                      ///< The structure to perform GPS navigation
    Mavlink_waypoint_handler waypoint_handler;

    State_machine state_machine;                                ///< The structure for the state machine

    hud_telemetry_t hud;                                        ///< The HUD structure
    servos_telemetry_t servos_telemetry;

    Data_logging    data_logging_continuous;
    Data_logging    data_logging_stat;

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

    conf.data_logging_continuous_config                  = data_logging_default_config();
    conf.data_logging_continuous_config.continuous_write = true;
    conf.data_logging_stat_config       = data_logging_default_config();

    conf.scheduler_config = Scheduler::default_config();

    conf.navigation_config = Navigation::default_config();

    conf.waypoint_handler_config = Mavlink_waypoint_handler::default_config();

    conf.qfilter_config = qfilter_default_config();

    conf.ahrs_ekf_config = Ahrs_ekf::default_config();

    conf.position_estimation_config = Position_estimation::default_config();

    conf.stabilisation_copter_config = stabilisation_copter_default_config();

    conf.servos_mix_quadcopter_diag_config = servos_mix_quadcopter_diag_default_config();

    conf.manual_control_config = Manual_control::default_config();

    conf.remote_config = remote_default_config();

    conf.attitude_controller_config = Attitude_controller::default_config();

    conf.velocity_controller_copter_config = Velocity_controller_copter::default_config();

    /* Mavlink communication config */
    Mavlink_communication::conf_t mavlink_communication_config   = Mavlink_communication::default_config(sysid);
    mavlink_communication_config.message_handler_config.debug    = false;
    mavlink_communication_config.onboard_parameters_config.debug = true;
    mavlink_communication_config.mavlink_stream_config.debug     = false;
    conf.mavlink_communication_config = mavlink_communication_config;

    return conf;
};

#endif /* LEQUAD_HPP_ */
