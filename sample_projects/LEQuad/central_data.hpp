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
 * \file central_data.h
 *
 * \author MAV'RIC Team
 *
 * \brief Place where the central data is stored and initialized
 *
 ******************************************************************************/


#ifndef CENTRAL_DATA_H_
#define CENTRAL_DATA_H_

#include <stdbool.h>
#include <stdint.h>


#include "sensing/imu.hpp"
#include "drivers/gps.hpp"
#include "drivers/sonar.hpp"
#include "hal/common/file.hpp"

#include "control/stabilisation_copter.hpp"
#include "communication/mavlink_communication.hpp"
#include "communication/onboard_parameters.hpp"
#include "communication/mavlink_waypoint_handler.hpp"
#include "control/navigation.hpp"
#include "communication/hud_telemetry.hpp"
#include "communication/state_machine.hpp"
#include "communication/data_logging.hpp"
#include "sensing/qfilter.hpp"
#include "communication/mavlink_stream.hpp"
#include "simulation/simulation.hpp"
#include "sensing/position_estimation.hpp"
#include "communication/state.hpp"
#include "control/manual_control.hpp"
#include "drivers/battery.hpp"
#include "control/servos_mix_quadcopter_diag.hpp"
#include "hal/common/led.hpp"
#include "drivers/servos_telemetry.hpp"
#include "control/altitude_controller.hpp"

#include "sensing/altitude_estimation.hpp"
#include "control/stabilisation_copter_default_config.hpp"
#include "communication/mavlink_communication_default_config.hpp"
#include "sensing/position_estimation_default_config.hpp"
#include "control/servos_mix_quadcopter_diag_default_config.hpp"
#include "control/manual_control_default_config.hpp"
#include "communication/remote_default_config.hpp"
#include "control/attitude_controller_default_config.hpp"
#include "control/velocity_controller_copter_default_config.hpp"
#include "control/navigation_default_config.hpp"
#include "sensing/qfilter_default_config.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "sensing/ahrs.h"
#include "sensing/altitude.h"
#include "control/pid_controller.h"
#include "util/print_util.h"
#include "util/coord_conventions.h"
#include "control/stabilisation.h"
#include "control/attitude_controller.h"
#include "runtime/scheduler_default_config.h"
}

#include "control/velocity_controller_copter.hpp"
#include "control/vector_field_waypoint.hpp"


typedef struct
{
    state_conf_t state_config;
    data_logging_conf_t data_logging_config;
    data_logging_conf_t data_logging_config2;
    scheduler_conf_t scheduler_config;
    mavlink_communication_conf_t mavlink_communication_config;
    qfilter_conf_t qfilter_config;
    position_estimation_conf_t position_estimation_config;
    navigation_conf_t navigation_config;
    stabilisation_copter_conf_t stabilisation_copter_config;
    servos_mix_quadcopter_diag_conf_t servos_mix_quadcopter_diag_config;
    manual_control_conf_t manual_control_config;
    remote_conf_t remote_config;
    attitude_controller_conf_t attitude_controller_config;
    velocity_controller_copter_conf_t velocity_controller_copter_config;

}central_data_conf_t;

static inline central_data_conf_t central_data_default_config();

/**
 * \brief The central data structure
 */
class Central_data
{
public:
    /**
     * \brief   Constructor
     */
    Central_data(uint8_t sysid, central_data_conf_t config, Imu& imu, Barometer& barometer, Gps& gps, Sonar& sonar, Serial& serial_mavlink, Satellite& satellite, Led& led, File& file_flash, Battery& battery, Servo& servo_0, Servo& servo_1, Servo& servo_2, Servo& servo_3, File& file1, File& file2);


    /**
     * \brief   Initialisation
     * \return [description]
     */
    bool init(central_data_conf_t config);


    /**
     * Public members
     */
    Imu&            imu;                ///< Reference to IMU
    Barometer&      barometer;          ///< Reference to barometer
    Gps&            gps;                ///< Reference to GPS
    Sonar&          sonar;              ///< Reference to sonar
    Serial&         serial_mavlink;     ///< Reference to telemetry serial
    Satellite&      satellite;          ///< Reference to remote control satellite
    Led&            led;                ///< Reference to the leds
    File&           file_flash;         ///< Reference to flash storage
    Battery&        battery;            ///< Reference to battery
    Servo&          servo_0;            ///< Reference to servos structure
    Servo&          servo_1;            ///< Reference to servos structure
    Servo&          servo_2;            ///< Reference to servos structure
    Servo&          servo_3;            ///< Reference to servos structure

    scheduler_t scheduler;
    mavlink_communication_t mavlink_communication;

    servos_mix_quadcotper_diag_t servo_mix;


    qfilter_t attitude_filter;                                  ///< The qfilter structure
    ahrs_t ahrs;                                                ///< The attitude estimation structure

    control_command_t controls;                                 ///< The control structure used for rate and attitude modes
    control_command_t controls_nav;                             ///< The control nav structure used for velocity modes

    manual_control_t manual_control;                            ///< The joystick parsing structure

    stabilisation_copter_t stabilisation_copter;                ///< The stabilisation structure for copter

    position_estimation_t position_estimation;                  ///< The position estimaton structure
    mavlink_waypoint_handler_t waypoint_handler;
    navigation_t navigation;                                    ///< The structure to perform GPS navigation

    State state;                                                ///< The structure with all state information
    state_machine_t state_machine;                              ///< The structure for the state machine

    hud_telemetry_structure_t hud_structure;                    ///< The HUD structure
    servos_telemetry_t servos_telemetry;

    Data_logging    data_logging;
    Data_logging    data_logging2;

    command_t                       command;
    attitude_controller_t           attitude_controller;
    velocity_controller_copter_t    velocity_controller;
    vector_field_waypoint_t         vector_field_waypoint;

    altitude_t                      altitude_;
    Altitude_estimation             altitude_estimation_;
    Altitude_controller             altitude_controller_;

private:
    uint8_t sysid_;     ///< System ID
};

static inline central_data_conf_t central_data_default_config()
{
    central_data_conf_t conf = {};

    conf.state_config = state_default_config();

    conf.data_logging_config = data_logging_default_config();
    conf.data_logging_config = data_logging_default_config();

    conf.scheduler_config = scheduler_default_config();

    conf.mavlink_communication_config = mavlink_communication_default_config();

    conf.qfilter_config = qfilter_default_config();

    conf.position_estimation_config = position_estimation_default_config();

    conf.navigation_config = navigation_default_config();

    conf.stabilisation_copter_config = stabilisation_copter_default_config();

    conf.servos_mix_quadcopter_diag_config = servos_mix_quadcopter_diag_default_config();

    conf.manual_control_config = manual_control_default_config();

    conf.remote_config = remote_default_config();

    conf.attitude_controller_config = attitude_controller_default_config();

    conf.velocity_controller_copter_config = velocity_controller_copter_default_config();

    return conf;
}

#endif /* CENTRAL_DATA_H_ */
