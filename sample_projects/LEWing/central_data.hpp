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

#include "control/stabilisation_wing.hpp"
#include "communication/mavlink_communication.hpp"
#include "communication/onboard_parameters.hpp"
#include "communication/mavlink_waypoint_handler.hpp"
#include "control/navigation.hpp"
#include "communication/hud_telemetry.hpp"
#include "communication/state_machine.hpp"
#include "communication/data_logging.hpp"
#include "sensing/ahrs_madgwick.hpp"
#include "communication/mavlink_stream.hpp"
#include "simulation/simulation.hpp"
#include "sensing/position_estimation.hpp"
#include "communication/state.hpp"
#include "control/manual_control.hpp"
#include "drivers/battery.hpp"
#include "control/servos_mix_wing.hpp"
#include "hal/common/led.hpp"
#include "drivers/servos_telemetry.hpp"
#include "drivers/airspeed_analog.hpp"

#include "control/stabilisation_wing_default_config.hpp"
#include "control/servos_mix_wing_default_config.hpp"
#include "communication/remote_default_config.hpp"
#include "sensing/ahrs_madgwick_default_config.hpp"

extern "C"
{
#include "sensing/ahrs.h"
#include "control/pid_controller.h"
#include "util/print_util.h"
#include "util/coord_conventions.h"
#include "control/stabilisation.h"
}

#include "control/vector_field_waypoint.hpp"

/**
 * \brief The central data structure
 */
class Central_data
{
public:
    struct conf_t
    {
        State::conf_t state_config;
        data_logging_conf_t data_logging_config;
        data_logging_conf_t data_logging_config2;
        Scheduler::conf_t scheduler_config;
        Mavlink_communication::conf_t mavlink_communication_config;
        Navigation::conf_t navigation_config;
        ahrs_madgwick_conf_t ahrs_madgwick_config;
        Position_estimation::conf_t position_estimation_config;
        stabilisation_wing_conf_t stabilisation_wing_config;
        servos_mix_wing_conf_t servos_mix_wing_config;
        Manual_control::conf_t manual_control_config;
        remote_conf_t remote_config;
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
    Central_data(   Imu& imu, 
                    Barometer& barometer, 
                    Gps& gps, 
                    Sonar& sonar, 
                    Serial& serial_mavlink, 
                    Satellite& satellite, 
                    Led& led, 
                    File& file_flash, 
                    Battery& battery, 
                    Servo& servo_0, 
                    Servo& servo_1, 
                    Servo& servo_2, 
                    Servo& servo_3, 
                    Airspeed_analog& airspeed_analog, 
                    File& file1, 
                    File& file2,
                    const conf_t& config = default_config());


    /**
     * \brief   Initialisation
     * \return [description]
     */
    bool init(void);


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

    Manual_control manual_control;                            ///< The joystick parsing structure

    State state;                                                ///< The structure with all state information

    Airspeed_analog& airspeed_analog;   ///< Reference to the analog airspeed

    Scheduler scheduler;
    Mavlink_communication mavlink_communication;

    command_t command;
    servos_mix_wing_t servo_mix;


    ahrs_madgwick_t attitude_filter;                            ///< The attitude filter structure
    ahrs_t ahrs;                                                ///< The attitude estimation structure

    control_command_t controls;                                 ///< The control structure used for rate and attitude modes
    control_command_t controls_nav;                             ///< The control nav structure used for velocity modes

    stabilisation_wing_t stabilisation_wing;                   ///< The stabilisation structure for a wing

    Position_estimation position_estimation;                  ///< The position estimaton structure
    Navigation navigation;                                    ///< The structure to perform GPS navigation
    Mavlink_waypoint_handler waypoint_handler;

    
    State_machine state_machine;                              ///< The structure for the state machine

    hud_telemetry_structure_t hud_structure;                    ///< The HUD structure
    servos_telemetry_t servos_telemetry;

    Data_logging    data_logging;
    Data_logging    data_logging2;

    vector_field_waypoint_t         vector_field_waypoint;

private:
    uint8_t sysid_;     ///< System ID

    conf_t config_;    ///< Configuration
};

Central_data::conf_t Central_data::default_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf.state_config = State::wing_default_config();

    conf.data_logging_config = data_logging_default_config();
    conf.data_logging_config = data_logging_default_config();

    conf.scheduler_config = Scheduler::default_config();

    conf.navigation_config = Navigation::default_config();
    conf.navigation_config.navigation_type = DUBIN;
    conf.navigation_config.takeoff_altitude = -40.0f;


    conf.ahrs_madgwick_config = ahrs_madgwick_default_config();

    conf.position_estimation_config = Position_estimation::default_config();

    conf.stabilisation_wing_config = stabilisation_wing_default_config();

    conf.servos_mix_wing_config = servos_mix_wing_default_config();

    conf.manual_control_config = Manual_control::default_config();

    conf.remote_config = remote_default_config();

    /* Mavlink communication config */
    Mavlink_communication::conf_t mavlink_communication_config = Mavlink_communication::default_config(sysid);
    mavlink_communication_config.message_handler_config.debug = true;
    mavlink_communication_config.onboard_parameters_config.debug = true;
    mavlink_communication_config.mavlink_stream_config.debug = true;
    conf.mavlink_communication_config = mavlink_communication_config;

    return conf;
}

#endif /* CENTRAL_DATA_H_ */
