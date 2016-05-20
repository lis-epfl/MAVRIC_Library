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
 * \file state.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief Holds the current status of the MAV
 *
 ******************************************************************************/


#ifndef STATE_H_
#define STATE_H_

#include "stdint.h"
#include <stdbool.h>

#include "communication/mav_modes.hpp"
#include "communication/mavlink_stream.hpp"
#include "drivers/battery.hpp"
#include "mavlink_message_handler.hpp"


/**
 * \brief The MAV state
 */
class State
{
    friend class State_machine;
public:

    /**
     * \brief    Configuration structure
     *
     * TODO: clean
     */
    struct conf_t
    {
        mav_mode_t mav_mode;                                ///< The value of the MAV mode
        mav_state_t mav_state;                              ///< The value of the MAV state

        mav_mode_custom_t mav_mode_custom;                  ///< The value of the custom_mode

        int32_t simulation_mode;                            ///< The value of the simulation_mode (0: real, 1: simulation)
        uint8_t autopilot_type;                             ///< The type of the autopilot (MAV_TYPE enum in common.h)
        uint8_t autopilot_name;                             ///< The name of the autopilot (MAV_AUTOPILOT enum in common.h)

        uint32_t sensor_present;                            ///< The type of sensors that are present on the autopilot (Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control)
        uint32_t sensor_enabled;                            ///< The sensors enabled on the autopilot (Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control)
        uint32_t sensor_health;                             ///< The health of sensors present on the autopilot (Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control)

        float fence_1_xy;                                   ///< Size of fence 1 in the XY plane, in meters
        float fence_1_z;                                    ///< Size of fence 1 in the Z direction, in meters
        float fence_2_xy;                                   ///< Size of fence 2 in the XY plane, in meters
        float fence_2_z;                                    ///< Size of fence 2 in the Z direction, in meters
        bool out_of_fence_1;                                ///< Flag to tell whether we are out the first fence or not
        bool out_of_fence_2;                                ///< Flag to tell whether we are out the second fence or not

        bool nav_plan_active;                               ///< Flag to tell that a flight plan (min 1 waypoint) is active
        bool reset_position;                                ///< Flag to enable the reset of the position estimation

        double last_heartbeat_msg;                          ///< Time of reception of the last heartbeat message from the ground station
        double max_lost_connection;                         ///< Maximum time without reception of a heartbeat message from the ground station

        uint32_t msg_count;                                 ///< Number of heartbeat message received from the Ground station

        bool connection_lost;                               ///< Flag to tell if we have connection with the GND station or not
        bool first_connection_set;                          ///< Flag to tell that we received a first message from the GND station
    };


    /**
     * \brief   Constructor
     *
     * \param   mavlink_stream  Mavlink downlink
     * \param   battery         Battery monitor
     * \param   state_config    State configuration structure
     */
    State(Mavlink_stream& mavlink_stream_, Battery& battery, conf_t config = default_config());


    /**
     * \brief                   Makes the switch to active mode
     *
     * \param   mav_state_       MAV state
     */
    void switch_to_active_mode(mav_state_t* mav_state_);


    /**
     * \brief                   Check the connection status with the GND station
     */
    void connection_status();


    /**
     * \brief   tries to arm/disarm; checks performed for arming
     *
     * \details arming is prevented if
     *          - IMU not ready (mav_state_ == MAV_STATE_CALIBRATING)
     *          - in manual and guided or stabilised mode
     *          - battery low
     *
     * \param   arming      true for arming, false for disarming
     *
     * \return true if desired arming state was accepted; false if refused
     */
    bool set_armed(bool arming);


    /**
     * \brief                   returns whether armed
     *
     * \return                  armed
     */
    inline bool is_armed() const {return ((mav_mode_ & MAV_MODE_FLAG_SAFETY_ARMED) == MAV_MODE_FLAG_SAFETY_ARMED);};


    /**
    * \brief                   returns whether in manual mode
    *
    * \return                  manual
    */
    inline bool is_manual() const {return ((mav_mode_ & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) == MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);};


    /**
    * \brief                   returns whether in hil mode
    *
    * \return                  hil
    */
    inline bool is_hil() const {return ((mav_mode_ & MAV_MODE_FLAG_HIL_ENABLED) == MAV_MODE_FLAG_HIL_ENABLED);};


    /**
    * \brief                   returns whether in stabilize mode
    *
    * \return                  stabilize
    */
    inline bool is_stabilize() const {return ((mav_mode_ & MAV_MODE_FLAG_STABILIZE_ENABLED) == MAV_MODE_FLAG_STABILIZE_ENABLED);};


    /**
     * \brief                   returns whether in guided mode
     *
     * \return                  guided
     */
    inline bool is_guided() const {return ((mav_mode_ & MAV_MODE_FLAG_GUIDED_ENABLED) == MAV_MODE_FLAG_GUIDED_ENABLED);};


    /**
     * \brief                   returns whether in auto mode
     *
     * \return                  auto
     */
    inline bool is_auto() const {return ((mav_mode_ & MAV_MODE_FLAG_AUTO_ENABLED) == MAV_MODE_FLAG_AUTO_ENABLED);};

    /**
     * \brief                   returns whether in custom mode
     *
     * \return                  custom
     */
    inline bool is_custom() const {return ((mav_mode_ & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);};


    /**
     * \brief                   returns whether in test mode
     *
     * \return                  test
     */
    inline bool is_test() const {return ((mav_mode_ & MAV_MODE_FLAG_TEST_ENABLED) == MAV_MODE_FLAG_TEST_ENABLED);};


    /**
     * \brief                   returns mav_mode (copy)
     *
     * \return                  mav_mode
     */
    inline mav_mode_t mav_mode() const {return mav_mode_;};


    /**
     * \brief   Default configuration for quadrotor
     *
     * \return  Config structure
     */
    static inline conf_t default_config();

    /**
     * \brief   Default configuration for wing
     *
     * \return  Config structure
     */
    static inline conf_t wing_default_config();

    friend bool state_telemetry_set_mode(State* state, mav_mode_t mav_mode);
    friend mav_result_t state_telemetry_send_autopilot_capabilities(State* state, mavlink_command_long_t* packet);

// TODO:
// All this should be private

    mav_state_t mav_state_;                              ///< The value of the MAV state
    mav_mode_t mav_mode_;                               ///< The value of the MAV mode
    mav_mode_custom_t mav_mode_custom;                  ///< The value of the custom_mode

    uint8_t autopilot_type;                             ///< The type of the autopilot (MAV_TYPE enum in common.h)
    uint8_t autopilot_name;                             ///< The name of the autopilot (MAV_AUTOPILOT enum in common.h)

    uint32_t sensor_present;                            ///< The type of sensors that are present on the autopilot (Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control)
    uint32_t sensor_enabled;                            ///< The sensors enabled on the autopilot (Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control)
    uint32_t sensor_health;                             ///< The health of sensors present on the autopilot (Value of 0: not enabled. Value of 1: enabled. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure, 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position, 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization, 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control)

    float fence_1_xy;                                   ///< Size of fence 1 in the XY plane, in meters
    float fence_1_z;                                    ///< Size of fence 1 in the Z direction, in meters
    float fence_2_xy;                                   ///< Size of fence 2 in the XY plane, in meters
    float fence_2_z;                                    ///< Size of fence 2 in the Z direction, in meters
    bool out_of_fence_1;                                ///< Flag to tell whether we are out the first fence or not
    bool out_of_fence_2;                                ///< Flag to tell whether we are out the second fence or not

    bool nav_plan_active;                               ///< Flag to tell that a flight plan (min 1 waypoint) is active
    bool in_the_air;                                    ///< Flag to tell whether the vehicle is airborne or not
    bool reset_position;                                ///< Flag to enable the reset of the position estimation

    double last_heartbeat_msg;                          ///< Time of reception of the last heartbeat message from the ground station
    double max_lost_connection;                         ///< Maximum time without reception of a heartbeat message from the ground station

    uint32_t msg_count;                                 ///< Number of heartbeat message received from the Ground station

    bool connection_lost;                               ///< Flag to tell if we have connection with the GND station or not
    bool first_connection_set;                          ///< Flag to tell that we received a first message from the GND station

    Battery& battery_;                                  ///< Pointer to battery structure

private:
    Mavlink_stream&   mavlink_stream_;                  ///< Mavlink communication, used to inform ground station of state and capabilities of drone
};


State::conf_t State::default_config()
{
    conf_t conf                  = {};

    conf.mav_mode                = MAV_MODE_SAFE;
    conf.mav_state               = MAV_STATE_BOOT;
    conf.simulation_mode         = HIL_OFF;
    conf.autopilot_type          = MAV_TYPE_QUADROTOR;
    conf.autopilot_name          = MAV_AUTOPILOT_GENERIC_MISSION_FULL; // TODO: add MAV_AUTOPILOT_MAVRIC to std mavlink and support waypoint commands in QGC for this autopilot type
    conf.sensor_present          = 0b1111110000100111;
    conf.sensor_enabled          = 0b1111110000100111;
    conf.sensor_health           = 0b1111110000100111;
    conf.max_lost_connection     = 60.0f;
    conf.fence_1_xy              = 100.0f;
    conf.fence_1_z               = 75.0f;
    conf.fence_2_xy              = 125.0f;
    conf.fence_2_z               = 100.0f;

    return conf;
}


State::conf_t State::wing_default_config()
{
    conf_t conf            = {};

    conf.mav_mode                = MAV_MODE_SAFE;
    conf.mav_state               = MAV_STATE_BOOT;
    conf.simulation_mode         = HIL_OFF;
    conf.autopilot_type          = MAV_TYPE_FIXED_WING;
    conf.autopilot_name          = MAV_AUTOPILOT_GENERIC_MISSION_FULL; // TODO: add MAV_AUTOPILOT_MAVRIC to std mavlink and support waypoint commands in QGC for this autopilot type
    conf.sensor_present          = 0b1111110000100111;
    conf.sensor_enabled          = 0b1111110000100111;
    conf.sensor_health           = 0b1111110000100111;
    conf.max_lost_connection     = 60.0f;
    conf.fence_1_xy              = 500.0f;
    conf.fence_1_z               = 150.0f;
    conf.fence_2_xy              = 600.0f;
    conf.fence_2_z               = 200.0f;

    return conf;
}

#endif //STATE_H_
