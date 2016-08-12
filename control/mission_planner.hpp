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
 * \file mission_planner.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The mission planner
 *
 ******************************************************************************/


#ifndef MISSION_PLANNER__
#define MISSION_PLANNER__

#include "communication/mavlink_communication.hpp"
#include "communication/mavlink_stream.hpp"
#include "communication/mavlink_waypoint_handler.hpp"
#include "sensing/position_estimation.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "communication/state.hpp"
#include "sensing/qfilter.hpp"
#include "control/manual_control.hpp"
#include "control/mission_handler_registry.hpp"
#include "control/navigation.hpp"
#include "control/dubin.hpp"
#include "control/mission_handler.hpp"

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_planner
{
public:
    enum internal_state_t
    {
        STANDBY,
        PREMISSION,
        MISSION,
        POSTMISSION,
        PAUSED
    };

    struct conf_t
    {
    };


    /**
     * \brief   Initialize the waypoint handler
     *
     * \param   ins                         The reference to the ins structure
     * \param   navigation                  The reference to the navigation structure
     * \param   ahrs                        The reference to the attitude estimation structure
     * \param   state                       The reference to the state structure
     * \param   manual_control              The reference to the manual control structure
     * \param   mavlink_communication       The reference to the MAVLink communication structure
     * \param   mavlink_stream              The reference to the MAVLink stream structure
     * \param   waypoint_handler            The handler for the waypoints
     * \param   mission_handler_registry    The reference to the mission handler registry
     *
     * \return  True if the init succeed, false otherwise
     */
    Mission_planner(    INS& ins,
                        Navigation& navigation,
                        const ahrs_t& ahrs,
                        State& state,
                        const Manual_control& manual_control,
                        Mavlink_message_handler& message_handler,
                        const Mavlink_stream& mavlink_stream,
                        Mavlink_waypoint_handler& waypoint_handler,
                        Mission_handler_registry& mission_handler_registry,
                        conf_t config = default_config());

    bool init();

    /**
     * \brief   The mission planner tasks, gives a goal for the navigation module
     *
     * \param   mission_planner     The pointer to the waypoint handler structure
     */
    static bool update(Mission_planner* mission_planner);

    /**
     * \brief   Default configuration
     *
     * \return  Config structure
     */
    static inline conf_t default_config();

    void set_critical_next_state(bool critical_next_state);

    /**
     * \brief   Gets the internal state
     *
     * \return  internal_state_
     */
    internal_state_t internal_state() const;

    /**
     * \brief   Sets the internal state based on default waypoints
     *
     * \details     THIS SHOULD NOT BE SET AS A METHOD TO CHANGE THE
     *              MISSION HANDLER! USE Mission_planner::switch_mission_handler()
     *              or Mission_planner::insert_mission_waypoint()
     *
     * \param   new_internal_state  The new internal state
     */
    void set_internal_state(internal_state_t new_internal_state);

    /**
     * \brief   Switches the mission handler to the inputted waypoint
     *
     * \details     The current mission handler is set based on the
     *              inputted waypoint. If the mission handler could
     *              not be setup or found, no change is made to the
     *              current mission handler
     *
     * \param   waypoint
     *
     * \return  Success
     */
    bool switch_mission_handler(const Waypoint& waypoint);

    /**
     * \brief   Inserts the inputted waypoint into the mission
     * 
     * \details     This inserts a waypoint into the mission and sets
     *              the internal state to paused. If the insert fails,
     *              the function returns false and does not set the
     *              mission handler or inserted waypoint
     *
     * \param   wpt     The waypoint
     *
     * \return  Success
     */
    bool insert_mission_waypoint(Waypoint wpt);

protected:
    Mission_handler* current_mission_handler_;                  ///< The currently used mission handler
    Waypoint inserted_waypoint_;                                ///< A waypoint that is inserted into the plan outside of the normal mission

    internal_state_t internal_state_;                           ///< The internal state of the navigation module

    Mavlink_waypoint_handler& waypoint_handler_;                ///< The reference to the mavlink waypoint handler
    Mission_handler_registry& mission_handler_registry_;        ///< The reference to the mission handler registry

    bool hold_position_set_;                                    ///< Flag stating if the pilot has specifically switched to hold position

    bool critical_next_state_;                                  ///< Flag to change critical state in its dedicated state machine
    Waypoint critical_waypoint_;                                ///< Waypoint for the critical state

    const Mavlink_stream& mavlink_stream_;                      ///< The reference to MAVLink stream
    State& state_;                                              ///< The reference to the state structure
    Navigation& navigation_;                                    ///< The reference to the navigation structure
    INS& ins_;                                                  ///< The reference to the ins structure
    const ahrs_t& ahrs_;                                        ///< The reference to the attitude estimation structure
    const Manual_control& manual_control_;                      ///< The reference to the manual_control structure
    Mavlink_message_handler& message_handler_;                  ///< The reference to the mavlink message handler

    conf_t config_;



    /**
     * \brief   State machine to drive the navigation module
     *
     */
    void state_machine();

    /**
     * \brief   Drives the critical navigation behavior
     *
     */
    void critical_handler();


    /************************************************
     *      static member functions (callbacks)     *
     ************************************************/

//     /**
//      * \brief   Set a new home position, origin of the local frame
//      *
//      * \param   mission_planner         The pointer to the waypoint handler
//      * \param   sysid                   The system ID
//      * \param   msg                     The received MAVLink message structure with the new home position
//      */
//     static void set_home(Mission_planner* mission_planner, uint32_t sysid, mavlink_message_t* msg);

    /**
     * \brief   Set the next waypoint as current waypoint
     *
     * \param   mission_planner         The pointer to the structure of the mission planner
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t continue_to_next_waypoint(Mission_planner* mission_planner, mavlink_command_long_t* packet);

    /**
     * \brief   Sends back whether the MAV is currently stopped at a waypoint or not
     *
     * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t is_arrived(Mission_planner* mission_planner, mavlink_command_long_t* packet);

    /**
     * \brief   Sets the current waypoint to num_of_waypoint
     *
     * \param   mission_planner         The pointer to the mission planner class
     * \param   sysid                   The system ID
     * \param   msg                     The received MAVLink message structure with the number of the current waypoint
     */
    static void set_current_waypoint(Mission_planner* mission_planner, uint32_t sysid, mavlink_message_t* msg);

    /**
     * \brief   Set the current waypoint to new_current
     *
     * \param   mission_planner         The pointer to the mission planner class
     * \param   packet                  The pointer to the decoded MAVLink message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t set_current_waypoint_from_parameter(Mission_planner* mission_planner, mavlink_command_long_t* packet);

    /**
     * \brief   Sets auto-takeoff procedure from a MAVLink command message MAV_CMD_NAV_TAKEOFF
     *
     * \param   mission_planner     The pointer to the object of the Mission planner takeoff handler
     * \param   packet              The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t set_auto_takeoff(Mission_planner* mission_planner, mavlink_command_long_t* packet);

    /**
     * \brief   Drives the auto landing procedure from the MAV_CMD_NAV_LAND message long
     *
     * \param   mission_planner         The pointer to the structure of the MAVLink waypoint handler
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t set_auto_landing(Mission_planner* mission_planner, mavlink_command_long_t* packet);

    /**
     * \brief   Start/Stop the navigation
     *
     * \param   navigating_handler      The pointer to the structure of the navigating handler
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t start_stop_navigation(Mission_planner* mission_planner, mavlink_command_long_t* packet);
};


Mission_planner::conf_t Mission_planner::default_config()
{
    conf_t conf                                                = {};

    return conf;
};







#endif // MISSION_PLANNER__
