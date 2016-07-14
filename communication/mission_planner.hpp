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
#include "control/navigation.hpp"
#include "control/dubin.hpp"

class Mission_planner_handler_takeoff;
class Mission_planner_handler_landing;
class Mission_planner_handler_on_ground;
class Mission_planner_handler_navigating;
class Mission_planner_handler_stop_there;
class Mission_planner_handler_stop_on_position;
class Mission_planner_handler_takeoff;
class Mission_planner_handler_manual_control;
class Mission_planner_handler_hold_position;

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_planner
{
public:

    struct conf_t
    {
    };


    /**
     * \brief   Initialize the waypoint handler
     *
     * \param   position_estimation         The pointer to the position estimator structure
     * \param   navigation                  The pointer to the navigation structure
     * \param   ahrs                        The pointer to the attitude estimation structure
     * \param   state                       The pointer to the state structure
     * \param   manual_control              The pointer to the manual control structure
     * \param   mavlink_communication       The pointer to the MAVLink communication structure
     * \param   mavlink_stream              The pointer to the MAVLink stream structure
     * \param   on_ground_handler           The handler for the on ground state
     * \param   takeoff_handler             The handler for the takeoff state
     * \param   landing_handler             The handler for the landing state
     * \param   hold_position_handler       The handler for the hold position state
     * \param   stop_on_position_handler    The handler for the stop on position state
     * \param   stop_there_handler          The handler for the stop there state
     * \param   navigating_handler          The handler for the navigating state
     * \param   manual_control_handler      The handler for the manual control state
     * \param   waypoint_handler    The handler for the manual control state
     *
     * \return  True if the init succeed, false otherwise
     */
    Mission_planner(    Position_estimation& position_estimation,
                        Navigation& navigation,
                        const ahrs_t& ahrs,
                        State& state,
                        const Manual_control& manual_control,
                        Mavlink_message_handler& message_handler,
                        Mavlink_stream& mavlink_stream,
                        Mission_planner_handler_on_ground& on_ground_handler,
                        Mission_planner_handler_takeoff& takeoff_handler,
                        Mission_planner_handler_landing& landing_handler,
                        Mission_planner_handler_hold_position& hold_position_handler,
                        Mission_planner_handler_stop_on_position& stop_on_position_handler,
                        Mission_planner_handler_stop_there& stop_there_handler,
                        Mission_planner_handler_navigating& navigating_handler,
                        Mission_planner_handler_manual_control& manual_control_handler,
                        Mavlink_waypoint_handler& waypoint_handler,
                        conf_t config = default_config());


    /**
     * \brief   The mission planner tasks, gives a goal for the navigation module
     *
     * \param   mission_planner     The pointer to the waypoint handler structure
     */
    static bool update(Mission_planner* mission_planner);

    /**
     * \brief   Initialise the position hold mode
     *
     * \param   local_pos               The position where the position will be held
     */
    void hold_init(local_position_t local_pos);

    /**
     * \brief   Computes the state machine for the Dubin navigation type
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     * \param   waypoint_next_           The next waypoint structure
     */
    void dubin_state_machine(Waypoint* waypoint_next_);

    /**
     * \brief   Initialise the position hold mode in Dubin navigation
     *
     * \param   local_pos               The position where the position will be held
     */
    void dubin_hold_init(local_position_t local_pos);

    /**
     * \brief   Check if the nav mode is equal to the state mav mode
     *
     * \return  True if the flag STABILISE, GUIDED and ARMED are equal, false otherwise
     */
    bool mode_change();

    /**
     * \brief   Default configuration
     *
     * \return  Config structure
     */
    static inline conf_t default_config();


    Waypoint waypoint_hold_coordinates;           ///< The coordinates of the waypoint in position hold mode (MAV_MODE_GUIDED_ARMED)

    void set_hold_waypoint_set(bool hold_waypoint_set);
    bool hold_waypoint_set() const;
    Mav_mode last_mode() const;
    void set_critical_next_state(bool critical_next_state);

    /**
     * \brief   Gets a flag stating if the drone is currently waiting at a waypoint
     *
     * \return  Is drone waiting at a waypoint
     */
    bool waiting_at_waypoint() const;

    /**
     * \brief   Sets the flag stating if the drone is currently waiting at a waypoint
     */
    void set_waiting_at_waypoint(bool waiting_at_waypoint);

    /**
     * \brief   Sets the start_wpt_time_ to the current time
     */
    void set_start_wpt_time();

    /**
     * \brief   Gets the time that the drone started to move towards the waypoint
     *
     * \return  start_wpt_time
     */
    uint32_t start_wpt_time() const;
protected:
    Mission_planner_handler_on_ground& on_ground_handler_;                  ///< The handler for the on ground state
    Mission_planner_handler_takeoff& takeoff_handler_;                      ///< The handler for the takeoff state
    Mission_planner_handler_landing& landing_handler_;                      ///< The handler for the landing state
    Mission_planner_handler_hold_position& hold_position_handler_;          ///< The handler for the hold position state
    Mission_planner_handler_stop_on_position& stop_on_position_handler_;    ///< The handler for the stop on position state
    Mission_planner_handler_stop_there& stop_there_handler_;                ///< The handler for the stop there state
    Mission_planner_handler_navigating& navigating_handler_;                ///< The handler for the navigating state
    Mission_planner_handler_manual_control& manual_control_handler_;        ///< The handler for the manual control state

    Mavlink_waypoint_handler& waypoint_handler_;        ///< The reference to the mavlink waypoint handler

    bool hold_waypoint_set_;                                    ///< Flag to tell if the hold position waypoint is set
    uint32_t start_wpt_time_;                                   ///< The time at which the MAV starts to travel towards its waypoint
    bool waiting_at_waypoint_;                                  ///< Flag stating if the drone is currently at a waypoint waiting to advance
    bool critical_next_state_;                                  ///< Flag to change critical state in its dedicated state machine
    Mav_mode last_mode_;                                        ///< The mode of the MAV to have a memory of its evolution
    Waypoint waypoint_critical_coordinates_;                    ///< Waypoint for the critical state

    Mavlink_stream& mavlink_stream_;                      ///< The reference to MAVLink stream
    State& state_;                                              ///< The reference to the state structure
    Navigation& navigation_;                                    ///< The reference to the navigation structure
    Position_estimation& position_estimation_;                  ///< The reference to the position estimation structure
    const ahrs_t& ahrs_;                                        ///< The reference to the attitude estimation structure
    const Manual_control& manual_control_;                      ///< The reference to the manual_control structure

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

    /**
     * \brief   Set a new home position, origin of the local frame
     *
     * \param   mission_planner         The pointer to the waypoint handler
     * \param   sysid                   The system ID
     * \param   msg                     The received MAVLink message structure with the new home position
     */
    static void set_home(Mission_planner* mission_planner, uint32_t sysid, mavlink_message_t* msg);

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
};


Mission_planner::conf_t Mission_planner::default_config()
{
    conf_t conf                                                = {};

    return conf;
};







#endif // MISSION_PLANNER__
