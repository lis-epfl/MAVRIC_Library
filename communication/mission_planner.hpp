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
#include "sensing/position_estimation.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "communication/state.hpp"
#include "sensing/qfilter.hpp"
#include "control/manual_control.hpp"
#include "control/navigation.hpp"
#include "control/dubin.hpp"

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
     * \param   position_estimation     The pointer to the position estimator structure
     * \param   navigation              The pointer to the navigation structure
     * \param   ahrs                    The pointer to the attitude estimation structure
     * \param   state                   The pointer to the state structure
     * \param   manual_control          The pointer to the manual control structure
     * \param   mavlink_communication   The pointer to the MAVLink communication structure
     * \param   mavlink_stream          The pointer to the MAVLink stream structure
     *
     * \return  True if the init succeed, false otherwise
     */
    Mission_planner(    Position_estimation& position_estimation,
                        Navigation& navigation,
                        const ahrs_t& ahrs,
                        State& state,
                        const Manual_control& manual_control,
                        Mavlink_message_handler& message_handler,
                        const Mavlink_stream& mavlink_stream,
                        conf_t config = default_config());


    /**
     * \brief   The waypoint handler tasks, gives a goal for the navigation module
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     */
    static bool update(Mavlink_waypoint_handler* waypoint_handler);

    /**
     * \brief   Initialize a first waypoint if a flight plan is set
     *
     * \details Is called by the constructor
     */
    void nav_plan_init();

    /**
     * \brief   Initialise the position hold mode
     *
     * \param   local_pos               The position where the position will be held
     */
    void hold_init(local_position_t local_pos);

    /**
     * \brief   Default configuration
     *
     * \return  Config structure
     */
    static inline conf_t default_config();


    waypoint_local_struct_t waypoint_hold_coordinates;           ///< The coordinates of the waypoint in position hold mode (MAV_MODE_GUIDED_ARMED)

protected:
    bool hold_waypoint_set_;                                     ///< Flag to tell if the hold position waypoint is set
    uint32_t start_wpt_time_;                                    ///< The time at which the MAV starts to travel towards its waypoint
    const Mavlink_stream& mavlink_stream_;                       ///< The reference to MAVLink stream

    State& state_;                                               ///< The reference to the state structure
    Navigation& navigation_;                                     ///< The reference to the navigation structure
    Position_estimation& position_estimation_;                   ///< The reference to the position estimation structure

    Mission_planner_handler& on_ground_handler_;                ///< The handler for the on ground state
    Mission_planner_handler& takeoff_handler_;                  ///< The handler for the takeoff state
    Mission_planner_handler& landing_handler_;                  ///< The handler for the landing state
    Mission_planner_handler& hold_position_handler_;            ///< The handler for the hold position state
    Mission_planner_handler& stop_on_position_handler_;         ///< The handler for the stop on position state
    Mission_planner_handler& stop_there_handler_;               ///< The handler for the stop there state
    Mission_planner_handler& navigating_handler_;               ///< The handler for the navigating state
    Mission_planner_handler& manual_control_handler_;           ///< The handler for the manual control state

private:

    waypoint_local_struct_t waypoint_coordinates_;               ///< The coordinates of the waypoint in GPS navigation mode (MAV_MODE_AUTO_ARMED)
    waypoint_local_struct_t waypoint_critical_coordinates_;      ///< The coordinates of the waypoint in critical state
    waypoint_local_struct_t waypoint_next_;                       ///< The coordinates of the next waypoint

    waypoint_struct_t current_waypoint_;                         ///< The structure of the current waypoint
    waypoint_struct_t next_waypoint_;                            ///< The structure of the next waypoint

    uint32_t travel_time_;                                       ///< The travel time between two waypoints, updated once the MAV arrives at its next waypoint

    bool critical_next_state_;                                   ///< Flag to change critical state in its dedicated state machine

    mav_mode_t last_mode_;                                       ///< The mode of the MAV to have a memory of its evolution
    const ahrs_t& ahrs_;                                         ///< The reference to the attitude estimation structure
    const Manual_control& manual_control_;                       ///< The reference to the manual_control structure
    conf_t config_;



    /**
     * \brief   State machine to drive the navigation module
     *
     */
    void state_machine();

    /**
     * \brief   Computes the state machine for the Dubin navigation type
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     * \param   waypoint_next_           The next waypoint structure
     */
    void dubin_state_machine(waypoint_local_struct_t* waypoint_next_);

    /**
     * \brief   Drives the critical navigation behavior
     *
     */
    void critical_handler();

    /**
     * \brief   Check if the nav mode is equal to the state mav mode
     *
     * \return  True if the flag STABILISE, GUIDED and ARMED are equal, false otherwise
     */
    bool mode_change();


    /**
     * \brief   Sends the travel time between the last two waypoints
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     * \param   mavlink_stream          The pointer to the MAVLink stream structure
     * \param   msg                     The pointer to the MAVLink message
     */
    void send_nav_time(const Mavlink_stream* mavlink_stream, mavlink_message_t* msg);


    /************************************************
     *      static member functions (callbacks)     *
     ************************************************/


    /**
     * \brief   Set the current waypoint to new_current
     *
     * \param   waypoint_handler        The pointer to the waypoint handler
     * \param   packet                  The pointer to the decoded MAVLink message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t set_current_waypoint_from_parameter(Mavlink_waypoint_handler* waypoint_handler, mavlink_command_long_t* packet);

    /**
     * \brief   Set a new home position, origin of the local frame
     *
     * \param   waypoint_handler        The pointer to the waypoint handler
     * \param   sysid                   The system ID
     * \param   msg                     The received MAVLink message structure with the new home position
     */
    static void set_home(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

    /**
     * \brief   Set the next waypoint as current waypoint
     *
     * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t continue_to_next_waypoint(Mavlink_waypoint_handler* waypoint_handler, mavlink_command_long_t* packet);

    /**
     * \brief   Sends back whether the MAV is currently stopped at a waypoint or not
     *
     * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t is_arrived(Mavlink_waypoint_handler* waypoint_handler, mavlink_command_long_t* packet);
};


Mission_planner::conf_t Mavlink_waypoint_handler::default_config()
{
    conf_t conf                                                = {};

    return conf;
};







#endif // MISSION_PLANNER__
