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
 * \file mavlink_waypoint_handler.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief The MAVLink waypoint handler
 *
 ******************************************************************************/


#ifndef MAVLINK_WAYPOINT_HANDLER__
#define MAVLINK_WAYPOINT_HANDLER__

#include "communication/mavlink_communication.hpp"
#include "communication/mavlink_stream.hpp"
#include "sensing/position_estimation.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "communication/state.hpp"
#include "sensing/qfilter.hpp"
#include "control/manual_control.hpp"
#include "automatic_navigation/navigation.hpp"

#define MAX_WAYPOINTS 10        ///< The maximal size of the waypoint list

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */


class Mavlink_waypoint_handler
{
public:

    /**
     * \brief   The MAVLink waypoint structure
     */
    typedef struct
    {
        uint8_t frame;                                              ///< The reference frame of the waypoint
        uint16_t command;                                           ///< The MAV_CMD_NAV id of the waypoint
        uint8_t current;                                            ///< Flag to tell whether the waypoint is the current one or not
        uint8_t autocontinue;                                       ///< Flag to tell whether the vehicle should auto continue to the next waypoint once it reaches the current waypoint
        float param1;                                               ///< Parameter depending on the MAV_CMD_NAV id
        float param2;                                               ///< Parameter depending on the MAV_CMD_NAV id
        float param3;                                               ///< Parameter depending on the MAV_CMD_NAV id
        float param4;                                               ///< Parameter depending on the MAV_CMD_NAV id
        double x;                                                   ///< The value on the x axis (depends on the reference frame)
        double y;                                                   ///< The value on the y axis (depends on the reference frame)
        double z;                                                   ///< The value on the z axis (depends on the reference frame)
    } waypoint_struct_t;


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
    Mavlink_waypoint_handler(Position_estimation& position_estimation,
                           Navigation& navigation,
                           const ahrs_t& ahrs,
                           State& state,
                           const Manual_control& manual_control,
                           Mavlink_message_handler& message_handler,
                           const Mavlink_stream& mavlink_stream);


    /**
     * \brief   The waypoint handler tasks, gives a goal for the navigation module
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     */
    static bool update(Mavlink_waypoint_handler* waypoint_handler);

    /**
     * \brief   Initialize a home waypoint at (0,0,0) at start up
     *
     * \details Is called by the constructor
     *
     */
    void init_homing_waypoint();

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

    inline uint16_t waypoint_count() const {return waypoint_count_;};

    
    local_position_t waypoint_hold_coordinates;                 ///< The coordinates of the waypoint in position hold mode (MAV_MODE_GUIDED_ARMED)
    waypoint_struct_t waypoint_list[MAX_WAYPOINTS];             ///< The array of all waypoints (max MAX_WAYPOINTS)

protected:
    uint16_t waypoint_count_;                                     ///< The total number of waypoints
    int8_t current_waypoint_index_;                              ///< The number of the current waypoint
    bool hold_waypoint_set_;                                     ///< Flag to tell if the hold position waypoint is set
    uint32_t start_wpt_time_;                                    ///< The time at which the MAV starts to travel towards its waypoint
    const Mavlink_stream& mavlink_stream_;                       ///< The pointer to MAVLink stream
    
    State& state_;                                               ///< The pointer to the state structure
    Navigation& navigation_;                                     ///< The pointer to the navigation structure
    Position_estimation& position_estimation_;                   ///< The pointer to the position estimation structure

private:
    waypoint_struct_t current_waypoint_;                         ///< The structure of the current waypoint
    local_position_t waypoint_coordinates_;                      ///< The coordinates of the waypoint in GPS navigation mode (MAV_MODE_AUTO_ARMED)
    local_position_t waypoint_critical_coordinates_;             ///< The coordinates of the waypoint in critical state

    bool waypoint_sending_;                                      ///< Flag to tell whether waypoint are being sent
    bool waypoint_receiving_;                                    ///< Flag to tell whether waypoint are being received or not

    int32_t sending_waypoint_num_;                               ///< The ID number of the sending waypoint
    int32_t waypoint_request_number_;                            ///< The ID number of the requested waypoint
    uint16_t waypoint_onboard_count_;                            ///< The number of waypoint onboard

    uint32_t start_timeout_;                                     ///< The start time for the waypoint timeout
    uint32_t timeout_max_waypoint_;                              ///< The max waiting time for communication
    uint32_t travel_time_;                                       ///< The travel time between two waypoints, updated once the MAV arrives at its next waypoint

    bool critical_next_state_;                                   ///< Flag to change critical state in its dedicated state machine
    bool auto_landing_next_state_;                               ///< Flag to change critical state in its dedicated state machine

    mav_mode_t last_mode_;                                       ///< The mode of the MAV to have a memory of its evolution    
    const ahrs_t& ahrs_;                                         ///< The pointer to the attitude estimation structure
    const Manual_control& manual_control_;                       ///< The pointer to the manual_control structure

    /**
     * \brief   Drives the stopping behavior
     *
     */
    void stopping_handler();

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

    /**
     * \brief   Drives the GPS navigation procedure
     *
     */
    void waypoint_navigation_handler(bool reset_hold_wpt);

    /**
     * \brief   Drives the automatic takeoff procedure
     *
     */
    bool take_off_handler();

    /**
     * \brief   Drives the auto-landing navigation behavior
     *
     */
    void auto_landing_handler();

    /**
     * \brief   Check if the nav mode is equal to the state mav mode
     *
     * \return  True if the flag STABILISE, GUIDED and ARMED are equal, false otherwise
     */
    bool mode_change();

    /**
     * \brief   Control if time is over timeout and change sending/receiving flags to false
     *
     * \return  The task status
     */
    void control_time_out_waypoint_msg();

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
     * \brief   Sends the number of onboard waypoint to MAVLink when asked by ground station
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     * \param   sysid                   The system ID
     * \param   msg                     The pointer to the received MAVLink message structure asking the send count
     */
    static void send_count(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

    /**
     * \brief   Sends a given waypoint via a MAVLink message
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     * \param   sysid                   The system ID
     * \param   msg                     The pointer to the received MAVLink message structure asking for a waypoint
     */
    static void send_waypoint(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

    /**
     * \brief   Receives a acknowledge message from MAVLink
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     * \param   sysid                   The system ID
     * \param   msg                     The received MAVLink message structure
     */
    static void receive_ack_msg(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

    /**
     * \brief   Receives the number of waypoints that the ground station is sending
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     * \param   sysid                   The system ID
     * \param   msg                     The received MAVLink message structure with the total number of waypoint
     */
    static void receive_count(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

    /**
     * \brief   Receives a given waypoint and stores it in the local structure
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     * \param   sysid                   The system ID
     * \param   msg                     The received MAVLink message structure with the waypoint
     */
    static void receive_waypoint(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

    /**
     * \brief   Sets the current waypoint to num_of_waypoint
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     * \param   sysid                   The system ID
     * \param   msg                     The received MAVLink message structure with the number of the current waypoint
     */
    static void set_current_waypoint(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

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
     * \brief   Clears the waypoint list
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     * \param   sysid                   The system ID
     * \param   msg                     The received MAVLink message structure with the clear command
     */
    static void clear_waypoint_list(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

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

    /**
     * \brief   Start/Stop the navigation
     *
     * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t start_stop_navigation(Mavlink_waypoint_handler* waypoint_handler, mavlink_command_long_t* packet);

    /**
     * \brief   Sets auto-takeoff procedure from a MAVLink command message MAV_CMD_NAV_TAKEOFF
     *
     * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
     * \param   packet              The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t set_auto_takeoff(Mavlink_waypoint_handler* waypoint_handler, mavlink_command_long_t* packet);

    /**
     * \brief   Drives the auto landing procedure from the MAV_CMD_NAV_LAND message long
     *
     * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t set_auto_landing(Mavlink_waypoint_handler* waypoint_handler, mavlink_command_long_t* packet);
};










#endif // MAVLINK_WAYPOINT_HANDLER__