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
 * \file mavlink_waypoint_handler.hpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Matthew Douglas
 *
 * \brief The MAVLink waypoint handler
 *
 ******************************************************************************/


#ifndef MAVLINK_WAYPOINT_HANDLER__
#define MAVLINK_WAYPOINT_HANDLER__

#include "communication/mavlink_communication.hpp"
#include "communication/mavlink_stream.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "communication/state.hpp"
#include "communication/waypoint.hpp"

#define MAX_WAYPOINTS 10        ///< The maximal size of the waypoint list

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mavlink_waypoint_handler
{
public:

    struct conf_t
    {
        float auto_take_off_altitude;                               ///< Altitude to which auto_take off flies; altitude over starting point (where auto_take off was started) should be > 0
    };


    /**
     * \brief   Initialize the waypoint handler
     *
     * \param   mission_planner         The reference to the mission planner
     * \param   navigation              The pointer to the navigation structure
     * \param   state                   The reference to the state structure
     * \param   message_handler         The reference to the message handler
     * \param   mavlink_stream          The reference to the MAVLink stream structure
     * \param   config                  The config structure (optional)
     *
     * \return  True if the init succeed, false otherwise
     */
    Mavlink_waypoint_handler(   Mission_planner& mission_planner,
                                Navigation& navigation,
                                State& state,
                                Mavlink_message_handler& message_handler,
                                const Mavlink_stream& mavlink_stream,
                                conf_t config = default_config());



    inline uint16_t waypoint_count() const {return waypoint_count_;};

    /**
     * \brief   Default configuration
     *
     * \return  Config structure
     */
    static inline conf_t default_config();

    /**
     * \brief Gets the current waypoint
     *
     * \return current waypoint
     */
    const Waypoint& current_waypoint() const;

    /**
     * \brief Gets the next waypoint if available
     *
     * \return next waypoint
     */
    const Waypoint& next_waypoint() const;

    /**
     * \brief   Sets the next waypoint as the current one. Should be called when
     * the current waypoint has been reached.
     */
    void advance_to_next_waypoint();

    /**
     * \brief   Initialize a first waypoint if a flight plan is set
     *
     * TODO: Change name
     */
    void nav_plan_init();

    /**
     * \brief   Updates the waypoint to calculate the local_position_t, radius,
     * loiter_time, and dubin.
     *
     * This is caused by the previous Navigation::waypoint_local_struct_t.
     * TODO: remove
     */
    void update_current_waypoint(global_position_t origin, dubin_state_t* dubin_state);

    /**
     * \brief   Gets the number of waypoints in the waypoint list
     *
     * \return  Waypoint count
     */
    uint16_t waypoint_count() const;

    /**
     * \brief   Gets the current waypoint index
     *
     * \return  Current waypoint index
     */
    uint16_t current_waypoint_index() const;

protected:
    Waypoint waypoint_list_[MAX_WAYPOINTS];                     ///< The array of all waypoints (max MAX_WAYPOINTS)

    uint16_t waypoint_count_;                                   ///< The total number of waypoints
    uint16_t current_waypoint_index_;                           ///< The current waypoint index

    const Mavlink_stream& mavlink_stream_;                      ///< The reference to MAVLink stream
    State& state_;                                              ///< The reference to the state structure
    Mission_planner& mission_planner_;                          ///< The reference to the mission planner class
    Navigation& navigation_;                                    ///< The reference to the navigation class
    
private:

    bool waypoint_sending_;                                     ///< Flag to tell whether waypoint are being sent
    bool waypoint_receiving_;                                   ///< Flag to tell whether waypoint are being received or not

    int32_t sending_waypoint_num_;                              ///< The ID number of the sending waypoint
    int32_t waypoint_request_number_;                           ///< The ID number of the requested waypoint
    uint16_t waypoint_onboard_count_;                           ///< The number of waypoint onboard

    uint32_t start_timeout_;                                    ///< The start time for the waypoint timeout
    uint32_t timeout_max_waypoint_;                             ///< The max waiting time for communication

    conf_t config_;


    /**
     * \brief   Control if time is over timeout and change sending/receiving flags to false
     *
     * \return  The task status
     */
    void control_time_out_waypoint_msg();


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
     * \brief   Initialize a home waypoint at (0,0,0) at start up
     *
     * \details Is called by the constructor
     *
     */
    void init_homing_waypoint();
};


Mavlink_waypoint_handler::conf_t Mavlink_waypoint_handler::default_config()
{
    conf_t conf                                                = {};

    conf.auto_take_off_altitude                                = 10;

    return conf;
};







#endif // MAVLINK_WAYPOINT_HANDLER__
