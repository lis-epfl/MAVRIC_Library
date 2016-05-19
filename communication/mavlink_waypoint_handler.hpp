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
 * \author Matthew Douglas
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
#include "control/navigation.hpp"
#include "control/dubin.hpp"

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
     * \param   state                   The reference to the state structure
     * \param   message_handler         The reference to the message handler
     * \param   mavlink_stream          The reference to the MAVLink stream structure
     *
     * \return  True if the init succeed, false otherwise
     */
    Mavlink_waypoint_handler(
                           State& state,
                           Mavlink_message_handler& message_handler,
                           const Mavlink_stream& mavlink_stream,
                           conf_t config = default_config());


    /**
     * \brief   Initialize a first waypoint if a flight plan is set
     *
     * \details Is called by the constructor
     */
    void nav_plan_init();


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
     * \details Returns a copy of the waypoint instead of a reference as
     * the waypoint could be removed/overwritten by the GCS.
     *
     * \return Copy of the current waypoint
     */
    const waypoint_struct_t current_waypoint() const;

    /**
     * \brief Sets the next waypoint as the current one. Should be called when
     * the current waypoint has been reached.
     */
    void advance_to_next_waypoint();

protected:
    waypoint_struct_t waypoint_list_[MAX_WAYPOINTS];              ///< The array of all waypoints (max MAX_WAYPOINTS)

    uint16_t waypoint_count_;                                    ///< The total number of waypoints
    bool hold_waypoint_set_;                                     ///< Flag to tell if the hold position waypoint is set
    uint32_t start_wpt_time_;                                    ///< The time at which the MAV starts to travel towards its waypoint
    const Mavlink_stream& mavlink_stream_;                       ///< The pointer to MAVLink stream

    State& state_;                                               ///< The pointer to the state structure

    /**
     * \brief Gets the current waypoint index
     *
     * \details Finds the current waypoint index by cycling though the
     * waypoint array and returning the first index that has the current
     * field as 1. Returns -1 if there is no current waypoint
     *
     * \return The index of the current waypoint. -1 if no current waypoint
     * found
     */
    int Mavlink_waypoint_handler::current_waypoint_index() const;

private:

    bool waypoint_sending_;                                      ///< Flag to tell whether waypoint are being sent
    bool waypoint_receiving_;                                    ///< Flag to tell whether waypoint are being received or not

    int32_t sending_waypoint_num_;                               ///< The ID number of the sending waypoint
    int32_t waypoint_request_number_;                            ///< The ID number of the requested waypoint
    uint16_t waypoint_onboard_count_;                            ///< The number of waypoint onboard

    uint32_t start_timeout_;                                     ///< The start time for the waypoint timeout
    uint32_t timeout_max_waypoint_;                              ///< The max waiting time for communication

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
