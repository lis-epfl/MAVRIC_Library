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
 * \file mission_planner_handler_navigating.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the navigating state
 *
 ******************************************************************************/


#ifndef MISSION_PLANNER_HANDLER_NAVIGATING__
#define MISSION_PLANNER_HANDLER_NAVIGATING__

#include "communication/mavlink_message_handler.hpp"
#include "communication/mission_planner_handler.hpp"
#include "communication/state.hpp"
#include "control/manual_control.hpp"
#include "control/navigation.hpp"
#include "sensing/position_estimation.hpp"

class Mavlink_waypoint_handler;
class Mission_planner_handler_landing;

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

class Mission_planner_handler_navigating : public Mission_planner_handler
{
public:


    /**
     * \brief   Initialize the navigating mission planner handler
     *
     * \param   position_estimation                 The pointer to the position estimator structure
     * \param   navigation                          The reference to the navigation structure
     * \param   state                               The reference to the state structure
     * \param   mission_planner                     The reference to the mission planner
     * \param   mavlink_stream                      The reference to the MAVLink stream structure
     * \param   waypoint_handler            The handler for the manual control state
     * \param   mission_planner_handler_landing     The reference to the landing handler
     * \param   message_handler                     The reference to the mavlink message handler
     */
     Mission_planner_handler_navigating(    const Position_estimation& position_estimation,
                                            Navigation& navigation,
                                            State& state,
                                            const Mavlink_stream& mavlink_stream,
                                            Mavlink_waypoint_handler& waypoint_handler,
                                            Mission_planner_handler_landing& mission_planner_handler_landing,
                                            Mavlink_message_handler& message_handler);


    /**
     * \brief   The handler for the takeoff state.
     *
     * \param   mission_planner     The reference to the misison planner that is
     * handling the request.
     */
    virtual void handle(Mission_planner& mission_planner);

    virtual bool init();

protected:
    Navigation& navigation_;                                            ///< The reference to the navigation object
    State& state_;                                                      ///< The reference to the state object
    const Mavlink_stream& mavlink_stream_;                              ///< The reference to the mavlink object
    Mavlink_waypoint_handler& waypoint_handler_;                        ///< The reference to the mavlink waypoint handler
    Mission_planner_handler_landing& mission_planner_handler_landing_;  ///< The reference to the landing handler
    Mavlink_message_handler& message_handler_;                          ///< The reference to the mavlink message handler

    uint32_t travel_time_;                                              ///< The travel time between two waypoints, updated once the MAV arrives at its next waypoint

    /**
     * \brief   Drives the GPS navigation procedure
     *
     * \param   mission_planner         The reference to the misison planner that is
     * handling the request.
     * \param   reset_hold_wpt          Resets the hold waypoint
     * \param   waypoint_coordinates    Output: Waypoint that we are navigating to
     */
    void waypoint_navigating_handler(Mission_planner& mission_planner, bool reset_hold_wpt, Waypoint& waypoint_coordinates);

    /**
     * \brief   Start/Stop the navigation
     *
     * \param   navigating_handler      The pointer to the structure of the navigating handler
     * \param   packet                  The pointer to the structure of the MAVLink command message long
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t start_stop_navigation(Mission_planner_handler_navigating* navigating_handler, mavlink_command_long_t* packet);

    /**
     * \brief   Sends the travel time between the last two waypoints
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     * \param   mavlink_stream          The pointer to the MAVLink stream structure
     * \param   msg                     The pointer to the MAVLink message
     */
    void send_nav_time(const Mavlink_stream* mavlink_stream, mavlink_message_t* msg);
};







#endif // MISSION_PLANNER_HANDLER_NAVIGATING__
