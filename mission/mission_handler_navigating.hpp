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
 * \file mission_handler_navigating.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the navigating state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_NAVIGATING__
#define MISSION_HANDLER_NAVIGATING__

#include "communication/mavlink_message_handler.hpp"
#include "mission/mission_handler.hpp"
#include "mission/navigation.hpp"

class Mavlink_waypoint_handler;

/*
 * The handler class takes in a template parameter that allows control inputs.
 */
template <class T>
class Mission_handler_navigating : public Mission_handler
{
public:


    /**
     * \brief   Initialize the navigating mission planner handler
     *
     * \param   controller                          The reference to the controller
     * \param   ins                                 The reference to the ins
     * \param   navigation                          The reference to the navigation structure
     * \param   mission_planner                     The reference to the mission planner
     * \param   mavlink_stream                      The reference to the MAVLink stream structure
     * \param   waypoint_handler                    The handler for the manual control state
     */
     Mission_handler_navigating(    T& controller,
                                    const INS& ins,
                                    Navigation& navigation,
                                    const Mavlink_stream& mavlink_stream,
                                    Mavlink_waypoint_handler& waypoint_handler);

    /**
     * \brief   Checks if the waypoint is a navigating waypoint
     *  
     * \details     Checks if the inputted waypoint is a:
     *                  MAV_CMD_NAV_WAYPOINT
     *
     * \param   wpt                 The waypoint class
     *
     * \return  Can handle
     */
    virtual bool can_handle(const Waypoint& wpt) const;

    /**
     * \brief   Sets up this handler class for a first time initialization
     *  
     * \details     Records the waypoint reference
     *
     * \param   mission_planner     The mission planner class
     * \param   wpt                 The waypoint class
     *
     * \return  Success
     */
    virtual bool setup(Mission_planner& mission_planner, const Waypoint& wpt);

    /**
     * \brief   Handles the mission every iteration
     *  
     * \details     Handles the navigation to the waypoint and determines the
     *              status code. The status code is 0 for currently navigating
     *              to the waypoint, 1 for waypoint reached and autocontinue,
     *              -1 for control command failed.
     *
     * \param   mission_planner     The mission planner class
     *
     * \return  Status code
     */
    virtual int handle(Mission_planner& mission_planner);

    /**
     * \brief   Returns that the mission state is in MISSION
     *
     * \return  Mission handler's mission state
     */
    virtual Mission_planner::internal_state_t handler_mission_state() const;

protected:
    T& controller_;                                                     ///< The reference to the controller
    const INS& ins_;                                                    ///< The reference to the ins interface
    Navigation& navigation_;                                            ///< The reference to the navigation object
    const Mavlink_stream& mavlink_stream_;                              ///< The reference to the mavlink object
    Mavlink_waypoint_handler& waypoint_handler_;                        ///< The reference to the mavlink waypoint handler

    Waypoint waypoint_;                                                 ///< The waypoint that we are heading towards
    uint64_t start_time_;                                               ///< The start time for travelling to this waypoint
    uint32_t travel_time_;                                              ///< The travel time between two waypoints, updated once the MAV arrives at its next waypoint

    /**
     * \brief   Function to controller specific functions
     *
     * \param   mission_planner     The reference to the mission planner class
     *
     * \return  Controller accepted input
     */
    virtual bool set_controller(Mission_planner& mission_planner);
    
    /**
     * \brief   Sends the travel time between the last two waypoints
     *
     * \param   waypoint_handler        The pointer to the waypoint handler structure
     * \param   mavlink_stream          The pointer to the MAVLink stream structure
     * \param   msg                     The pointer to the MAVLink message
     */
    void send_nav_time(const Mavlink_stream* mavlink_stream, mavlink_message_t* msg);
};

#include "mission/mission_handler_navigating.hxx"

#endif // MISSION_HANDLER_NAVIGATING__
