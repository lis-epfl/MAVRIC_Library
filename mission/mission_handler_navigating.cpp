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
 * \file mission_handler_navigating.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 * \author Julien Lecoeur
 *
 * \brief The MAVLink mission planner handler for the navigating state
 *
 ******************************************************************************/


#include "mission/mission_handler_navigating.hpp"
#include "navigation/navigation.hpp"
#include "communication/mavlink_waypoint_handler.hpp"
#include "hal/common/time_keeper.hpp"

void Mission_handler_navigating::send_nav_time(const Mavlink_stream* mavlink_stream_, mavlink_message_t* msg)
{
    mavlink_msg_named_value_int_pack(mavlink_stream_->sysid(),
                                     mavlink_stream_->compid(),
                                     msg,
                                     time_keeper_get_ms(),
                                     "travel_time_",
                                     travel_time_);
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


Mission_handler_navigating::Mission_handler_navigating( const INS& ins,
                                                        const Mavlink_stream& mavlink_stream,
                                                        Mavlink_waypoint_handler& waypoint_handler):
    Mission_handler(),
    ins_(ins),
    mavlink_stream_(mavlink_stream),
    waypoint_handler_(waypoint_handler),
    waypoint_reached_(false),
    start_time_(0),
    travel_time_(0)
{
    waypoint_ = Waypoint(   MAV_FRAME_LOCAL_NED,
                            MAV_CMD_NAV_WAYPOINT,
                            0,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f);
}


bool Mission_handler_navigating::can_handle(const Waypoint& wpt) const
{
    bool handleable = false;

    uint16_t cmd = wpt.command();
    if (cmd == MAV_CMD_NAV_WAYPOINT)
    {
        handleable = true;
    }

    return handleable;
}


bool Mission_handler_navigating::setup(const Waypoint& wpt)
{
    bool success = true;

    start_time_ = time_keeper_get_ms();
    waypoint_reached_ = false;
    waypoint_ = wpt;

    return success;
}


Mission_handler::update_status_t Mission_handler_navigating::update()
{
    /**********************************
    Determine if arrived for first time
    **********************************/
    // Find distance to waypoint
    local_position_t wpt_pos = waypoint_.local_pos();
    float rel_pos[3];
    for (int i = 0; i < 3; i++)
    {
        rel_pos[i] = wpt_pos[i] - ins_.position_lf()[i];
    }
    float dist2wp_sqr = vectors_norm_sqr(rel_pos);

    // Check if radius is available
    float radius;
    if (!waypoint_.radius(radius))
    {
        radius = 4.0f;
    }

    // If we are near the waypoint or are doing dubin circles
    if (dist2wp_sqr < (radius * radius))
    {
        // If we are near the waypoint but the flag has not been set, do this once ...
        if (!waypoint_reached_)
        {
            // Send debug log ...
            print_util_dbg_print("Waypoint reached, distance: ");
            print_util_dbg_putfloat(sqrt(dist2wp_sqr), 3);
            print_util_dbg_print(" m. Less than acceptable radius:");
            print_util_dbg_putfloat(sqrt(radius * radius), 3);
            print_util_dbg_print(" m.\r\n");

            // ... as well as a mavlink message ...
            mavlink_message_t msg;
            mavlink_msg_mission_item_reached_pack(mavlink_stream_.sysid(),
                                                  mavlink_stream_.compid(),
                                                  &msg,
                                                  waypoint_handler_.current_waypoint_index());
            mavlink_stream_.send(&msg);

            // ... and record the travel time ...
            travel_time_ = time_keeper_get_ms() - start_time_;

            // ... and set to waiting at waypoint ...
            waypoint_reached_ = true;
        }
    }

    /*******************
    Determine status code
    ********************/
    // First check if we have reached the waypoint
    if (waypoint_reached_)
    {
        // Then ensure that we are in autocontinue
        if ((waypoint_.autocontinue() == 1) && (waypoint_handler_.waypoint_count() > 1))
        {
            // Differentiate between dubin and direct to
            return MISSION_FINISHED;
        }
    }

    return MISSION_IN_PROGRESS;
}


bool Mission_handler_navigating::write_flight_command(Flight_controller& flight_controller) const
{
    position_command_t cmd;

    // Position and heading from waypoint
    float heading = 0.0f;
    waypoint_.heading(heading);
	cmd.xyz        = waypoint_.local_pos();
    cmd.heading    = heading;

	return flight_controller.set_command(cmd);
}


Mission_planner::internal_state_t Mission_handler_navigating::handler_mission_state() const
{
    return Mission_planner::MISSION;
}
