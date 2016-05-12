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
 * \file mavlink_waypoint_handler_tag.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink tag waypoint handler
 *
 ******************************************************************************/


#include "communication/mavlink_waypoint_handler_tag.hpp"
#include <cstdlib>
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/maths.h"
#include "util/constants.h"
}


Mavlink_waypoint_handler_tag::Mavlink_waypoint_handler_tag(Position_estimation& position_estimation,
                           Navigation& navigation,
                           const ahrs_t& ahrs,
                           State& state,
                           const Manual_control& manual_control,
                           Mavlink_message_handler& message_handler,
                           const Mavlink_stream& mavlink_stream,
                           Offboard_Tag_Search& offboard_tag_search,
                           Mavlink_communication& raspi_mavlink_communication) :
            Mavlink_waypoint_handler(position_estimation,
                                    navigation,
                                    ahrs,
                                    state,
                                    manual_control,
                                    message_handler,
                                    mavlink_stream),
            offboard_tag_search_(offboard_tag_search),
            raspi_mavlink_communication_(raspi_mavlink_communication)
{
    bool init_success = true;

    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_NAV_LAND; // 21
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_CAMERA; // 100
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_auto_landing;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler.add_cmd_callback(&callbackcmd);

    if(!init_success)
    {
        print_util_dbg_print("[MAVLINK_WAYPOINT_HANDLER] constructor: ERROR\r\n");
    }
}


mav_result_t Mavlink_waypoint_handler_tag::set_auto_landing(Mavlink_waypoint_handler_tag* waypoint_handler, mavlink_command_long_t* packet)
{
    // mav_result_t result;
    mav_result_t result = MAV_RESULT_ACCEPTED; // Testing

    print_util_dbg_print("Attempting to land\r\n");

    //if ((waypoint_handler->navigation_.internal_state == NAV_NAVIGATING) || (waypoint_handler->navigation_.internal_state == NAV_HOLD_POSITION))
    {
        // Set waypoint to search for tag and land
        waypoint_handler->navigation_.internal_state_ = Navigation::internal_state_t::NAV_LAND_ON_TAG;
        print_util_dbg_print("internal_state = NAV_LAND_ON_TAG\r\n");

        // Set hold position point and set tag location
        waypoint_handler->waypoint_hold_coordinates.pos[0] = waypoint_handler->position_estimation_.local_position.pos[0];
        waypoint_handler->waypoint_hold_coordinates.pos[1] = waypoint_handler->position_estimation_.local_position.pos[1];
        waypoint_handler->tag_search_altitude_ = -10.0f;
        waypoint_handler->waypoint_hold_coordinates.pos[2] = waypoint_handler->tag_search_altitude_;
        waypoint_handler->offboard_tag_search_.tag_location().pos[0] = 0.0f;
        waypoint_handler->offboard_tag_search_.tag_location().pos[1] = 0.0f;
        waypoint_handler->offboard_tag_search_.tag_location().pos[2] = 0.0f;
        waypoint_handler->offboard_tag_search_.tag_location().heading = waypoint_handler->position_estimation_.local_position.heading;
        waypoint_handler->offboard_tag_search_.tag_location().origin = waypoint_handler->position_estimation_.local_position.origin;

        // Set new tag search start time
        waypoint_handler->tag_search_start_time_ = time_keeper_get_us();

        // Land
        waypoint_handler->offboard_tag_search_.update(&(waypoint_handler->raspi_mavlink_communication_.scheduler()), true);
    }
    /*else
    {
        result = MAV_RESULT_DENIED;
    }*/

    return result;
}


void Mavlink_waypoint_handler_tag::auto_land_on_tag_handler()
{
    float tag_pos[3];
    float cur_pos[3];

    // Set position vectors to shorten code later
    for (uint8_t i = 0; i < 3; i++)
    {
        tag_pos[i] = offboard_tag_search_.tag_location().pos[i];
        cur_pos[i] = position_estimation_.local_position.pos[i];
    }

    bool next_state = false;

    // If the camera has detected the tag and the data is healthy...
    if ((offboard_tag_search_.land_on_tag_behavior() == Offboard_Tag_Search::land_on_tag_behavior_t::TAG_FOUND) &&
        (offboard_tag_search_.is_healthy()))
    {
        // The hold coordinates has been already been updated during the tag location reading...
        // Changed the z goal to ground if we are positioned directly above the tag
        float horizontal_distance_to_tag_sqr = (cur_pos[0] - tag_pos[0]) * (cur_pos[0] - tag_pos[0]) + (cur_pos[1] - tag_pos[1]) * (cur_pos[1] - tag_pos[1]);

        // Set hold location to tag location
        waypoint_hold_coordinates.pos[0] = offboard_tag_search_.tag_location().pos[0];
        waypoint_hold_coordinates.pos[1] = offboard_tag_search_.tag_location().pos[1];

        // If we are not above tag
        if (horizontal_distance_to_tag_sqr > offboard_tag_search_.allowable_horizontal_tag_offset_sqr())
        {
            // Stay at tag search altitude
            waypoint_hold_coordinates.pos[2] = tag_search_altitude_;

        }
        else // Descend to ground
        {
            waypoint_hold_coordinates.pos[2] = 0.0f;

            // Set tag search altitude to current height, so it will reposition itself at this altitude if it drifts away
            tag_search_altitude_ = navigation_.alt_lpf;
        }
    }
    else if (offboard_tag_search_.land_on_tag_behavior() == Offboard_Tag_Search::land_on_tag_behavior_t::TAG_NOT_FOUND)// Else we need to search for the tag ...
    {
        // Don't change the hold coordinates
    }

    // Calculate low-pass filter altitude for when to turn off motors
    navigation_.alt_lpf = navigation_.LPF_gain * (navigation_.alt_lpf) + (1.0f - navigation_.LPF_gain) * position_estimation_.local_position.pos[2];
    if ((position_estimation_.local_position.pos[2] > -0.1f) && (maths_f_abs(position_estimation_.local_position.pos[2] - navigation_.alt_lpf) <= 0.2f))
    {
        // Disarming
        next_state = true;
    }

    // If the tag search has gone on too long, set mode to landing
    if ((time_keeper_get_us() - tag_search_start_time_) > offboard_tag_search_.tag_search_timeout_us())
    {
        print_util_dbg_print("Auto-landing on tag: Timeout: Switch to normal landing\r\n");
        navigation_.internal_state_ = Navigation::internal_state_t::NAV_LANDING;
        navigation_.auto_landing_behavior = Navigation::auto_landing_behavior_t::DESCENT_TO_SMALL_ALTITUDE;
        offboard_tag_search_.land_on_tag_behavior(Offboard_Tag_Search::land_on_tag_behavior_t::TAG_NOT_FOUND);
    }

    if (next_state)
    {
        auto_landing_next_state_ = false;

        switch (navigation_.auto_landing_behavior)
        {
            case Navigation::DESCENT_TO_SMALL_ALTITUDE:
                print_util_dbg_print("Automatic-landing: descent_to_GND\r\n");
                navigation_.auto_landing_behavior = Navigation::DESCENT_TO_GND;
                break;

            case Navigation::DESCENT_TO_GND:
                print_util_dbg_print("Auto-landing: disarming motors \r\n");
                navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
                //state_.mav_mode_custom = CUSTOM_BASE_MODE;
                hold_waypoint_set_ = false;
                navigation_.internal_state_ = Navigation::NAV_ON_GND;
                state_.set_armed(false);
                state_.mav_state_ = MAV_STATE_STANDBY;
                break;
        }
    }
}


void Mavlink_waypoint_handler_tag::state_machine()
{
    mav_mode_t mode_local = state_.mav_mode();

    // Check if it is land on tag state
    if (navigation_.internal_state_ = Navigation::NAV_LAND_ON_TAG)
    {
        auto_land_on_tag_handler();

        navigation_.goal = waypoint_hold_coordinates;
        /*
        float goal_output[3];
        goal_output[0] = waypoint_handler->navigation_.goal.pos[0] - waypoint_handler->position_estimation_.local_position.pos[0];
        goal_output[1] = waypoint_handler->navigation_.goal.pos[1] - waypoint_handler->position_estimation_.local_position.pos[1];
        goal_output[2] = waypoint_handler->navigation_.goal.pos[2] - waypoint_handler->position_estimation_.local_position.pos[2];
        print_util_dbg_print("Goal:");
        print_util_dbg_print_vector(goal_output, 4);
        print_util_dbg_print("\r\n");
        */

        if ((!mav_modes_is_auto(mode_local)) && (!mav_modes_is_guided(mode_local)))
        {
            navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
        }
    }
    else // Call super.state_machine()
    {
        Mavlink_waypoint_handler::state_machine();
    }
}

const float& Mavlink_waypoint_handler_tag::tag_search_altitude() const
{
    return tag_search_altitude_;
}

void Mavlink_waypoint_handler_tag::tag_search_altitude(float alt)
{
    tag_search_altitude_ = alt;
}

const uint32_t Mavlink_waypoint_handler_tag::tag_search_start_time() const
{
    return tag_search_start_time_;
}
