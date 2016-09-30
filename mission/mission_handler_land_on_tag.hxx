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
 * \file mission_handler_land_on_tag.hxx
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler functions for the land on tag state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_LAND_ON_TAG_HXX__
#define MISSION_HANDLER_LAND_ON_TAG_HXX__

#include "hal/common/time_keeper.hpp"

template <class T1, class T2, class T3>
Mission_handler_land_on_tag<T1, T2, T3>::Mission_handler_land_on_tag(   T1& fly_to_landing_location_controller,
                                                                        T2& desc_to_small_alt_controller,
                                                                        T3& desc_to_ground_controller,
                                                                        const ahrs_t& ahrs,
                                                                        const INS& ins,
                                                                        State& state,
                                                                        Offboard_Tag_Search& offboard_tag_search,
                                                                        Mission_planner& mission_planner,
                                                                        Mavlink_message_handler& message_handler,
                                                                        conf_t config):
            Mission_handler(),
            is_landed_(false),
            alt_lpf_(0.0f),
            tag_search_altitude_(0.0f),
            tag_search_start_time_(0),
            fly_to_landing_location_controller_(fly_to_landing_location_controller),
            desc_to_small_alt_controller_(desc_to_small_alt_controller),
            desc_to_ground_controller_(desc_to_ground_controller),
            ahrs_(ahrs),
            ins_(ins),
            state_(state),
            offboard_tag_search_(offboard_tag_search),
            mission_planner_(mission_planner)
{
    waypoint_ = Waypoint (  MAV_FRAME_LOCAL_NED,
                            MAV_CMD_NAV_LAND_ON_TAG,
                            0,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f);
    auto_landing_behavior_ = DESCENT_TO_SMALL_ALTITUDE;
    LPF_gain_ = config.LPF_gain;
    desc_to_ground_altitude_ = config.desc_to_ground_altitude;
    desc_to_ground_range_ = config.desc_to_ground_range;
    min_waypoint_acceptance_radius_ = config.min_waypoint_acceptance_radius;

    bool init_success = true;

    // Add callbacks for landing on the tag
    init_success &= message_handler.add_cmd_callback(   MAV_CMD_NAV_LAND, // 300
                                                        MAVLINK_BASE_STATION_ID,
                                                        MAV_COMP_ID_ALL,
                                                        MAV_COMP_ID_ALL, // 190
                                                        &set_auto_landing_tag,
                                                        this );

    if(!init_success)
    {
        print_util_dbg_print("[MAVLINK_WAYPOINT_HANDLER] constructor: ERROR\r\n");
    }
}

template <class T1, class T2, class T3>
bool Mission_handler_land_on_tag<T1, T2, T3>::can_handle(const Waypoint& wpt) const
{
    bool handleable = false;

    uint16_t cmd = wpt.command();
    if (cmd == MAV_CMD_NAV_LAND_ON_TAG)
    {
        handleable = true;
    }

    return handleable;
}

template <class T1, class T2, class T3>
bool Mission_handler_land_on_tag<T1, T2, T3>::setup(const Waypoint& wpt)
{
    bool success = true;

    print_util_dbg_print("Automatic-landing: fly_to_landing_location\r\n");
    auto_landing_behavior_ = FLY_TO_LANDING_LOCATION;
    state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
    state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_SMALL_ALTITUDE;
    is_landed_ = false;
    tag_search_altitude_ = wpt.local_pos()[Z];
    tag_search_start_time_ = time_keeper_get_us();
    alt_lpf_ = ins_.position_lf()[Z];

    waypoint_ = wpt;

    // Set hold position point and set tag location
    tag_search_altitude_ = wpt.local_pos()[Z];
    offboard_tag_search_.tag_location()[0] = 0.0f;
    offboard_tag_search_.tag_location()[1] = 0.0f;
    offboard_tag_search_.land_on_tag_behavior(Offboard_Tag_Search::land_on_tag_behavior_t::TAG_NOT_FOUND);

    // Land
    offboard_tag_search_.update(true);

    

    return success;
}

template <class T1, class T2, class T3>
Mission_handler::update_status_t Mission_handler_land_on_tag<T1, T2, T3>::update()
{
    /*****************************
    Handle internal landing states 
    *****************************/
    // Determine if we should switch between the landing states
    bool next_state = false;
    switch (auto_landing_behavior_)
    {
        case FLY_TO_LANDING_LOCATION:
            {
                // Reset alt_lpf_ to keep it with the actual altitude
                alt_lpf_ = ins_.position_lf()[2];
                local_position_t pos = ins_.position_lf();
                local_position_t wpt_pos = waypoint_.local_pos();

                // Accept that we've arrived at waypoint if we are within 30% height or config.min_waypoint_acceptance_radius meters
                float acceptance_radius = min_waypoint_acceptance_radius_;
                if (-0.3*wpt_pos[Z] > acceptance_radius)
                {
                    acceptance_radius = -0.3*wpt_pos[Z];
                }

                if (((pos[X]-wpt_pos[X])*(pos[X]-wpt_pos[X])+(pos[Y]-wpt_pos[Y])*(pos[Y]-wpt_pos[Y])+(pos[Z]-wpt_pos[Z])*(pos[Z]-wpt_pos[Z])) <= acceptance_radius)
                {
                    next_state = true;
                }
            }
            break;

        case DESCENT_TO_SMALL_ALTITUDE:
            // Use a different LPF gain to descend faster when tag is found
            alt_lpf_ = 0.5f * alt_lpf_ + (1.0f - 0.5f) * ins_.position_lf()[2];
            if (maths_f_abs(ins_.position_lf()[Z] - desc_to_ground_altitude_) < desc_to_ground_range_ ||
                (ins_.position_lf()[Z] > desc_to_ground_altitude_))
            {
                next_state = true;
            }
            break;

        case DESCENT_TO_GND:
            // Use a high LPF gain to descend to ground to know when we have stopped descending
            alt_lpf_ = LPF_gain_ * alt_lpf_ + (1.0f - LPF_gain_) * ins_.position_lf()[2];
            if ((ins_.position_lf()[Z] > -0.1f) && (maths_f_abs(ins_.position_lf()[Z] - alt_lpf_) <= 0.2f))
            {
                next_state = true;
            }
            break; 
    }
    
    /*************
    Set controller 
    *************/
    bool ret = false;
    switch (auto_landing_behavior_)
    {
        case FLY_TO_LANDING_LOCATION:
            ret = set_fly_to_landing_location_control_command();
            break;

        case DESCENT_TO_SMALL_ALTITUDE:
            ret = set_desc_to_small_alt_control_command();
            break;

        case DESCENT_TO_GND:
            ret = set_desc_to_ground_control_command();
            break;
    }

    // If we are switching between states, ... then switch
    if (next_state)
    {
        switch (auto_landing_behavior_)
        {
            case FLY_TO_LANDING_LOCATION:
                print_util_dbg_print("Automatic-landing: descent_to_small_alt\r\n");
                auto_landing_behavior_ = DESCENT_TO_SMALL_ALTITUDE;
                state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_SMALL_ALTITUDE;
                break;

            case DESCENT_TO_SMALL_ALTITUDE:
                print_util_dbg_print("Automatic-landing: descent_to_GND\r\n");
                auto_landing_behavior_ = DESCENT_TO_GND;
                state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
                state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_GND;
                break;

            case DESCENT_TO_GND:
                print_util_dbg_print("Auto-landing: disarming motors \r\n");
                auto_landing_behavior_ = FLY_TO_LANDING_LOCATION;
                //Do not reset custom flag here, to be able to check after landing
                // in case something went wrong. Is reset while arming
                offboard_tag_search_.set_is_camera_running(false);       // Dont send messages to take photos
                state_.set_armed(false);
                state_.mav_state_ = MAV_STATE_STANDBY;
                is_landed_ = true;
                break;
        }
    }

    /********************
    Determine status code 
    ********************/
    if (waypoint_.autocontinue() == 1 && is_landed_)
    {
        return MISSION_FINISHED;
    }

    // Handle control command failed status
    if (!ret)
    {
        return MISSION_FAILED;
    }

    // If the tag search has gone on too long, set mode to landing
    if ((time_keeper_get_us() - tag_search_start_time_) > offboard_tag_search_.tag_search_timeout_us() &&
        !is_landed_)
    {
        print_util_dbg_print("Auto-landing on tag: Timeout: Switch to normal landing\r\n");
        local_position_t landing_position = ins_.position_lf();
        float heading = coord_conventions_get_yaw(ahrs_.qe);
        Waypoint landing_wpt(   MAV_FRAME_LOCAL_NED,
                                MAV_CMD_NAV_LAND,
                                0,
                                0.0f,
                                0.0f,
                                0.0f,
                                heading,
                                landing_position[X],
                                landing_position[Y],
                                landing_position[Z]);
        mission_planner_.insert_ad_hoc_waypoint(landing_wpt);
        offboard_tag_search_.land_on_tag_behavior(Offboard_Tag_Search::land_on_tag_behavior_t::TAG_NOT_FOUND);
        offboard_tag_search_.set_is_camera_running(false);       // Dont send messages to take photos
    }

    return MISSION_IN_PROGRESS;
}

template <class T1, class T2, class T3>
Mission_planner::internal_state_t Mission_handler_land_on_tag<T1, T2, T3>::handler_mission_state() const
{
    return Mission_planner::POSTMISSION;
}

template <class T1, class T2, class T3>
mav_result_t Mission_handler_land_on_tag<T1, T2, T3>::set_auto_landing_tag(Mission_handler_land_on_tag* land_on_tag_handler, const mavlink_command_long_t* packet)
{
    // mav_result_t result;
    mav_result_t result = MAV_RESULT_ACCEPTED; // Testing

    print_util_dbg_print("Attempting to land on tag\r\n");

    if (   (land_on_tag_handler->mission_planner_.internal_state() == Mission_planner::MISSION)
        || (land_on_tag_handler->mission_planner_.internal_state() == Mission_planner::PAUSED))
    {
        result = MAV_RESULT_ACCEPTED;

        land_on_tag_handler->offboard_tag_search_.increment_start_tag_msg_count();

        // Determine landing location
        Waypoint landing_wpt;
        float heading = coord_conventions_get_yaw(land_on_tag_handler->ahrs_.qe);

        // Specific spot case
        if (packet->param1 == 1)
        {
            print_util_dbg_print("Searching for tag at a given local location\r\n");
            landing_wpt = Waypoint( MAV_FRAME_LOCAL_NED,
                                    MAV_CMD_NAV_LAND_ON_TAG,
                                    0,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    heading,
                                    packet->param5,
                                    packet->param6,
                                    packet->param7);
        }
        else
        {
            print_util_dbg_print("Searching for tag on the spot\r\n");
            local_position_t landing_position = land_on_tag_handler->ins_.position_lf();
            landing_wpt = Waypoint( MAV_FRAME_LOCAL_NED,
                                    MAV_CMD_NAV_LAND_ON_TAG,
                                    0,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    heading,
                                    landing_position[X],
                                    landing_position[Y],
                                    landing_position[Z]);
        }

        // Set hold position for landing
        land_on_tag_handler->mission_planner_.insert_ad_hoc_waypoint(landing_wpt);

        print_util_dbg_print("Land on tag procedure initialised.\r\n");
        print_util_dbg_print("Landing at: (");
        local_position_t local_pos = landing_wpt.local_pos();
        print_util_dbg_print_num(local_pos[X], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(local_pos[Y], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(local_pos[Z], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num((int32_t)(heading * 180.0f / 3.14f), 10);
        print_util_dbg_print(")\r\n");
    }
    else
    {
        result = MAV_RESULT_DENIED;
    }

    return result;
}

#endif // MISSION_HANDLER_LAND_ON_TAG_HXX__
