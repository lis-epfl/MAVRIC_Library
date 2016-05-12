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
 * \file offboard_tag_search.c
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief Offboard camera control
 *
 ******************************************************************************/


#include "sensing/offboard_tag_search.hpp"
#include "sample_projects/LEQuad/tasks.hpp"
#include "sample_projects/LEQuad/central_data.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
#include "util/constants.h"
#include "util/maths.h"
}



Offboard_Tag_Search::Offboard_Tag_Search(Position_estimation& position_estimation, const ahrs_t& ahrs, Mavlink_waypoint_handler_tag& waypoint_handler, offboard_tag_search_conf_t config):
    conf_(config),
    is_camera_running_(config.initial_camera_state),
    last_update_us_(time_keeper_get_us()),
    position_estimation_(position_estimation),
    ahrs_(ahrs),
    waypoint_handler_(waypoint_handler)
{
    // Set picture count to 0
    picture_count_ = 0;
}


bool Offboard_Tag_Search::update(const Scheduler* scheduler, bool camera_state)
{
    bool success = true;
    
    // Switch camera on and off
    is_camera_running_ = camera_state;

    // Update timing
    uint32_t t      = time_keeper_get_us();
    //dt_s_           = (float)(t - last_update_us_) / 1000000.0f;
    last_update_us_ = t;

    // Send the message now
    Scheduler_task* camera_send_message_task = scheduler->get_task_by_id(MAVLINK_MSG_ID_COMMAND_LONG);
    camera_send_message_task->run_now();

    return success;
}


bool Offboard_Tag_Search::is_healthy() const
{
    // Check last detection time
    if ((time_keeper_get_us() - last_update_us()) > conf_.max_acc_time_since_last_detection_us)
    {
        return false;
    }
    else
    {
        return true;
    }
}


const float& Offboard_Tag_Search::last_update_us(void) const
{
    return last_update_us_;
}

const bool& Offboard_Tag_Search::is_camera_running() const
{
    return is_camera_running_;
}

int Offboard_Tag_Search::camera_id() const
{
    return conf_.camera_id;
}

float  Offboard_Tag_Search::allowable_horizontal_tag_offset_sqr() const
{
    return conf_.allowable_horizontal_tag_offset_sqr;
}

float Offboard_Tag_Search::max_acc_drone_height_from_camera_mm() const
{
    return conf_.max_acc_drone_height_from_camera_mm;
}

float Offboard_Tag_Search::tag_search_timeout_us() const
{
    return conf_.tag_search_timeout_us;
}

void Offboard_Tag_Search::update_last_update_us()
{
    last_update_us_ = time_keeper_get_us();
}

int Offboard_Tag_Search::camera_x_resolution() const
{
    return conf_.camera_res_x;
}

int Offboard_Tag_Search::camera_y_resolution() const
{
    return conf_.camera_res_y;
}

float Offboard_Tag_Search::camera_rotation() const
{
    return conf_.camera_rotation;
}

float Offboard_Tag_Search::camera_x_fov() const
{
    return conf_.camera_fov_x;
}

float Offboard_Tag_Search::camera_y_fov() const
{
    return conf_.camera_fov_y;
}

local_position_t& Offboard_Tag_Search::tag_location()
{
    return tag_location_;
}

const int16_t& Offboard_Tag_Search::picture_count() const
{
    return picture_count_;
}

void Offboard_Tag_Search::increment_picture_count()
{
    picture_count_++;
}

Offboard_Tag_Search::land_on_tag_behavior_t Offboard_Tag_Search::land_on_tag_behavior() const
{
    return land_on_tag_behavior_;
}

void Offboard_Tag_Search::land_on_tag_behavior(land_on_tag_behavior_t behavior)
{
    land_on_tag_behavior_ = behavior;
}

Position_estimation& Offboard_Tag_Search::position_estimation()
{
    return position_estimation_;
}

const ahrs_t& Offboard_Tag_Search::ahrs() const
{
    return ahrs_;
}

Mavlink_waypoint_handler_tag& Offboard_Tag_Search::waypoint_handler()
{
    return waypoint_handler_;
}
