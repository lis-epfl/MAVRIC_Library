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
 * \file offboard_camera.c
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief Offboard camera control
 *
 ******************************************************************************/


#include "sensing/offboard_camera.hpp"
#include "sample_projects/LEQuad/tasks.hpp"
#include "sample_projects/LEQuad/central_data.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
#include "util/constants.h"
#include "util/maths.h"
}



Offboard_Camera::Offboard_Camera(int camera_id, bool is_camera_running):
    camera_id_(camera_id),
    is_camera_running_(is_camera_running),
    last_update_us_(time_keeper_get_us())
{
    // Set picture count to 0
    picture_count = 0.0f;

    // Set camera resolution
    camera_res[0] = 1280;
    camera_res[1] = 960;

    // Set camera rotation
    camera_rotation = 90.0f * PI / 180;

    // Set camera field of view
    camera_fov[0] = 53.50f * PI / 180;
    camera_fov[1] = 41.41f * PI / 180;
}


bool Offboard_Camera::update(const Scheduler* scheduler)
{
    bool success = true;
    

    // TEMPORARY...
    // Switch camera on and off
    is_camera_running_ = true;
    // ...TEMPORARY

    // Update timing
    uint32_t t      = time_keeper_get_us();
    //dt_s_           = (float)(t - last_update_us_) / 1000000.0f;
    last_update_us_ = t;

    // Send the message now
    Scheduler_task* camera_send_message_task = scheduler->get_task_by_id(MAVLINK_MSG_ID_COMMAND_LONG);
    camera_send_message_task->run_now();

    return success;
}


const float& Offboard_Camera::last_update_us(void) const
{
    return last_update_us_;
}

bool Offboard_Camera::get_is_camera_running()
{
    return is_camera_running_;
}

int Offboard_Camera::get_camera_id()
{
    return camera_id_;
}

void Offboard_Camera::update_last_update_us()
{
    last_update_us_ = time_keeper_get_us();
}