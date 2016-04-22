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
 * \file gps_telemetry.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief GPS telemetry
 *
 ******************************************************************************/


#include "drivers/gps_telemetry.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/maths.h"
#include "util/constants.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Launch the configuration of the GPS
 *
 * \param   gps                     The pointer to the gps structure
 * \param   packet                  The pointer to the decoded MAVLink message long
 *
 * \return  The MAV_RESULT of the command
 */
static mav_result_t gps_start_configuration(Gps* gps, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t gps_start_configuration(Gps* gps, mavlink_command_long_t* packet)
{
    mav_result_t result = MAV_RESULT_TEMPORARILY_REJECTED;

    if (packet->param1 == 1)
    {
        gps->configure();

        result = MAV_RESULT_ACCEPTED;
    }

    return result;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool gps_telemetry_init(Gps* gps, Mavlink_message_handler* message_handler)
{
    bool init_success = true;

    // Add callbacks for fat_fs_mounting commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_PREFLIGHT_UAVCAN; // 243
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)            &gps_start_configuration;
    callbackcmd.module_struct  = (Mavlink_message_handler::handling_module_struct_t) gps;
    init_success &= message_handler->add_cmd_callback(&callbackcmd);

    return init_success;
}

void gps_telemetry_send_raw(const Gps* gps, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    global_position_t global_position = gps->position_gf();
    float ground_speed = maths_fast_sqrt(gps->velocity_lf()[X] * gps->velocity_lf()[X]
                                         + gps->velocity_lf()[Y] * gps->velocity_lf()[Y]
                                         + gps->velocity_lf()[Z] * gps->velocity_lf()[Z]);
    mavlink_msg_gps_raw_int_pack(mavlink_stream->sysid(),
                                 mavlink_stream->compid(),
                                 msg,
                                 gps->last_update_us(),
                                 gps->fix(),
                                 global_position.latitude  * 10000000.0f,
                                 global_position.longitude * 10000000.0f,
                                 global_position.altitude  * 1000.0f,
                                 gps->horizontal_position_accuracy() * 100.0f,
                                 gps->vertical_position_accuracy() * 100.0f,
                                 ground_speed * 100.0f,
                                 gps->heading() * 100.0f,
                                 gps->num_sats());
}