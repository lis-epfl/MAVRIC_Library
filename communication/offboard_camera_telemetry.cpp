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
 * \file state_telemetry.c
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of sending periodic telemetric messages for
 * the state
 *
 ******************************************************************************/


//#include "communication/state_telemetry.hpp"
#include "communication/offboard_camera_telemetry.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief                       Sets the information received from the offboard camera
 *
 * \param   camera              The pointer to the offboard camera class
 * \param   sysid               The system ID
 * \param   msg                 The received MAVLink message structure
 */
void offboard_camera_telemetry_start_stop(Offboard_Camera* camera, uint32_t sysid, mavlink_message_t* msg);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void offboard_camera_telemetry_start_stop(Offboard_Camera* camera, uint32_t sysid, mavlink_message_t* msg)
{
    
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool offboard_camera_telemetry_init(Offboard_Camera* camera, mavlink_message_handler_t* message_handler)
{
    bool init_success = true;
/*
    // Add callbacks for onboard parameters requests
    mavlink_message_handler_msg_callback_t callback;

    callback.message_id         = MAV_CMD_DO_CONTROL_VIDEO; // 1
    callback.sysid_filter       = MAVLINK_BASE_STATION_ID;
    callback.compid_filter      = MAV_COMP_ID_ALL;
    //callback.compid_target      = MAV_COMP_ID_ALL; // 0
    callback.function           = (mavlink_msg_callback_function_t) &offboard_camera_telemetry_start_stop;
    callback.module_struct      = (handling_module_struct_t)        camera;
    init_success &= mavlink_message_handler_add_cmd_callback(message_handler, &callback);
*/
    return init_success;
}

void offboard_camera_telemetry_send_start_stop(const Offboard_Camera* camera, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    int is_camera_running = -1;
    switch(camera->is_camera_running_)
    {
        case true:
            is_camera_running = 1;
            break;
        case false:
            is_camera_running = 0;
            break;
    }
    mavlink_msg_command_long_pack(  mavlink_stream->sysid,      // system_id
                                    mavlink_stream->compid,     // component_id
                                    msg,                        // mavlink_msg
                                    0,      // target_system
                                    0,      // target_component
                                    MAV_CMD_DO_CONTROL_VIDEO,   // command
                                    0,     // confirmation
                                    camera->camera_id_,    // param1
                                    is_camera_running,          // param2
                                    0,                          // param3
                                    0,                          // param4
                                    0,                          // param5
                                    0,                          // param6
                                    0);                         // param7
}
