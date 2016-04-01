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

#include <cstdlib>

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
#include "util/maths.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief                       Receives information related to the camera results
 *
 * \param   central_data        The pointer to the central data class
 * \param   sysid               The system ID
 * \param   msg                 The received MAVLink message structure
 */
//void offboard_camera_telemetry_receive_camera_output(Offboard_Camera* camera, uint32_t sysid, mavlink_message_t* msg);
static mav_result_t offboard_camera_telemetry_receive_camera_output(Central_data* central_data, mavlink_command_long_t* packet);
//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

//void offboard_camera_telemetry_receive_camera_output(Offboard_Camera* camera, uint32_t sysid, mavlink_message_t* msg)
static mav_result_t offboard_camera_telemetry_receive_camera_output(Central_data* central_data, mavlink_command_long_t* packet)
{
    mav_result_t result;

    print_util_dbg_print("Tag loc: (");
    print_util_dbg_print_num(packet->param3,10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(packet->param4,10);
    print_util_dbg_print(")\r\n");

    Offboard_Camera camera = central_data->offboard_camera;

    // Set the x and y hold position to be equal to the tag location

    // Get drone height
    float drone_height = central_data->waypoint_handler.navigation->position_estimation->local_position.pos[2];

    // Find pixel dimensions
    /*
        tan(fov/2) = (width / 2) / drone_height
        width = 2 * drone_height * tan(fov/2)

        pixel_width = width / resolution
        pixel_width = 2 * drone_height * tan(fov/2) / resolution
    */
    float pixel_width = 2 * drone_height * tan(camera.camera_fov[0]) / (camera.camera_res[0]);
    float pixel_height = 2 * drone_height * tan(camera.camera_fov[1]) / (camera.camera_res[1]);

    // Get drone offset
    float picture_forward_offset = -packet->param4 * pixel_width; // Negative, because in vision positive is towards the bottom of the picture
    float picture_right_offset = packet->param3 * pixel_height; 
    
    // Rotate offset to align with drone
    quat_t q_rot, q_offset;
    q_rot.s = cos(camera.camera_rotation/2);
    q_rot.v[0] = 0;
    q_rot.v[1] = 0;
    q_rot.v[2] = 1*sin(camera.camera_rotation/2);
    q_offset.s = 0;
    q_offset.v[0] = picture_forward_offset;
    q_offset.v[1] = picture_right_offset;
    q_offset.v[2] = 0;
    /*
    QUAT(q_rot, cos(camera.camera_rotation/2), 0, 0, 1*sin(camera.camera_rotation/2));
    QUAT(q_offset, 0, picture_forward_offset, picture_right_offset, 0);
    */
    quat_t q_new_dir = quaternions_rotate(q_offset, q_rot);
    float drone_x_offset = q_new_dir.v[0];
    float drone_y_offset = q_new_dir.v[1];

    // Get local tag position from drone position and offset
    float tag_x_pos = central_data->waypoint_handler.navigation->position_estimation->local_position.pos[0] + drone_x_offset;
    float tag_y_pos = central_data->waypoint_handler.navigation->position_estimation->local_position.pos[1] + drone_y_offset;

    // Set hold position
    central_data->waypoint_handler.waypoint_hold_coordinates.pos[0] = tag_x_pos;
    central_data->waypoint_handler.waypoint_hold_coordinates.pos[1] = tag_y_pos;
    central_data->waypoint_handler.waypoint_hold_coordinates.pos[2] = drone_height; // This should probably be changed to a constant value to prevent drift

    result = MAV_RESULT_ACCEPTED;
    return result;

}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool offboard_camera_telemetry_init(Central_data* central_data, mavlink_message_handler_t* message_handler)
{
    bool init_success = true;

    // Add callbacks for cmd
    mavlink_message_handler_cmd_callback_t callbackcmd;
    callbackcmd.command_id = MAV_CMD_DO_CONTROL_VIDEO; // 200
    callbackcmd.sysid_filter = MAV_SYS_ID_ALL;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // WRONG 190
    callbackcmd.function = (mavlink_cmd_callback_function_t)    &offboard_camera_telemetry_receive_camera_output;
    callbackcmd.module_struct =                                 central_data;
    init_success &= mavlink_message_handler_add_cmd_callback(message_handler, &callbackcmd);
    print_util_dbg_init_msg("[PICAMERA TELEMETRY INIT]", init_success);
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
