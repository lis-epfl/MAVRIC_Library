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
 * \file offboard_camera_telemetry.c
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief This module takes care of sending and receiving messages between the
 * offboard camera and the MAVRIC autopilot for tag recognition purposes.
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

#define MAX_ACC_DRONE_HEIGHT_FROM_CAMERA_MM 15000
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
static mav_result_t offboard_camera_telemetry_receive_camera_output(Central_data* central_data, mavlink_command_long_t* packet);
//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t offboard_camera_telemetry_receive_camera_output(Central_data* central_data, mavlink_command_long_t* packet)
{
    mav_result_t result;

    // Increment counter
    central_data->offboard_camera.picture_count++;

    // Only change stuff if we are search for a tag
    if (central_data->waypoint_handler.navigation->internal_state == NAV_LAND_ON_TAG)
    {
        // Set waypoint enum to tag found
        central_data->waypoint_handler.navigation->land_on_tag_behavior = TAG_FOUND;

        Offboard_Camera camera = central_data->offboard_camera;

        /*
         * The incoming packet is of this format:
         * param1: camera number
         * param2: camera status
         * param3: tag horizontal location in pixels, positive is right
         * param4: tag vertical location in pixels, positive is down
         * param5: tag horizontal location in mm, divide by 1000 to make m, positive is right, -1000000000 for unknown
         * param6: tag vertical location in mm, divide by 1000 to make m, positive is down, -1000000000 for unknown
         * param7: estimated drone height in mm, divide by 1000 to make m. positive is up, -1000000000 for unknown
         */

        /*
         * Set the x and y hold position to be equal to the tag location
         */

        // Get drone height, drone height tells you the pixel dimensions on the ground, +z is down
        float drone_height = 0.0f;
        if ((packet->param7 > -900000000) && (packet->param7 < MAX_ACC_DRONE_HEIGHT_FROM_CAMERA_MM)) // Get drone height from the packet if available
            // Restrict to drone heights that are within a set range
        {
           drone_height = -packet->param7 / 1000.0f;
        }
        else // Get drone height from the local position, drone height tells you the pixel dimensions on the ground, +z is down
        {
           drone_height = central_data->waypoint_handler.navigation->position_estimation->local_position.pos[2];
        }
        
        // Get tag location in m
        float picture_forward_offset = 0.0f;
        float picture_right_offset = 0.0f;
        if ((packet->param5 > -900000000) && (packet->param6 > -900000000)) // Get tag location from packet in m if available
        {
            // Forward corresponds to param6 as the picamera code outputs (right,down)
            picture_forward_offset = -packet->param6 / 1000.0f; // Negative, because in vision positive is towards the bottom of the picture
            picture_right_offset = packet->param5 / 1000.0f;
        }
        else // Use pixels
        {
            // Find pixel dimensions
            /*
                tan(fov/2) = (width / 2) / drone_height
                width = 2 * drone_height * tan(fov/2)

                pixel_width = width / resolution
                pixel_width = 2 * drone_height * tan(fov/2) / resolution
            */
            float pixel_width = 2 * (-drone_height) * tan(camera.camera_fov[0]) / (camera.camera_res[0]); // drone_height negated as +z is down
            float pixel_height = 2 * (-drone_height) * tan(camera.camera_fov[1]) / (camera.camera_res[1]); // drone_height negated as +z is down

            // Get drone offset
            // Forward corresponds to param4 as the picamera code outputs (right,down)
            picture_forward_offset = -packet->param4 * pixel_width; // Negative, because in vision positive is towards the bottom of the picture
            picture_right_offset = packet->param3 * pixel_height; 
        }
        
        
        // Rotate offset to align with drone
        quat_t q_rot, q_offset;
        q_rot.s = cos(camera.camera_rotation/2); // Based off how camera is mounted
        q_rot.v[0] = 0;
        q_rot.v[1] = 0;
        q_rot.v[2] = 1*sin(camera.camera_rotation/2);
        q_offset.s = 0;
        q_offset.v[0] = picture_forward_offset;
        q_offset.v[1] = picture_right_offset;
        q_offset.v[2] = 0;
        quat_t q_new_dir = quaternions_rotate(q_offset, q_rot);

        // Convert to local coordinates due to yaw not facing north
        float yaw = coord_conventions_get_yaw(central_data->ahrs.qe);
        quat_t q_yaw_rot;
        q_yaw_rot.s = cos(yaw/2);
        q_yaw_rot.v[0] = 0;
        q_yaw_rot.v[1] = 0;
        q_yaw_rot.v[2] = -1*sin(yaw/2); // Negative to rotate CCW
        quat_t q_new_local_dir = quaternions_rotate(q_new_dir, q_yaw_rot);

        // Determine how far the drone is from the tag in north and east coordinates
        float drone_x_offset = q_new_local_dir.v[0];
        float drone_y_offset = q_new_local_dir.v[1];

        // Get local tag position from drone position and offset
        float tag_x_pos = central_data->waypoint_handler.navigation->position_estimation->local_position.pos[0] + drone_x_offset;
        float tag_y_pos = central_data->waypoint_handler.navigation->position_estimation->local_position.pos[1] + drone_y_offset;

        // Set hold position
        central_data->waypoint_handler.waypoint_hold_coordinates.pos[0] = tag_x_pos;
        central_data->waypoint_handler.waypoint_hold_coordinates.pos[1] = tag_y_pos;
        central_data->waypoint_handler.waypoint_hold_coordinates.pos[2] = central_data->waypoint_handler.navigation->tag_search_altitude;

        // Update recorded time
        camera.update_last_update_us();
    }
    
    result = MAV_RESULT_ACCEPTED;
    return result;

}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool offboard_camera_telemetry_init(Central_data* central_data, mavlink_message_handler_t* message_handler)
{
    bool init_success = true;

    // Set tag landing state to tag not found
    central_data->waypoint_handler.navigation->land_on_tag_behavior = TAG_NOT_FOUND;

    // Set the tag landing altitude to be the starting altitude
    central_data->waypoint_handler.navigation->tag_search_altitude = -10.0f;

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

void offboard_camera_goal_location_telemetry_send(const Central_data* central_data, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_debug_vect_pack(mavlink_stream->sysid,
                                mavlink_stream->compid,
                                msg,
                                "Tag_Search_Goal_Location",
                                time_keeper_get_us(),
                                central_data->waypoint_handler.navigation->goal.pos[0] - central_data->waypoint_handler.navigation->position_estimation->local_position.pos[0],
                                central_data->waypoint_handler.navigation->goal.pos[1] - central_data->waypoint_handler.navigation->position_estimation->local_position.pos[1],
                                //central_data->waypoint_handler.navigation->goal.pos[2] - central_data->waypoint_handler.navigation->position_estimation->local_position.pos[2]);
                                central_data->offboard_camera.picture_count);
}

void offboard_camera_telemetry_send_start_stop(const Offboard_Camera* camera, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    int is_camera_running = -1;
    // Switch boolean to int as mavlink sends ints/flaots
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
