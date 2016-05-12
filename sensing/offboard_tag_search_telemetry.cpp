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
 * \file offboard_tag_search_telemetry.c
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief This module takes care of sending and receiving messages between the
 * offboard camera and the MAVRIC autopilot for tag recognition purposes.
 *
 ******************************************************************************/


//#include "communication/state_telemetry.hpp"
#include "sensing/offboard_tag_search_telemetry.hpp"

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
static mav_result_t offboard_tag_search_telemetry_receive_camera_output(Central_data* central_data, mavlink_command_long_t* packet);
//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t offboard_tag_search_telemetry_receive_camera_output(Central_data* central_data, mavlink_command_long_t* packet)
{
    mav_result_t result;

    // Increment counter
    central_data->offboard_tag_search.increment_picture_count();

    // Set waypoint enum to tag found
    central_data->offboard_tag_search.land_on_tag_behavior(Offboard_Tag_Search::land_on_tag_behavior_t::TAG_FOUND);

    Offboard_Tag_Search camera = central_data->offboard_tag_search;

    /*
     * The incoming packet is of this format:
     * param1: camera number
     * param2: camera status
     * param3: tag horizontal location in pixels, positive is right
     * param4: tag vertical location in pixels, positive is down
     * param5: tag horizontal location in mm, divide by 1000 to make m, positive is right, -1000 for unknown
     * param6: tag vertical location in mm, divide by 1000 to make m, positive is down, -1000 for unknown
     * param7: estimated drone height in mm, divide by 1000 to make m. positive is up, -1000 for unknown as positive is up
     */

    /*
     * Set the x and y hold position to be equal to the tag location
     */

    // Get drone height, drone height tells you the pixel dimensions when projected on the ground, positive z is down
    float drone_height = 0.0f;

    // Get drone height from packet if available and reasonable
    if ((packet->param7 > 0.0f) &&                                          // Packet outputs + as up, must be greater than 0
        (packet->param7 < camera.max_acc_drone_height_from_camera_mm()))   // Don't allow too high estimations as accuracy decreases with altitude
    {
       drone_height = -packet->param7 / 1000.0f;
    }
    else // Get drone height from the local position, drone height tells you the pixel dimensions on the ground, +z is down
    {
       drone_height = central_data->position_estimation.local_position.pos[2];
    }
    
    // Get tag location in m
    float dist_to_edge_of_fov_x = -drone_height * tan(camera.camera_x_fov() / 2); // Distance from center to x edge in m
    float dist_to_edge_of_fov_y = -drone_height * tan(camera.camera_y_fov() / 2); // Distance from center to y edge in m
    float picture_forward_offset = 0.0f;
    float picture_right_offset = 0.0f;
    // Attempt to get tag distance using camera approximation
    if ((packet->param7 > 0.0f) &&                                                  // Picture gave a good estimated height --> if no good height estimation, no good distance to tag estimation
        (packet->param7 < camera.max_acc_drone_height_from_camera_mm()) &&          // Picture gave a good estimated height --> if no good height estimation, no good distance to tag estimation
        (packet->param5 > -(dist_to_edge_of_fov_x)) &&                              // Ensure that x distance to tag is within frame
        (packet->param5 <  (dist_to_edge_of_fov_x)) &&                              // Ensure that x distance to tag is within frame
        (packet->param6 > -(dist_to_edge_of_fov_y)) &&                              // Ensure that y distance to tag is within frame
        (packet->param6 <  (dist_to_edge_of_fov_y)))                                // Ensure that y distance to tag is within frame
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
        float pixel_width = 2 * (-drone_height) * tan(camera.camera_x_fov()) / (camera.camera_x_resolution()); // drone_height negated as +z is down
        float pixel_height = 2 * (-drone_height) * tan(camera.camera_y_fov()) / (camera.camera_y_resolution()); // drone_height negated as +z is down

        // Get drone offset
        // Forward corresponds to param4 as the picamera code outputs (right,down)
        picture_forward_offset = -packet->param4 * pixel_width; // Negative, because in vision positive is towards the bottom of the picture
        picture_right_offset = packet->param3 * pixel_height; 
    }
    
    
    // Rotate offset to align with drone
    quat_t q_rot, q_offset;
    q_rot.s = cos(camera.camera_rotation()/2); // Based off how camera is mounted
    q_rot.v[0] = 0;
    q_rot.v[1] = 0;
    q_rot.v[2] = 1*sin(camera.camera_rotation()/2);
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
    float tag_x_pos = central_data->position_estimation.local_position.pos[0] + drone_x_offset;
    float tag_y_pos = central_data->position_estimation.local_position.pos[1] + drone_y_offset;

    // Set hold position
    camera.tag_location().pos[0] = tag_x_pos;
    camera.tag_location().pos[1] = tag_y_pos;
    camera.tag_location().pos[2] = central_data->waypoint_handler.tag_search_altitude();

    // Update recorded time
    camera.update_last_update_us();
    
    result = MAV_RESULT_ACCEPTED;
    return result;

}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool offboard_tag_search_telemetry_init(Central_data* central_data, Mavlink_message_handler* message_handler)
{
    bool init_success = true;

    // Set tag landing state to tag not found
    central_data->offboard_tag_search.land_on_tag_behavior(Offboard_Tag_Search::land_on_tag_behavior_t::TAG_NOT_FOUND);

    // Set the tag landing altitude to be the starting altitude
    central_data->waypoint_handler.tag_search_altitude(-10.0f);

    // Init tag_location vector to 0
    central_data->offboard_tag_search.tag_location().pos[0] = 0.0f;
    central_data->offboard_tag_search.tag_location().pos[1] = 0.0f;
    central_data->offboard_tag_search.tag_location().pos[2] = 0.0f;

    // Add callbacks for cmd
    Mavlink_message_handler::cmd_callback_t callbackcmd;
    callbackcmd.command_id = MAV_CMD_DO_CONTROL_VIDEO; // 200
    callbackcmd.sysid_filter = MAV_SYS_ID_ALL;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // WRONG 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)    &offboard_tag_search_telemetry_receive_camera_output;
    callbackcmd.module_struct =                                 central_data;
    init_success &= message_handler->add_cmd_callback(&callbackcmd);
    print_util_dbg_init_msg("[PICAMERA TELEMETRY INIT]", init_success);
    return init_success;
}

void offboard_tag_search_goal_location_telemetry_send(const Central_data* central_data, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_debug_vect_pack(mavlink_stream->sysid(),
                                mavlink_stream->compid(),
                                msg,
                                "Tag_Search_Goal_Location",
                                time_keeper_get_us(),
                                central_data->offboard_tag_search.tag_location().pos[0] - central_data->position_estimation.local_position.pos[0],
                                central_data->offboard_tag_search.tag_location().pos[1] - central_data->position_estimation.local_position.pos[1],
                                //central_data->offboard_tag_search.tag_location().pos[2] - central_data->waypoint_handler.navigation->position_estimation->local_position.pos[2]);
                                central_data->offboard_tag_search.picture_count());
}

void offboard_tag_search_telemetry_send_start_stop(const Offboard_Tag_Search* camera, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    int is_camera_running = -1;
    // Switch boolean to int as mavlink sends ints/flaots
    switch(camera->is_camera_running())
    {
        case true:
            is_camera_running = 1;
            break;
        case false:
            is_camera_running = 0;
            break;
    }
    mavlink_msg_command_long_pack(  mavlink_stream->sysid(),      // system_id
                                    mavlink_stream->compid(),     // component_id
                                    msg,                        // mavlink_msg
                                    0,      // target_system
                                    0,      // target_component
                                    MAV_CMD_DO_CONTROL_VIDEO,   // command
                                    0,     // confirmation
                                    camera->camera_id(),    // param1
                                    camera->is_camera_running(),          // param2
                                    0,                          // param3
                                    0,                          // param4
                                    0,                          // param5
                                    0,                          // param6
                                    0);                         // param7
}
