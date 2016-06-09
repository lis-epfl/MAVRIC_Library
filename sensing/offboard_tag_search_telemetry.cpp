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
#include "sensing/offboard_tag_search.hpp"

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
 * \param   offboard_tag_search The pointer to the offboard tag search class
 * \param   sysid               The system ID
 * \param   msg                 The received MAVLink message structure
 */
static mav_result_t offboard_tag_search_telemetry_receive_camera_output(Offboard_Tag_Search& offboard_tag_search, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t offboard_tag_search_telemetry_receive_camera_output(Offboard_Tag_Search& offboard_tag_search, mavlink_command_long_t* packet)
{
    mav_result_t result;

    // Increment counter
    offboard_tag_search.increment_picture_count();

    // Set waypoint enum to tag found
    offboard_tag_search.land_on_tag_behavior(Offboard_Tag_Search::land_on_tag_behavior_t::TAG_FOUND);

    /*
     * The incoming packet is of this format:
     * param1: camera number
     * param2: thread index
     * param3: tag horizontal location in pixels, positive is right, -1000 for unknown
     * param4: tag vertical location in pixels, positive is down, -1000 for unknown
     * param5: tag horizontal location in mm, divide by 1000 to make m, positive is right, -1000 for unknown
     * param6: tag vertical location in mm, divide by 1000 to make m, positive is down, -1000 for unknown
     * param7: estimated drone height in mm, divide by 1000 to make m. positive is up, -1000 for unknown as positive is up
     */

    /*
     * Set the x and y hold position to be equal to the tag location
     */

    // Get drone height, drone height tells you the pixel dimensions when projected on the ground, positive z is down
    float drone_height = 0.0f;
    int thread_index = packet->param2;

    // If a tag had actually been detected
    if (packet->param3 < -999.0f &&     // Use range due to float errors
        packet->param3 > -1001.0f &&    // Use range due to float errors
        packet->param4 < -999.0f &&     // Use range due to float errors
        packet->param4 > -1001.0f)      // Use range due to float errors
    {
        // Get drone height from packet if available and reasonable
        if ((packet->param7 > 0.0f) &&                                          // Packet outputs + as up, must be greater than 0
            (packet->param7 < offboard_tag_search.max_acc_drone_height_from_camera_mm()))   // Don't allow too high estimations as accuracy decreases with altitude
        {
           drone_height = -packet->param7 / 1000.0f;
        }
        else // Get drone height from the local position, drone height tells you the pixel dimensions on the ground, +z is down
        {
           drone_height = offboard_tag_search.position_at_photo(thread_index).pos[2];
        }

        // Get tag location in m
        float dist_to_edge_of_fov_x = -drone_height * tan(offboard_tag_search.camera_x_fov() / 2); // Distance from center to x edge in m
        float dist_to_edge_of_fov_y = -drone_height * tan(offboard_tag_search.camera_y_fov() / 2); // Distance from center to y edge in m
        float picture_forward_offset = 0.0f;
        float picture_right_offset = 0.0f;
        // Attempt to get tag distance using camera approximation
        if ((packet->param7 > 0.0f) &&                                                          // Picture gave a good estimated height --> if no good height estimation, no good distance to tag estimation
            (packet->param7 < offboard_tag_search.max_acc_drone_height_from_camera_mm()) &&     // Picture gave a good estimated height --> if no good height estimation, no good distance to tag estimation
            (packet->param5 > -(dist_to_edge_of_fov_x)) &&                                      // Ensure that x distance to tag is within frame
            (packet->param5 <  (dist_to_edge_of_fov_x)) &&                                      // Ensure that x distance to tag is within frame
            (packet->param6 > -(dist_to_edge_of_fov_y)) &&                                      // Ensure that y distance to tag is within frame
            (packet->param6 <  (dist_to_edge_of_fov_y)))                                        // Ensure that y distance to tag is within frame
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
            float pixel_width = 2 * (-drone_height) * tan(offboard_tag_search.camera_x_fov() / 2) / (offboard_tag_search.camera_x_resolution()); // drone_height negated as +z is down
            float pixel_height = 2 * (-drone_height) * tan(offboard_tag_search.camera_y_fov() / 2) / (offboard_tag_search.camera_y_resolution()); // drone_height negated as +z is down

            // Get drone offset
            // Forward corresponds to param4 as the picamera code outputs (right,down)
            picture_forward_offset = -packet->param4 * pixel_width; // Negative, because in vision positive is towards the bottom of the picture
            picture_right_offset = packet->param3 * pixel_height;
        }


        // Rotate offset to align with drone
        quat_t q_rot;
        float v_offset[3], v_new_dir[3];
        q_rot.s = cos(offboard_tag_search.camera_rotation()/2); // Based off how camera is mounted
        q_rot.v[0] = 0.0f;
        q_rot.v[1] = 0.0f;
        q_rot.v[2] = -1.0f*sin(offboard_tag_search.camera_rotation()/2); // Negative to rotate CW
        v_offset[0] = picture_forward_offset;
        v_offset[1] = picture_right_offset;
        v_offset[2] = -drone_height; // Negative as the positive direction is down
        v_new_dir[0] = 0.0f;
        v_new_dir[1] = 0.0f;
        v_new_dir[2] = 0.0f;
        quaternions_rotate_vector(q_rot, v_offset, v_new_dir);

        // Convert to local coordinates due to yaw, pitch, roll
        quat_t q_ahrs = offboard_tag_search.ahrs_at_photo(thread_index).qe;
        float v_new_local_dir[3];
        quaternions_rotate_vector(q_ahrs, v_new_dir, v_new_local_dir);

        // Determine how far the drone is from the tag in north and east coordinates
        float drone_x_offset = v_new_local_dir[0];
        float drone_y_offset = v_new_local_dir[1];

        // Get local tag position from drone position and offset
        float tag_x_pos = offboard_tag_search.position_at_photo(thread_index).pos[0] + drone_x_offset;
        float tag_y_pos = offboard_tag_search.position_at_photo(thread_index).pos[1] + drone_y_offset;

        // Set hold position
        offboard_tag_search.tag_location().pos[0] = tag_x_pos;
        offboard_tag_search.tag_location().pos[1] = tag_y_pos;
        offboard_tag_search.tag_location().pos[2] = offboard_tag_search.waypoint_handler().tag_search_altitude();

        // Update recorded time
        offboard_tag_search.update_last_update_us();
    }

    // Send message to take new photo
    mavlink_message_t msg;
    offboard_tag_search_telemetry_send_take_new_photo(thread_index, &offboard_tag_search, &(offboard_tag_search.mavlink_communication().mavlink_stream()), &msg);
    offboard_tag_search.mavlink_communication().mavlink_stream().send(&msg);

    result = MAV_RESULT_ACCEPTED;
    return result;

}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool offboard_tag_search_telemetry_init(Offboard_Tag_Search* offboard_tag_search, Mavlink_message_handler* message_handler)
{
    bool init_success = true;

    // Set tag landing state to tag not found
    offboard_tag_search->land_on_tag_behavior(Offboard_Tag_Search::land_on_tag_behavior_t::TAG_NOT_FOUND);

    // Set the tag landing altitude to be the starting altitude
    offboard_tag_search->waypoint_handler().tag_search_altitude(-10.0f);

    // Init tag_location vector to 0
    offboard_tag_search->tag_location().pos[0] = 0.0f;
    offboard_tag_search->tag_location().pos[1] = 0.0f;
    offboard_tag_search->tag_location().pos[2] = 0.0f;

    // Add callbacks for cmd
    Mavlink_message_handler::cmd_callback_t callbackcmd;
    callbackcmd.command_id = MAV_CMD_DO_CONTROL_VIDEO; // 200
    callbackcmd.sysid_filter = MAV_SYS_ID_ALL;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // WRONG 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)    &offboard_tag_search_telemetry_receive_camera_output;
    callbackcmd.module_struct =                                 offboard_tag_search;
    init_success &= message_handler->add_cmd_callback(&callbackcmd);
    print_util_dbg_init_msg("[PICAMERA TELEMETRY INIT]", init_success);
    return init_success;
}

void offboard_tag_search_goal_location_telemetry_send(Offboard_Tag_Search* offboard_tag_search, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_debug_vect_pack(mavlink_stream->sysid(),
                                mavlink_stream->compid(),
                                msg,
                                "Tag_Search_Goal_Location",
                                time_keeper_get_us(),
                                offboard_tag_search->tag_location().pos[0] - offboard_tag_search->position_estimation().local_position.pos[0],
                                offboard_tag_search->tag_location().pos[1] - offboard_tag_search->position_estimation().local_position.pos[1],
                                //offboard_tag_search->tag_location().pos[2] - offboard_tag_search->position_estimation()->local_position.pos[2]);
                                offboard_tag_search->picture_count());
}

void offboard_tag_search_telemetry_send_take_new_photo(int index, Offboard_Tag_Search* camera, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    // Send message
    mavlink_msg_command_long_pack(  mavlink_stream->sysid(),        // system_id
                                    mavlink_stream->compid(),       // component_id
                                    msg,                            // mavlink_msg
                                    0,                              // target_system
                                    0,                              // target_component
                                    MAV_CMD_DO_CONTROL_VIDEO,       // command
                                    0,                              // confirmation
                                    camera->camera_id(),            // param1
                                    camera->is_camera_running(),    // param2
                                    index,                          // param3
                                    0,                              // param4
                                    0,                              // param5
                                    0,                              // param6
                                    0);                             // param7

    // Record position
    camera->set_position_at_photo(index);
}

void offboard_tag_search_telemetry_send_start_stop(Offboard_Tag_Search* camera, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    // Only send message if the change is new
    if (camera->has_camera_state_changed())
    {
        camera->camera_state_has_changed(false);

        int is_camera_running = -1;
        // Switch boolean to int as mavlink sends ints/flaots
        switch(camera->is_camera_running())
        {
            case true: // Send 1 message per thread to take photo and start
                is_camera_running = 1;
                // Send thread message
                for (int i = 0; i < camera->offboard_threads(); i++)
                {
                    offboard_tag_search_telemetry_send_take_new_photo(i, camera, mavlink_stream, msg);
                }
                break;
            case false: // Send message to stop
                is_camera_running = 0;
                mavlink_msg_command_long_pack(  mavlink_stream->sysid(),        // system_id
                                                mavlink_stream->compid(),       // component_id
                                                msg,                            // mavlink_msg
                                                0,                              // target_system
                                                0,                              // target_component
                                                MAV_CMD_DO_CONTROL_VIDEO,       // command
                                                0,                              // confirmation
                                                camera->camera_id(),            // param1
                                                camera->is_camera_running(),    // param2
                                                0,                              // param3
                                                0,                              // param4
                                                0,                              // param5
                                                0,                              // param6
                                                0);                             // param7
                break;
        }
    }
}
