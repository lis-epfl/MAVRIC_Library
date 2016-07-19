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
 * \file offboard_tag_search.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief Offboard camera control
 *
 ******************************************************************************/


#ifndef OFFBOARD_TAG_SEARCH_HPP_
#define OFFBOARD_TAG_SEARCH_HPP_

#include "runtime/scheduler.hpp"
#include "util/coord_conventions.h"
#include "sensing/ahrs.h"
#include "sensing/position_estimation.hpp"
#include "sensing/offboard_tag_search_telemetry.hpp"
#include "communication/mavlink_communication.hpp"
#include "communication/mavlink_stream.hpp"
#include "communication/mavlink_message_handler.hpp"

class Mavlink_waypoint_handler_tag;

extern "C"
{

}


/**
 * \brief   Configuration structure
 */
typedef struct
{
    int camera_id;                                      ///< The camera id to send to the offboard camera computer
    float allowable_horizontal_tag_offset_sqr;          ///< The square distance from the drone to the center of the tag that is acceptable
    float descent_to_gnd_altitude;                      ///< The altitude that the landing algorithm should switch from descent to small altitude to descent to ground
    float max_acc_drone_height_from_camera_mm;          ///< The maximum acceptable drone height where the code will trust the cameras height estimation
    int camera_res_x;                                   ///< The x resolution of the offboard camera
    int camera_res_y;                                   ///< The y resolution of the offboard camera
    float camera_rotation;                              ///< The rotation of the offboard camera w.r.t. the front of the drone, CCW from drone to camera is +
    float camera_fov_x;                                 ///< The horizontal field of view of the camera in radians
    float camera_fov_y;                                 ///< The vertical field of view of the camera in radians
    float tag_search_timeout_us;                        ///< The time allowed before the tag search will time out and descend
    float max_acc_time_since_last_detection_us;         ///< The maximum acceptable time since the last detection in us for healthy data
} offboard_tag_search_conf_t;


static inline offboard_tag_search_conf_t offboard_tag_search_conf_default() {
    offboard_tag_search_conf_t conf;

    conf.camera_id                               = 1;
    conf.allowable_horizontal_tag_offset_sqr     = 0.25f;
    conf.descent_to_gnd_altitude                 = -0.5f;
    conf.max_acc_drone_height_from_camera_mm     = 15000.0f;
    conf.tag_search_timeout_us                   = 60000000.0f;                 // 1 minute
    conf.camera_res_x                            = 1280;                        // Suitable for picamera
    conf.camera_res_y                            = 960;                         // Suitable for picamera
    conf.camera_rotation                         = maths_deg_to_rad(90.0f);
    conf.camera_fov_x                            = maths_deg_to_rad(53.50f);    // Suitable for picamera
    conf.camera_fov_y                            = maths_deg_to_rad(41.41f);    // Suitable for picamera
    conf.max_acc_time_since_last_detection_us    = 10000000.0f;                 // 10 seconds

    return conf;
}

/**
 * \brief   Offboard Camera Master Controller
 *
 * \details This module controls whether the offboard camera should be running or not.
 *
 *          This module does not actually contian information related to the sensing of
 *          of the offboard camera, just related to whether the camera should be runnning
 *          or not.
 */
class Offboard_Tag_Search
{
public:
    /**
     * \brief   The auto-landing enum
     */
    enum land_on_tag_behavior_t
    {
        TAG_NOT_FOUND,                                      ///< Tag has not yet been found, search for tag
        TAG_FOUND                                           ///< Tag has been found, go to location
    };

    /**
     * \brief Constructor
     *
     * \param position_estimation   Central_data's position_estimation
     * \param ahrs                  Central_data's ahrs
     * \param waypoint_handler      Central_data's waypoint_handler
     * \param mavlink_communication Central_data's mavlink_communication
     * \param config                The offboard camera configuration
     */
    Offboard_Tag_Search(const Position_estimation& position_estimation, const ahrs_t& ahrs, Mavlink_waypoint_handler_tag& waypoint_handler, Mavlink_communication& mavlink_communication, offboard_tag_search_conf_t config = offboard_tag_search_conf_default());


    /**
     * \brief   Main update
     *
     * \param   scheduler       The scheduler communicates with the offboard computer
     * \param   camera_state    The new state of the camera
     * \return  Success
     */
    bool update(const Scheduler* scheduler, bool camera_state);


    /**
     * \brief   Get last update time in microseconds
     *
     * \return  Value
     */
    const float& last_update_us(void) const;

    /**
     * \brief   Updates the time that the tag was last updated
     */
    void update_last_update_us();

    /**
     * \brief   Increments the picture count by one
     */
    void increment_picture_count();

    /**
     * \brief   Checks if the current tag reading is healthy.
     *
     * An unhealthy reading could be due to some of the following:
     *      - Too significant time has passed since the last reading
     *
     * \return  Boolean stating if the data is healthy
     */
    bool is_healthy() const;

    // Getters and setters
    const ahrs_t& ahrs() const;
    const Position_estimation& position_estimation() const;
    Mavlink_communication& mavlink_communication();
    Mavlink_waypoint_handler_tag& waypoint_handler();

    int camera_id() const;
    int camera_x_resolution() const;
    int camera_y_resolution() const;
    float camera_rotation() const;
    float camera_x_fov() const;
    float camera_y_fov() const;

    float allowable_horizontal_tag_offset_sqr() const;
    float descent_to_gnd_altitude() const;
    float max_acc_drone_height_from_camera_mm() const;
    float tag_search_timeout_us() const;

    const bool& is_camera_running() const;
    void set_is_camera_running(bool is_camera_running);
    land_on_tag_behavior_t land_on_tag_behavior() const;
    void land_on_tag_behavior(land_on_tag_behavior_t land_on_tag_behavior);

    const int16_t& picture_count() const;
    local_position_t& tag_location();

    const int offboard_threads() const;
    const local_position_t position_at_photo(int index) const;
    const ahrs_t ahrs_at_photo(int index) const;
    bool has_photo_been_taken(int index) const;
    void set_has_photo_been_taken(int index, bool state);
    void set_position_at_photo(int index);
    
protected:
    Offboard_Tag_Search();

    static const int offboard_threads_ = 1;                 ///< The number of threads that can be running on the offboard computer
    local_position_t position_at_photo_[offboard_threads_]; ///< The local position when the photo was taken
    ahrs_t ahrs_at_photo_[offboard_threads_];               ///< The ahrs vector when the photo was taken

    offboard_tag_search_conf_t conf_;                       ///< The configuration of the offboard tag search object
    bool is_camera_running_;                                ///< States whether the camera should be running
    bool has_photo_been_taken_[offboard_threads_];           ///< Boolean array stating if the thread has taken a photo
    float last_update_us_;                                  ///< Last update time in microseconds
    int16_t picture_count_;                                 ///< The count of the pictures received
    local_position_t tag_location_;                         ///< The location of the tag in the local frame
    land_on_tag_behavior_t land_on_tag_behavior_;           ///< The land on tag behavior enum

    const Position_estimation& position_estimation_;
    const ahrs_t& ahrs_;
    Mavlink_communication& mavlink_communication_;
    Mavlink_waypoint_handler_tag& waypoint_handler_;

};


#endif /* OFFBOARD_TAG_SEARCH_HPP_ */
