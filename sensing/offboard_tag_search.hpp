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

extern "C"
{

}


/**
 * \brief   Configuration structure
 */
typedef struct
{
    int camera_id;                                      ///< The camera id to send to the offboard camera computer
    bool initial_camera_state;                          ///< The starting on/off state of the camera
    float allowable_horizontal_tag_offset_sqr;          ///< The square distance from the drone to the center of the tag that is acceptable
    float max_acc_drone_height_from_camera_mm;          ///< The maximum acceptable drone height where the code will trust the cameras height estimation
    int camera_res_x;                                   ///< The x resolution of the offboard camera
    int camera_res_y;                                   ///< The y resolution of the offboard camera
    float camera_rotation;                              ///< The rotation of the offboard camera w.r.t. the front of the drone, CCW from drone to camera is +
    float camera_fov_x;                                 ///< The horizontal field of view of the camera in radians
    float camera_fov_y;                                 ///< The vertical field of view of the camera in radians
    float tag_search_timeout_us;                        ///< The time allowed before the tag search will time out and descend
    float max_acc_time_since_last_detection_us;         ///< The maximum acceptable time since the last detection in us for healthy data
} offboard_tag_search_conf_t;


class Central_data;

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
     * \brief Constructor
     *
     * \param config    The offboard camera configuration
     */
    Offboard_Tag_Search(offboard_tag_search_conf_t config);


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

    const bool& is_camera_running() const;
    int camera_id() const;
    const float& picture_count() const;
    float allowable_horizontal_tag_offset_sqr() const;
    float max_acc_drone_height_from_camera_mm() const;
    const float& tag_search_timeout_us() const;
    int camera_x_resolution() const;
    int camera_y_resolution() const;
    float camera_rotation() const;
    float camera_x_fov() const;
    float camera_y_fov() const;
    local_position_t& tag_location();

protected:
    Offboard_Tag_Search();

    const int camera_id_;                               ///< ID number of camera
    bool is_camera_running_;                            ///< States whether the camera should be running
    float last_update_us_;                              ///< Last update time in microseconds
    float picture_count_;                               ///< The count of the pictures received
    local_position_t tag_location_;                     ///< The location of the tag in the local frame
    const float allowable_horizontal_tag_offset_sqr_;   ///< The maximum allowable horizontal distance between the tag and the drone before the drone will start to descend
    const float max_acc_drone_height_from_camera_mm_;   ///< The maximum acceptable altitude where the code will trust the cameras height estimation
    const float tag_search_timeout_us_;                 ///< The allowable time to try to search for the tag
    const int camera_res_[2];                           ///< The resolution of the offboard camera
    const float camera_rotation_;                       ///< The rotation of the offboard camera w.r.t. the front of the drone, CCW from drone to camera is +
    const float camera_fov_[2];                         ///< The field of view of the camera in radians
    const float max_acc_time_since_last_detection_us_;  ///< The maximum acceptible time since last detection in us for a healthy read
};


#endif /* OFFBOARD_TAG_SEARCH_HPP_ */
