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
 * \file offboard_camera.hpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief Offboard camera control
 *
 ******************************************************************************/


#ifndef OFFBOARD_CAMERA_HPP_
#define OFFBOARD_CAMERA_HPP_

#include "runtime/scheduler.h"

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
    int camera_res[2];                                  ///< The resolution of the offboard camera
    float camera_rotation;                              ///< The rotation of the offboard camera w.r.t. the front of the drone, CCW from drone to camera is +
    float camera_fov[2];                                ///< The field of view of the camera in radians
} offboard_camera_conf_t;


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
class Offboard_Camera
{
public:
    /**
     * \brief Constructor
     *
     * \param config    The offboard camera configuration
     */
    Offboard_Camera(offboard_camera_conf_t config);


    /**
     * \brief   Main update
     *
     * \return  Success
     */
    bool update(const scheduler_t* scheduler);


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

    bool get_is_camera_running();
    int get_camera_id();

    int camera_id_;                     ///< ID number of camera
    bool is_camera_running_;            ///< States whether the camera should be running
    float last_update_us_;              ///< Last update time in microseconds
    float picture_count;                ///< The count of the pictures received

    float get_allowable_horizontal_tag_offset_sqr();
    float get_max_acc_drone_height_from_camera_mm();
    float get_tag_search_timeout_us();
    int get_camera_x_resolution();
    int get_camera_y_resolution();
    float get_camera_rotation();
    float get_camera_x_fov();
    float get_camera_y_fov();
    
private:
    Offboard_Camera();

    float allowable_horizontal_tag_offset_sqr_;     ///< The maximum allowable horizontal distance between the tag and the drone before the drone will start to descend
    float max_acc_drone_height_from_camera_mm_;     ///< The maximum acceptable altitude where the code will trust the cameras height estimation
    float tag_search_timeout_us_;                   ///< The allowable time to try to search for the tag
    int camera_res_[2];                              ///< The resolution of the offboard camera
    float camera_rotation_;                          ///< The rotation of the offboard camera w.r.t. the front of the drone, CCW from drone to camera is +
    float camera_fov_[2];                            ///< The field of view of the camera in radians
};


#endif /* OFFBOARD_CAMERA_HPP_ */
