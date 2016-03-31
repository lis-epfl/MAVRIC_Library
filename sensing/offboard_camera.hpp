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
     * \param camera_id             The id number of the camera
     * \param is_camera_running     States if this camera should be running
     */
    Offboard_Camera(int camera_id, bool is_camera_running);


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

    bool get_is_camera_running();
    int get_camera_id();

    int camera_id_;
    bool is_camera_running_;            ///< States whether the camera should be running
    float last_update_us_;              ///< Last update time in microseconds
    
private:
    Offboard_Camera();
};


#endif /* OFFBOARD_CAMERA_HPP_ */
