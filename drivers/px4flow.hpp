/*******************************************************************************
 * Copyright (c) 2009-2015, MAV'RIC Development Team
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
 * \file px4flow.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Interface for Optic Flow sensors
 *
 ******************************************************************************/

#ifndef PX4FLOW_HPP_
#define PX4FLOW_HPP_

#include "util/buffer.hpp"
#include <cstdint>

/**
 * \brief   Interface for Optic Flow sensors
 */
class PX4Flow
{
public:
    /**
     * \brief Enumeration of possible orientations for the sensor rel. to the drone
     */
    enum orientation_t
    {
        ORIENT_0_DEG   = 0,
        ORIENT_90_DEG  = 1,
        ORIENT_180_DEG = 2,
        ORIENT_270_DEG = 3,
    };

    /**
     * \brief Constructor
     */
    PX4Flow(void);

    /**
     * \brief   Indicates whether the measurements can be trusted
     *
     * \return  Value
     */
    bool healthy(void) const;

    /**
     * \brief   Main update function
     *
     * \return  success
     */
    virtual bool update(void) = 0;

    /**
     * \brief   Glue function used for scheduler
     *
     * \param   flow  Pointer to flow object

     * \return  success
     */
    static bool update_task(PX4Flow* flow)
    {
        return flow->update();
    }


    /**
     * \brief   Get flow in x direction
     *
     * \return  Value
     */
    float flow_x(void) const;

    /**
     * \brief   Get flow in y direction
     *
     * \return  Value
     */
    float flow_y(void) const;

    /**
     * \brief   Get flow quality
     *
     * \return  Value
     */
    uint8_t flow_quality(void) const;

    /**
     * \brief   Get velocity in x direction
     *
     * \return  Value
     */
    float velocity_x(void) const;

    /**
     * \brief   Get velocity in y direction
     *
     * \return  Value
     */
    float velocity_y(void) const;

    /**
     * \brief   Get velocity in z direction
     *
     * \return  Value
     */
    float velocity_z(void) const;

    /**
     * \brief   Get ground distance
     *
     * \return  Value
     */
    float ground_distance(void) const;

    /**
     * \brief   Get last update time in seconds
     *
     * \return  Value
     */
    float last_update_s(void) const;

protected:
    /**
     * \brief   Applies rotation to raw readings based on how the camera is mounted on drone
     *
     * \param   orientation     Orientation of camera
     * \param   flow_x_raw      X flow in camera frmae
     * \param   flow_y_raw      Y flow in camera frmae
     * \param   velocity_x_raw  X velocity in camera frmae
     * \param   velocity_y_raw  Y velocity in camera frmae
     */
    void rotate_raw_values(orientation_t orientation, float flow_x_raw, float flow_y_raw, float velocity_x_raw, float velocity_y_raw);

protected:
    float               flow_x_;                    ///< Optic flow in x direction (rad/s)
    float               flow_y_;                    ///< Optic flow in y direction (rad/s)
    uint8_t             flow_quality_;              ///< Quality of optic flow measurement (between 0 and 255)
    float               velocity_x_;                ///< Velocity in x direction (m/s)
    float               velocity_y_;                ///< Velocity in y direction (m/s)
    float               velocity_z_;                ///< Velocity in z direction (m/s)
    float               ground_distance_;           ///< Ground distance (m)
    Buffer_T<3,float>   ground_distance_buffer_;    ///< Buffer used to filter sonar measurement
    float               last_update_s_;             ///< Last update time in seconds
    bool                is_healthy_;                ///< Indicates if sensor data can be trusted
};

#endif /* PX4FLOW_HPP_ */
