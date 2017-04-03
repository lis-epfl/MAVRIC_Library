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
 * \file ahrs.hpp
 *
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 * \author Julien Lecoeur
 *
 * \brief  Interface for the estimated Attitude and Heading Reference System
 *
 ******************************************************************************/


#ifndef AHRS_HPP_
#define AHRS_HPP_

#include <cstdint>
#include <cstdbool>
#include <array>

#include "util/coord_conventions.hpp"	/* coord_conventions_get_yaw */
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/quaternions.h"
}

/**
 * \brief   Interface for the estimated Attitude and Heading Reference System
 */
class AHRS
{
public:

    /**
     * \brief The calibration level of the filter
     */
    enum ahrs_state_t
    {
        AHRS_INITIALISING   = 0,    ///< Calibration level: Not initialised
        AHRS_LEVELING       = 1,    ///< Calibration level: No calibration, attitude estimation correction by accelero/magneto measurements
        AHRS_CONVERGING     = 2,    ///< Calibration level: leveling, correction of gyro biais
        AHRS_READY          = 3,    ///< Calibration level: leveled
    };

    /**
     * \brief   Main update function
     *
     * \return  Success
     */
    virtual bool update(void) = 0;


    /**
     * \brief     Last update in seconds
     *
     * \return    time
     */
    virtual time_us_t last_update_us(void) const = 0;


    /**
    * \brief   Indicates which estimate can be trusted
    *
    * \param   type    Type of estimate
    *
    * \return  boolean
    */
    virtual bool is_healthy(void) const = 0;


    /**
     * \brief     Estimated attitude
     *
     * \return    quaternion
     */
    virtual quat_t attitude(void) const = 0;


    /**
     * \brief     Estimated angular velocity
     *
     * \return    3D angular velocity
     */
    virtual std::array<float,3> angular_speed(void) const = 0;


    /**
     * \brief     Estimated linear acceleration
     *
     * \return    3D linear acceleration
     */
    virtual std::array<float,3> linear_acceleration(void) const = 0;


    /**
     * \brief	Gets the roll angle from the ahrs quaternion
     *
     * \return 	roll
     */
    virtual float roll() const
    {
        return coord_conventions_get_roll(attitude());
    }


    /**
     * \brief	Gets the pitch angle from the ahrs quaternion
     *
     * \return 	pitch
     */
    virtual float pitch() const
    {
        return coord_conventions_get_pitch(attitude());
    }


    /**
     * \brief	Gets the yaw angle from the ahrs quaternion
     *
     * \return 	yaw
     */
    virtual float yaw() const
    {
        return coord_conventions_get_yaw(attitude());
    }

};

#endif /* AHRS_HPP_ */
