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
 * \file ins.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Inertial Navigation System (estimates position and velocity)
 *
 ******************************************************************************/


#ifndef INS_HPP_
#define INS_HPP_

#include <array>
#include "communication/mavlink_stream.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/coord_conventions.hpp"
#include "util/constants.hpp"

class INS
{
public:
    /**
      * \brief Constructor
      */
    INS(global_position_t origin = ORIGIN_EPFL);

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
    virtual float last_update_s(void) const = 0;


    /**
     * \brief     3D Position in meters (NED frame)
     *
     * \return    position
     */
    virtual local_position_t position_lf(void) const = 0;


    /**
     * \brief     Velocity in meters/seconds in NED frame
     *
     * \return    velocity
     */
    virtual std::array<float,3> velocity_lf(void) const = 0;


    /**
     * \brief     Absolute altitude above sea level in meters (>=0)
     *
     * \return    altitude
     */
    virtual float absolute_altitude(void) const = 0;


    /**
     * \brief     Enumeration of potential healthy estimates
     */
    enum healthy_t
    {
        XY_VELOCITY,
        Z_VELOCITY,
        XYZ_VELOCITY,
        XY_REL_POSITION,
        Z_REL_POSITION,
        XYZ_REL_POSITION,
        XY_ABS_POSITION,
        Z_ABS_POSITION,
        XYZ_ABS_POSITION
    };


    /**
    * \brief   Indicates which estimate can be trusted
    *
    * \param   type    Type of estimate
    *
    * \return  boolean
    */
    virtual bool is_healthy(INS::healthy_t type) const = 0;


    /**
     * \brief     Position of origin in global coordinates
     *
     * \return    origin
     */
    static const global_position_t& origin(void);

protected:
    static global_position_t origin_;
};

#endif /* INS_HPP_ */
