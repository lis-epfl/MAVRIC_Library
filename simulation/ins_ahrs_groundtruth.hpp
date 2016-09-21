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
 * \file ins_ahrs_groundtruth.hpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber

 * \brief   Mockup Inertial Navigation System (INS) and Attitude Heading Reference System (AHRS) providing groundtruth 
 *          for position and velocity, attitude, acceleration and rates, obtained from the dynamic model
 *
 ******************************************************************************/



#ifndef INS_AHRS_GROUNDTRUTH_HPP_
#define INS_AHRS_GROUNDTRUTH_HPP_

#include "sensing/ins.hpp"
#include "sensing/ahrs.hpp"
#include "simulation/dynamic_model.hpp"

class INS_AHRS_groundtruth : public INS
{
public:

    struct conf_t
    {
        global_position_t origin;   ///<    Global coordinates of the local frame's origin (ie. local (0, 0, 0) expressed in the global frame)
    };

    /**
      * \brief Constructor
      */
    INS_AHRS_groundtruth(Dynamic_model& model, ahrs_t& ahrs, const conf_t& config = default_config());

    /**
     * \brief   Main update function
     * \details Update dynamic model and internally stored values
     *
     * \return  Success
     */
    virtual bool update(void);


    /**
     * \brief     Last update of the internal values in seconds
     * \details   Value is only updated upon calling update()
     *
     * \return    time [seconds]
     */
    virtual inline float last_update_s(void) const {return last_update_s_;};


    /**
     * \brief     3D Position in meters (NED frame)
     * \details   Value is only updated upon calling update()
     *
     * \return    position [NED in meters]
     */
    virtual inline local_position_t position_lf(void) const {return position_lf_;};


    /**
     * \brief     Velocity in meters/seconds in NED frame
     * \details   Value is only updated upon calling update()
     *
     * \return    velocity [NED in meters/second]
     */
    virtual inline std::array<float,3> velocity_lf(void) const {return velocity_lf_;};


    /**
     * \brief     Absolute altitude above sea level in meters (>=0)
     * \details   Value is only updated upon calling update()
     *
     * \return    altitude [meters above sea level (>0)]
     */
    virtual inline float absolute_altitude(void) const {return absolute_altitude_;};


    /**
    * \brief   Indicates which estimate can be trusted (always returns true)
    *
    * \param   type    Type of estimate
    *
    * \return  boolean
    */
    virtual inline bool is_healthy(INS::healthy_t type) const {return true;};


    /**
     * \brief   Returns the AHRS structure
     * \details   Value is only updated upon calling update()
     *
     * \return ahrs
     */
     virtual inline const ahrs_t& ahrs() const {return ahrs_;};


    static inline conf_t default_config();

protected:
    Dynamic_model& model_;              ///< Dynamic model providing position and velocity information

    float last_update_s_;               ///< time stamp of last update in seconds
    local_position_t position_lf_;      ///< local position estimation (NED in meters)
    std::array<float,3> velocity_lf_;   ///< local velocity estimation (NED in meters/seconds)
    float absolute_altitude_;           ///< Absolute altitude above sea level in meters (>=0)
    ahrs_t& ahrs_;                      ///< Attitude, acceleration and rate estimation
};

INS_AHRS_groundtruth::conf_t INS_AHRS_groundtruth::default_config()
{
    conf_t config;
    config.origin = ORIGIN_EPFL;
    return config;
}

#endif /* INS_AHRS_GROUNDTRUTH_HPP_ */
