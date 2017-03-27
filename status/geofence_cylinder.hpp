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
* \file geofence_cylinder.hpp
*
* \author MAV'RIC Team
* \author Julien Lecoeur
*
* \brief  Geofence with cylindrical shape
*
******************************************************************************/


#ifndef GEOFENCE_CYLINDER_HPP_
#define GEOFENCE_CYLINDER_HPP_

#include "status/geofence.hpp"
#include "util/constants.hpp"


/**
 * \brief   Geofence with cylindrical shape
 */
class Geofence_cylinder: public Geofence
{
public:
    /**
     * \brief Configuration structure
     */
    struct conf_t
    {
        bool                enabled;            ///< Whether the geofence is active. If false, is_allowed will always return true.
        global_position_t   center;             ///< Center of the cylinder
        float               radius;             ///< Radius of the cylinder in meters
        float               height;             ///< Height of the cylinder in meters
        bool                allowed_inside;     ///< Indicates if the allowed space is inside the cylinder (true), or outside (false)
    };


    /**
     * \brief   Default configuration
     *
     * \return  config
     */
    static inline conf_t default_config(void);


    /**
     * \brief Constructor
     *
     * \param config    Configuration structure
     */
    Geofence_cylinder(conf_t config = default_config());


   /**
    * \brief     Indicates if the position is allowed by the geofence
    *
    * \return    boolean (true if allowed, false if not)
    */
   bool is_allowed(const global_position_t& position) const;


   /**
    * \brief     Computes the closest border between allowed and disallowed space
    *
    * \param     current_position      Current position of the MAV (input)
    * \param     border_position       Closest position at the border (output)
    * \param     distance              Distance to closest border (output)
    *
    * \return    success               If false, no border_position and/or distance could be computed
    */
   bool closest_border(const global_position_t& current_position, global_position_t& border_position, float& distance) const;

 // protected:
    conf_t config_;     ///< Configuration
};


Geofence_cylinder::conf_t Geofence_cylinder::default_config(void)
{
    conf_t config;

    config.enabled          = true;
    config.center           = ORIGIN_EPFL;
    config.radius           = 200.0f;
    config.height           = 150.0f;
    config.allowed_inside   = true;

    return config;
};

#endif /* GEOFENCE_CYLINDER_HPP_ */
