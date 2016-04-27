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
 * \file pfm.hpp
 * 
 * \author MAV'RIC Team
 * \author Gaëtan Burri
 * 
 * \brief This file computes a collision-free trajectory for the potential field
 * method (PFM)
 * 
 ******************************************************************************/


#ifndef PFM_H__
#define PFM_H__

#include "communication/neighbor_selection.hpp"
#include "control/navigation.hpp"
#include "sensing/position_estimation.hpp"

extern "C" 
{
#include <stdbool.h>
#include <stdint.h>
}

/**
 * \brief   Structure of the potential field module
 */
typedef struct
{
    float gain_attr;                                    ///< The gain for the attractive force
    float gain_rep;                                     ///< The gain for the repulsive force
    float dist_threshold_attr;                          ///< The threshold distance between linear and constant attractive force
    float dist_threshold_rep;                           ///< The threshold distance of the repulsive force from a quad
    float gain_force2velocity;                          ///< The gain to transform the force into velocity
    
    const Neighbors*            neighbors;              ///< The pointer to the neighbor structure
    const Position_estimation*  position_estimation;    ///< The pointer to the position estimation structure
    const ahrs_t*               ahrs;                   ///< The pointer to the attitude estimation structure
    const Navigation*           navigation;             ///< The pointer to the navigation structure
} pfm_t;

/**
 * \brief   Configuration structure of the potential field module
 */
typedef struct
{   
    float gain_attr;                                    ///< The gain for the attractive force
    float gain_rep;                                     ///< The gain for the repulsive force
    float dist_threshold_attr;                          ///< The threshold distance between linear and constant attractive force
    float dist_threshold_rep;                           ///< The threshold distance of the repulsive force from a quad
    
} pfm_conf_t;


/**
 * \brief   Initialize the PFM module
 *
 * \param   pfm                         The pointer to the pfm structure
 * \param   pfm_config                  The config structure
 * \param   neighbors                   The pointer to the neighbor data structure
 * \param   position_estimation         The pointer to the position structure
 * \param   ahrs                        The pointer to the attitude estimation structure
 * \param   navigation                  The pointer to the navigation structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool pfm_init(pfm_t *pfm, pfm_conf_t pfm_config, const Neighbors *neighbors, const Position_estimation* position_estimation, const ahrs_t *ahrs, const Navigation* navigation);

/**
 * \brief   Set the value of the different parameters
 *
 * \param   pfm                 The pointer to the pfm_t struct
 * \param   packet              The pointer to the decoded MAVLink message long
 * 
 * \return  The MAV_RESULT of the command
 */
mav_result_t pfm_set_parameters_value(pfm_t* pfm, mavlink_command_long_t* packet);

/**
 * \brief   Computes the new collision-free velocity
 *
 * \param   pfm                 pointer to the pfm_t struct
 * \param   new_velocity        the 3D output array
 */
void pfm_compute_new_velocity(pfm_t* pfm, float new_velocity[]);


static inline pfm_conf_t pfm_default_config(void)
{
    pfm_conf_t conf;

    conf.gain_attr = 5.0f;
    conf.gain_rep = 10.0f;
    conf.dist_threshold_attr = 5.0f;
    conf.dist_threshold_rep = 15.0f;

    return conf;
}

#endif // PFM_H_