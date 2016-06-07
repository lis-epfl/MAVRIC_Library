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
 * \file flocking.hpp
 * 
 * \author MAV'RIC Team
 * \author Gaëtan Burri
 * 
 * \brief This file computes a collision-free trajectory for the flocking method
 * 
 ******************************************************************************/


#ifndef FLOCKING_H__
#define FLOCKING_H__



#include "communication/neighbor_selection.hpp"
#include "automatic_navigation/navigation.hpp"
#include "sensing/position_estimation.hpp"

extern "C"
{
#include <stdbool.h>
#include <stdint.h>
}

/**
 * \brief   Structure of the flocking module
 */
typedef struct
{
    float range;                                        ///< The range of the flock
    float d;                                            ///< The alpha lattice
    float range_alpha;                                  ///< The sigma-norm of the range
    float d_alpha;                                      ///< The sigma-norm of the alpha lattice
    float a;                                            ///< Parameter for the sigma function of the cohesion/separation term
    float b;                                            ///< Parameter for the sigma function of the cohesion/separation term
    float c;                                            ///< Parameter for the sigma function of the cohesion/separation term
    float delta;                                        ///< Parameter for the rho function of the cohesion/separation term
    float c1;                                           ///< Parameter for the migration term
    float c2;                                           ///< Parameter for the migration term
    
    //float k1;                                         ///< Weight of the cohesion/separation term
    //float k2;                                         ///< Weight of the alignment term
    //float k3;                                         ///< Weight of the migration term
    //float k123;                                           ///< Sum of the weight
    //
    //float gain_control2vel    ;                           ///< The gain to transform the control into velocity
    
    const Neighbors*                neighbors;              ///< The pointer to the neighbor structure
    const Position_estimation* position_estimation;     ///< The pointer to the position estimation structure
    const ahrs_t*               ahrs;                   ///< The pointer to the attitude estimation structure
    const Navigation*               navigation;             ///< The pointer to the navigation structure
} flocking_t;

/**
 * \brief   Config structure of the flocking module
 */
typedef struct
{   
    float d;                                    ///< The alpha lattice
    float a;                                    ///< Parameter for the sigma function of the cohesion/separation term
    float b;                                    ///< Parameter for the sigma function of the cohesion/separation term
    float c1;                                   ///< Parameter for the migration term
    float c2;                                   ///< Parameter for the migration term
    
    //float k1;                                 ///< Weight of the cohesion/separation term
    //float k2;                                 ///< Weight of the alignment term
    //float k3;                                 ///< Weight of the migration term
    
} flocking_conf_t;


/**
 * \brief   Initialize the flocking module
 *
 * \param   flocking                    The pointer to the flocking structure
 * \param   flocking_config             The pointer to the config structure
 * \param   neighbors                   The pointer to the neighbor data structure
 * \param   position_estimation         The pointer to the position structure
 * \param   ahrs                        The pointer to the attitude estimation structure
 * \param   navigation                  The pointer to the navigation structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool flocking_init(flocking_t *flocking, flocking_conf_t flocking_config, const Neighbors *neighbors, const Position_estimation* position_estimation, const ahrs_t *ahrs, const Navigation *navigation);

/**
 * \brief   Set the value of the different parameters
 *
 * \param   flocking            The pointer to the flocking_t struct
 * \param   packet              The pointer to the decoded MAVLink message long
 * 
 * \return  The MAV_RESULT of the command
 */
mav_result_t flocking_set_parameters_value(flocking_t* flocking, mavlink_command_long_t* packet);

/**
 * \brief   Computes the new collision-free velocity
 *
 * \param   flocking            pointer to the flocking_t struct
 * \param   optimal_velocity    A 3D array
 * \param   new_velocity        the 3D output array
 */
void flocking_compute_new_velocity(flocking_t* flocking, float const optimal_velocity[], float new_velocity[]);

static inline flocking_conf_t flocking_default_config(void)
{
    flocking_conf_t conf;
    conf.d = 8.0f;
    conf.a = 1.0f;
    conf.b = 9.0f;
    conf.c1 = 0.4f;
    conf.c2 = 0.75f;
    //conf.k1 = 1.0f;
    //conf.k2 = 1.0f;
    //conf.k3 = 1.0f;

    return conf;
};

#endif // FLOCKING_H__