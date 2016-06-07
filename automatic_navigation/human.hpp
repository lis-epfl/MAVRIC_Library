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
 * \file human.hpp
 * 
 * \author MAV'RIC Team
 * \author Gaëtan Burri
 * 
 * \brief This file computes a collision-free trajectory for the human friendly
 * algorithm
 * 
 ******************************************************************************/


#ifndef HUMAN_H__
#define HUMAN_H__

#include "communication/neighbor_selection.hpp"
#include "automatic_navigation/navigation.hpp"
#include "sensing/position_estimation.hpp"

extern "C"
{
#include <stdbool.h>
#include <stdint.h>
}

#define MAX_THETA_LOOKUP    19
#define MAX_PHI_LOOKUP      19

/**
 * \brief   Structure of the human-like module
 */
typedef struct
{
    int8_t loop_count_human;                            ///< The number of time the function was entered

    float time_horizon;                                 ///< The time horizon in seconds
    float time_interval;                                    ///< The time interval in seconds
    float brake_time;                                       ///< The brake time in seconds

    float angle_range_theta;                            ///< The range of the longitudinal angle
    float angle_resolution_theta;                       ///< The resolution of the longitudinal angle
    float angle_range_phi;                              ///< The range of the latitudinal angle
    float angle_resolution_phi;                         ///< The resolution of the latitudinal angle
    
    float cos_theta[MAX_THETA_LOOKUP];                  ///< The lookup table for the cosine of theta
    float sin_theta[MAX_THETA_LOOKUP];                  ///< The lookup table for the sine of theta
    float cos_phi[MAX_PHI_LOOKUP];                      ///< The lookup table for the cosine of phi
    float sin_phi[MAX_PHI_LOOKUP];                      ///< The lookup table for the sin of phi
    
    const Neighbors*                neighbors;              ///< The pointer to the neighbor structure
    const Position_estimation* position_estimation;     ///< The pointer to the position estimation structure
    const ahrs_t*               ahrs;                   ///< The pointer to the attitude estimation structure
    const State*                state;                  ///< The pointer to the state structure
    const Navigation*               navigation;             ///< The pointer to the navigation structure
} human_t;

typedef struct
{
    float time_horizon;
    float time_interval;

    float angle_range_theta;                            ///< The range of the longitudinal angle
    float angle_resolution_theta;                       ///< The resolution of the longitudinal angle
    float angle_range_phi;                              ///< The range of the latitudinal angle
    float angle_resolution_phi;                         ///< The resolution of the latitudinal angle
} human_conf_t;

/**
 * \brief   Initialize the HUMAN module
 *
 * \param   human                       The pointer to the human structure
 * \param   human_config                The pointer to the config structure
 * \param   neighbors                   The pointer to the neighbor data structure
 * \param   position_estimation         The pointer to the position structure
 * \param   ahrs                        The pointer to the attitude estimation structure
 * \param   navigation                  The pointer to the navigation structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool human_init(human_t *human, human_conf_t human_config, const Neighbors* neighbors, const Position_estimation* position_estimation, const ahrs_t *ahrs, const Navigation* navigation);

/**
 * \brief   Set the value of the different parameters
 *
 * \param   human               The pointer to the human_t struct
 * \param   packet              The pointer to the decoded MAVLink message long
 * 
 * \return  The MAV_RESULT of the command
 */
mav_result_t human_set_parameters_value(human_t* human, mavlink_command_long_t* packet);

/**
 * \brief   Computes the new collision-free velocity
 *
 * \param   human               pointer to the human_t struct
 * \param   new_velocity        the 3D output array
 */
void human_compute_new_velocity(human_t* human, float new_velocity[]);

static inline human_conf_t human_default_config(void)
{
    human_conf_t conf;

    conf.time_horizon = 5.0f;
    conf.time_interval = 1.0f;
    conf.angle_range_theta = 90.0f;
    conf.angle_resolution_theta = 10.0f;
    conf.angle_range_phi = 90.0f;
    conf.angle_resolution_phi = 10.0f;

    return conf;
};

#endif // HUMAN_H__