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
 * \file orca.hpp
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * 
 * \brief This file computes a collision-free trajectory for the ORCA algorithm
 * 
 ******************************************************************************/


#ifndef ORCA_H__
#define ORCA_H__

#include "communication/neighbor_selection.hpp"
#include "control/navigation.hpp"
#include "sensing/position_estimation.hpp"

#include "communication/mavlink_message_handler.hpp"
#include "communication/state.hpp"

extern "C"
{
#include "control/stabilisation.h"
#include "sensing/ahrs.h"
#include <stdbool.h>
#include <stdint.h>
}

/**
 * \brief The 3D plane structure
 */
typedef struct{
    float normal[3];                                    ///< The normal vector to the plane
    float point[3];                                     ///< A point of the plane
} plane_t;

/**
 * \brief The 3D line structure
 */
typedef struct{
    float direction[3];                                 ///< The direction vector of a line
    float point[3];                                     ///< A point of the line
} line_t;

/**
 * \brief   Structure of the ORCA module
 */
typedef struct
{
    float time_horizon;                                 ///< The time horizon of the method
    float inv_time_horizon;                             ///< The inverse of the time horizon
    float comfort_slider;                               ///< The value of the comfort slider
    uint8_t max_number_of_planes;                       ///< The maximum number of planes

    int8_t loop_count_orca;
    int8_t loop_count_collisions;
    
    Neighbors*                 neighbors;               ///< The pointer to the neighbor structure
    const Position_estimation* position_estimation;     ///< The pointer to the position estimation structure
    const ahrs_t*              ahrs;                    ///< The pointer to the attitude estimation structure
    const State*               state;                   ///< The pointer to the state structure
    Navigation*                navigation;              ///< The pointer to the navigation structure
    control_command_t*         controls_nav;            ///< The pointer to the controls_nav structure
} orca_t;

/**
 * \brief   Config structure of the ORCA module
 */
typedef struct
{
    float time_horizon;                                 ///< The time horizon of the method
    float comfort_slider;                               ///< The value of the comfort slider
} orca_conf_t;

/**
 * \brief   Initialize the ORCA module
 *
 * \param   orca                    The pointer to the ORCA structure
 * \param   orca_config             The config structure of the ORCA module
 * \param   neighbors               The pointer to the neighbor data structure
 * \param   position_estimation     The pointer to the position structure
 * \param   ahrs                    The pointer to the attitude estimation structure
 * \param   state                   The pointer to the state structure
 * \param   navigation_             The pointer to the navigation structure
 * \param   controls_nav            The pointer to the control nav structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool orca_init(orca_t *orca, orca_conf_t orca_config, Neighbors* neighbors, const Position_estimation* position_estimation, const ahrs_t *ahrs, const State* state, Navigation* navigation, control_command_t* controls_nav);

/**
 * \brief   Initialise the ORCA telemetry module
 *
 * \param   orca                    The pointer to the ORCA structure
 * \param   message_handler         The pointer to the message handler structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool orca_telemetry_init(orca_t* orca, Mavlink_message_handler* message_handler);


/**
 * \brief   Performs the ORCA update module
 *
 * \param   orca                    The pointer to the ORCA structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool orca_update(orca_t* orca);

static inline orca_conf_t orca_default_config()
{
    orca_conf_t conf = {};

    conf.comfort_slider = 0.0f;
    conf.time_horizon = 12.0f;

    return conf;
};


#endif // ORCA_H__