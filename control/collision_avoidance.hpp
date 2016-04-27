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
 * \file collision_avoidance.hpp
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * 
 * \brief This file drives the collision avoidance strategy
 * 
 ******************************************************************************/


#ifndef COLLISION_AVOIDANCE_H__
#define COLLISION_AVOIDANCE_H__


#include "communication/neighbor_selection.hpp"
#include "control/navigation.hpp"
#include "control/orca.hpp"
#include "control/human.hpp"
#include "control/pfm.hpp"
#include "control/flocking.hpp"

#include "communication/state.hpp"
#include "sensing/position_estimation.hpp"


#include "communication/mavlink_message_handler.hpp"

extern "C"
{
#include <stdbool.h>
#include <stdint.h>
#include "sensing/ahrs.h"
}

/**
 * \brief   Enum of the possible collision avoidance strategies
 */
typedef enum
{
    ORCA,                           ///< The ORCA collision avoidance strategy
    HUMAN,                          ///< The human-like collision avoidance strategy
    POTENTIAL_FIELD,                ///< The potential field collision avoidance strategy
    FLOCKING                        ///< The flocking collision avoidance strategy
}collision_avoidance_strategy_t;

/**
 * \brief   Structure of the collision avoidance module
 */
typedef struct 
{
    Neighbors* neighbors;                           ///< The neighbors structure

    collision_avoidance_strategy_t strategy;        ///< The collision avoidance strategy

    orca_t orca;                                    ///< The ORCA structure
    human_t human;                                ///< The human-like structure
    pfm_t pfm;                                      ///< The potential field structure
    flocking_t flocking;                          ///< The flocking structure

    Navigation* navigation;                         ///< The pointer to the navigation structure
    State* state;                                   ///< The pointer to the state structure
    control_command_t* controls_nav;                ///< The pointer to the control nav structure
}collision_avoidance_t;

/**
 * \brief   Config structure of the collision avoidance module
 */
typedef struct
{
    collision_avoidance_strategy_t strategy;        ///< The collision avoidance strategy
    
    orca_conf_t orca_config;                        ///< The pointer to the config structure for the ORCA strategy
    human_conf_t human_config;                    ///< The pointer to the config structure for the human-like strategy
    pfm_conf_t pfm_config;                          ///< The pointer to the config structure for the potential field strategy
    flocking_conf_t flocking_config;              ///< The pointer to the config structure for the flocking strategy
}collision_avoidance_conf_t;

/**
 * \brief   Initialise the collision avoidance module
 *
 * \param   collision_avoidance         The pointer to the collision avoidance structure
 * \param   collision_avoidance_config  The pointer to the collision avoidance config structure
 * \param   state                           The pointer to the state structure
 * \param   navigation                      The pointer to the navigation structure
 * \param   position_estimation         The pointer to the position estimation structure
 * \param   gps                             The pointer to the gps structure
 * \param   barometer                       The pointer to the pressure structure
 * \param   ahrs                            The pointer to the ahrs structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool collision_avoidance_init(collision_avoidance_t* collision_avoidance, collision_avoidance_conf_t config, Neighbors* neighbors, State* state, Navigation* navigation, Position_estimation* position_estimation, const ahrs_t *ahrs, control_command_t* controls_nav);

/**
 * \brief   Initialize the MAVLink communication module for the collision avoidance
 * 
 * \param   collision-avoidance         The pointer to the data logging structure
 * \param   message_handler             The pointer to the MAVLink message handler
 *
 * \return  True if the init succeed, false otherwise
 */
bool collision_avoidance_telemetry_init(collision_avoidance_t* collision_avoidance, Mavlink_message_handler* message_handler);

/**
 * \brief   The task to perform collision avoidance
 *
 * \param   collision_avoidance         The pointer to the collision avoidance structure
 *
 * \return  The result of the task execution
 */
bool collision_avoidance_update(collision_avoidance_t* collision_avoidance);


static inline collision_avoidance_conf_t collision_avoidance_default_config(void)
{
    collision_avoidance_conf_t conf;

    conf.orca_config = orca_default_config();
    conf.human_config = human_default_config();
    conf.pfm_config = pfm_default_config();
    conf.flocking_config = flocking_default_config();

    return conf;
}

#endif // COLLISION_AVOIDANCE_H__