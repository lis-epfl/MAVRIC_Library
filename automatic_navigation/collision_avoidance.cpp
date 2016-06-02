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
 * \file collision_avoidance.cpp
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * 
 * \brief This file drives the collision avoidance strategy
 * 
 ******************************************************************************/


#include "automatic_navigation/collision_avoidance.hpp"
#include "communication/mav_modes.hpp"

extern "C"
{
#include "util/print_util.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Sets the parameters of the collision avoidance depending on the current strategy
 *
 * \param collision_avoidance   The pointer to the collision avoidance structure
 * \param packet                The pointer to the structure of the MAVLink command message long
 */
static mav_result_t collision_avoidance_set_parameters(Collision_avoidance* collision_avoidance, mavlink_command_long_t* packet);

/**
 * \brief Sets the strategy of the collision avoidance
 *
 * \param collision_avoidance   The pointer to the collision avoidance structure
 * \param packet                The pointer to the structure of the MAVLink command message long
 */
static mav_result_t collision_avoidance_set_strategy(Collision_avoidance* collision_avoidance, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t collision_avoidance_set_parameters(Collision_avoidance* collision_avoidance, mavlink_command_long_t* packet)
{
    mav_result_t result = MAV_RESULT_UNSUPPORTED;

    print_util_dbg_print("Setting new collision avoidance parameters...\r\n");

    switch(collision_avoidance->strategy)
    {
        case Collision_avoidance::ORCA:
            result = orca_set_parameters_value(&collision_avoidance->orca,packet);
            break;

        case Collision_avoidance::HUMAN:
            result = human_set_parameters_value(&collision_avoidance->human,packet);
            break;

        case Collision_avoidance::POTENTIAL_FIELD:
            result = pfm_set_parameters_value(&collision_avoidance->pfm,packet);
            break;

        case Collision_avoidance::FLOCKING:
            result = flocking_set_parameters_value(&collision_avoidance->flocking,packet);
            break;

        default:
            break;
    }

    if (result == MAV_RESULT_ACCEPTED)
    {
        uint8_t i;
        collision_avoidance->neighbors_.collision_log_.count_near_miss = 0;
        collision_avoidance->neighbors_.collision_log_.count_collision = 0;
        for (i = 0; i < collision_avoidance->neighbors_.config_.max_num_neighbors; i++)
        {
            collision_avoidance->neighbors_.collision_log_.near_miss_flag[i] = false;
            collision_avoidance->neighbors_.collision_log_.collision_flag[i] = false;
            collision_avoidance->neighbors_.collision_log_.transition_flag[i] = false;
        }

        print_util_dbg_print("New collision avoidance parameters set.\r\n");
    }

    return result;
}

static mav_result_t collision_avoidance_set_strategy(Collision_avoidance* collision_avoidance, mavlink_command_long_t* packet)
{
    mav_result_t result = MAV_RESULT_UNSUPPORTED;
    
    int32_t strategy_num = packet->param1;
    switch(strategy_num)
    {
        case 1:
            collision_avoidance->strategy = Collision_avoidance::ORCA;
            result = MAV_RESULT_ACCEPTED;
            break;
            
        case 2:
            collision_avoidance->strategy = Collision_avoidance::HUMAN;
            result = MAV_RESULT_ACCEPTED;
            break;
            
        case 3:
            collision_avoidance->strategy = Collision_avoidance::POTENTIAL_FIELD;
            result = MAV_RESULT_ACCEPTED;
            break;
            
        case 4:
            collision_avoidance->strategy = Collision_avoidance::FLOCKING;
            result = MAV_RESULT_ACCEPTED;
            break;
    }
    
    if (result == MAV_RESULT_ACCEPTED)
    {
        print_util_dbg_print("New strategy:");
        print_util_dbg_print_num(collision_avoidance->strategy,10);
        print_util_dbg_print("\r\n");
    }

    return result;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Collision_avoidance::Collision_avoidance(Neighbors& neighbors, State& state, Navigation& navigation, Position_estimation& position_estimation, const ahrs_t* ahrs, control_command_t* controls_nav, conf_t col_config):
    neighbors_(neighbors),
    navigation_(navigation),
    state_(state)
{
    bool init_success = true;

    controls_nav_ = controls_nav;

    // Init ORCA
    init_success &= orca_init(  &orca,
                                col_config.orca_config,
                                &neighbors,
                                &position_estimation,
                                ahrs,
                                &state);
    
    init_success &= human_init( &human,
                                col_config.human_config,
                                &neighbors,
                                &position_estimation,
                                ahrs,
                                &navigation);
    
    
    init_success &= pfm_init(   &pfm,
                                col_config.pfm_config,
                                &neighbors,
                                &position_estimation,
                                ahrs,
                                &navigation);
    
    
    init_success &= flocking_init(  &flocking,
                                    col_config.flocking_config,
                                    &neighbors,
                                    &position_estimation,
                                    ahrs,
                                    &navigation);
    
    strategy = col_config.strategy;

    if (init_success)
    {
        print_util_dbg_print("[COLLISION_AVOIDANCE] Initialised.\r\n");
    }
    else
    {
        print_util_dbg_print("[COLLISION_AVOIDANCE] Initialised error.\r\n");
    }
}

bool collision_avoidance_telemetry_init(Collision_avoidance* collision_avoidance, Mavlink_message_handler* message_handler)
{
    bool init_success = true;
    
    // Add callbacks for collision avoidance commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;
    
    callbackcmd.command_id = MAV_CMD_NAV_PATHPLANNING; // 81
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &collision_avoidance_set_parameters;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) collision_avoidance;
    init_success &= message_handler->add_cmd_callback(&callbackcmd);
    
    callbackcmd.command_id = MAV_CMD_NAV_ROI; // 80
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &collision_avoidance_set_strategy;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) collision_avoidance;
    init_success &= message_handler->add_cmd_callback(&callbackcmd);

    return init_success;
}

bool Collision_avoidance::update(Collision_avoidance* collision_avoidance)
{
    bool run_success = true;

    float new_velocity[3];

    Mav_mode mode_local = collision_avoidance->state_.mav_mode();
    
    run_success &= collision_avoidance->navigation_.update(&collision_avoidance->navigation_);

    if( mode_local.is_custom() &&
        ( (collision_avoidance->state_.mav_state_ == MAV_STATE_ACTIVE) && 
            ((collision_avoidance->navigation_.internal_state_ > Navigation::NAV_TAKEOFF) && (collision_avoidance->navigation_.internal_state_ < Navigation::NAV_STOP_ON_POSITION))
            && (mode_local.is_auto() || mode_local.is_guided()) )
        )
    {
        collision_avoidance->state_.mav_mode_custom |= Mav_mode::CUST_COLLISION_AVOIDANCE;
        switch(collision_avoidance->strategy)
        {
            case ORCA:
                orca_compute_new_velocity(&collision_avoidance->orca, collision_avoidance->controls_nav_->tvel, new_velocity);
                break;

            case HUMAN:
                human_compute_new_velocity(&collision_avoidance->human, new_velocity);
                break;

            case POTENTIAL_FIELD:
                pfm_compute_new_velocity(&collision_avoidance->pfm, new_velocity);
                break;

            case FLOCKING:
                flocking_compute_new_velocity(&collision_avoidance->flocking,collision_avoidance->controls_nav_->tvel, new_velocity);
                break;
            default:
                break;
        }
        
        collision_avoidance->controls_nav_->tvel[X] = new_velocity[X];
        collision_avoidance->controls_nav_->tvel[Y] = new_velocity[Y];
        collision_avoidance->controls_nav_->tvel[Z] = new_velocity[Z];
    }
    else
    {
        collision_avoidance->state_.mav_mode_custom &= ~Mav_mode::CUST_COLLISION_AVOIDANCE;
    }
    
    return run_success;
}