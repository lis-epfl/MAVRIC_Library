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
 * \file pfm.cpp
 * 
 * \author MAV'RIC Team
 * \author Gaëtan Burri
 * 
 * \brief This file computes a collision-free trajectory for the potential field
 * method (PFM)
 * 
 ******************************************************************************/


#include "control/pfm.hpp"

extern "C" 
{
#include "util/print_util.h"
#include "util/quaternions.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Computes the attractive force
 *
 * \param   pfm                     pointer to the pfm_t struct
 * \param   attractive_force        the 3D output array
 */
static void pfm_calculate_attractive_force(pfm_t* pfm, float attractive_force[]);

/**
 * \brief   Computes the repulsive force
 *
 * \param   pfm                 pointer to the pfm_t struct
 * \param   repulsive_force     the 3D output array
 */
static void pfm_calculate_repulsive_force(pfm_t* pfm, float repulsive_force[]);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void pfm_calculate_attractive_force(pfm_t *pfm, float attractive_force[])
{
    uint8_t i;
    float goal_position[3], current_position[3], diff_current_goal[3];
    float dist;
    
    local_position_t goal = pfm->navigation->goal;
    
    for(i = 0; i < 3; i++)
    {
        current_position[i] = pfm->position_estimation->local_position.pos[i];
        goal_position[i] = goal.pos[i];
        diff_current_goal[i] = current_position[i]-goal_position[i];
    }
    dist = vectors_norm(diff_current_goal);
    
    if(dist > pfm->dist_threshold_attr)
    {
        for(i = 0; i < 3; i++)
        {
            attractive_force[i] = -pfm->dist_threshold_attr*pfm->gain_attr*diff_current_goal[i]/dist;
        }
    }
    else
    {
        for(i = 0; i < 3; i++)
        {
            attractive_force[i] = -pfm->gain_attr*diff_current_goal[i];
        }
    }
    
}

static void pfm_calculate_repulsive_force(pfm_t *pfm, float repulsive_force[])
{
    uint8_t ind, i;
    float relative_position[3], neighbor_velocity[3], force[3];
    float dist, dist_threshold_theta, neighbor_velocity_norm;
    float eccentricity, cos_theta = 0;
    float norm_product_sqr;
    
    for (ind=0; ind < pfm->neighbors->number_of_neighbors_; ind++)
    {
        for (i = 0; i < 3; i++)
        {
            relative_position[i] = pfm->position_estimation->local_position.pos[i]-pfm->neighbors->neighbors_list_[ind].extrapolated_position[i];
            neighbor_velocity[i] = pfm->neighbors->neighbors_list_[ind].velocity[i];
        }
        
        neighbor_velocity_norm = vectors_norm(neighbor_velocity);
        eccentricity = neighbor_velocity_norm/(2*pfm->neighbors->config_.cruise_speed);
        
        norm_product_sqr = vectors_norm_sqr(relative_position)*vectors_norm_sqr(neighbor_velocity);
        if (norm_product_sqr > 0.0025)
        {
            cos_theta = vectors_scalar_product(relative_position,neighbor_velocity)/sqrt(norm_product_sqr);
        }
        
        dist_threshold_theta = pfm->dist_threshold_rep*(1-SQR(eccentricity))/(1-eccentricity*cos_theta);
        dist = vectors_norm(relative_position);
        
        if (dist < dist_threshold_theta)
        {
            float k = pfm->gain_rep*pow(dist_threshold_theta/dist,3)*(1/dist-1/dist_threshold_theta);
            for (i = 0; i < 3; i++)
            {
                force[i] = k*relative_position[i];
            }
        } 
        else
        {
            for (i = 0; i < 3; i++)
            {
                force[i] = 0;
            }
        }
        
        for (i = 0; i < 3; i++)
        {
            repulsive_force[i] += force[i];
        }
    }
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool pfm_init(pfm_t *pfm, pfm_conf_t pfm_config, Neighbors *neighbors, const Position_estimation* position_estimation, const ahrs_t *ahrs, const State* state, Navigation* navigation)
{
    bool init_success = true;

    pfm->neighbors = neighbors;
    pfm->position_estimation = position_estimation;
    pfm->ahrs = ahrs;
    pfm->navigation = navigation;
    pfm->state = state;
    
    pfm->gain_attr = pfm_config.gain_attr;
    pfm->gain_rep = pfm_config.gain_rep;
    pfm->dist_threshold_attr = pfm_config.dist_threshold_attr;
    pfm->dist_threshold_rep = pfm_config.dist_threshold_rep;
    pfm->gain_force2velocity = neighbors->config_.cruise_speed/(pfm->dist_threshold_attr*pfm->gain_attr);
    
    pfm->loop_count_pfm = 0;
    
    print_util_dbg_print("PFM initialized.\r\n");

    return init_success;
}

mav_result_t pfm_set_parameters_value(pfm_t* pfm, mavlink_command_long_t* packet)
{
    mav_result_t result;
    
    print_util_dbg_print("Setting new potential field parameters.\r\n");
    
    result = MAV_RESULT_ACCEPTED;
    
    if (packet->param1 < 0.001f)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    if (packet->param2 < 0.0001f)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }   

    if (packet->param3 < 0.0001f)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    if (packet->param4 < (2.0f * pfm->neighbors->config_.safe_size_vhc))
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    if (result == MAV_RESULT_ACCEPTED)
    {
        pfm->gain_attr = packet->param1;
        pfm->gain_rep = packet->param2;
        pfm->dist_threshold_attr = packet->param3;
        pfm->dist_threshold_rep = packet->param4;
        pfm->gain_force2velocity = pfm->neighbors->config_.cruise_speed/(pfm->dist_threshold_attr*pfm->gain_attr);
    }
    
    return result;
}

void pfm_compute_new_velocity(pfm_t *pfm, float new_velocity[])
{
    uint8_t i;
    float force_from_potential[3];
    float att_force[3] = {0, 0, 0};
    float rep_force[3] = {0, 0, 0};
    quat_t q_velocity, q_velocity_bf;
    float velocity_norm;
    
    pfm_calculate_attractive_force(pfm, att_force);
    pfm_calculate_repulsive_force(pfm, rep_force);
    
    for(i = 0; i < 3; i++)
    {
        force_from_potential[i] = att_force[i]+rep_force[i];
    }
    
    for (i = 0; i < 3; i++)
    {
        new_velocity[i] = pfm->gain_force2velocity*force_from_potential[i];
    }
    
    velocity_norm = vectors_norm(new_velocity);
    if(velocity_norm > pfm->neighbors->config_.cruise_speed)
    {
        for (i = 0; i < 3; i++)
        {
            new_velocity[i] /= velocity_norm;
            new_velocity[i] *= pfm->neighbors->config_.cruise_speed;
        }
    }
    
    q_velocity = quaternions_create_from_vector(new_velocity);
    q_velocity_bf = quaternions_global_to_local(pfm->ahrs->qe, q_velocity);
    
    for (i = 0; i < 3; i++)
    {
        new_velocity[i] = q_velocity_bf.v[i];
    }
}