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
 * \file flocking.cpp
 * 
 * \author MAV'RIC Team
 * \author Gaëtan Burri
 * 
 * \brief This file computes a collision-free trajectory for the flocking method
 * 
 ******************************************************************************/


#include "automatic_navigation/flocking.hpp"
#include "automatic_navigation/navigation.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/quaternions.h"
#include "util/quick_trig.h"
}

#define KP_YAW 0.2f

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Computes the motion planning term (cohesion and separation)
 *
 * \param   flocking                    pointer to the flocking_t struct
 * \param   u_cohesion_separation       the 3D output array
 * \param   relative_position           relative position of the neighbor
 * \param   cd                          gain depending on speed orientation
 * \param   a_ij                        weight of the neighbor
 */
static void flocking_calculate_cohesion_separation(flocking_t *flocking, float u_cohesion_separation[], float relative_position[], float cd, float a_ij);

/**
 * \brief   Computes the velocity matching term (alignment)
 *
 * \param   flocking            pointer to the flocking_t struct
 * \param   u_alignment         the 3D output array
 * \param   neighbor_velocity   velocity of the neighbor
 * \param   cd                  gain depending on speed orientation
 */
static void flocking_calculate_alignment(flocking_t *flocking, float u_alignment[], float const neighbors_velocity[], float cd);

/**
 * \brief   Computes the navigation feedback term (migration)
 *
 * \param   flocking            pointer to the flocking_t struct
 * \param   u_migration         the 3D output array
 */
static void flocking_calculate_migration(flocking_t* flocking, float u_migration[]);

/**
 * \brief   Calculate the sigma-norm
 *
 * \param   z                   number of which we want the sigma_norm
 * 
 * \return  The sigma_norm of the input
 */
static float flocking_sigma_norm(float z);

/**
 * \brief   Calculate the sigma-gradient
 *
 * \param   z                   vector of which we want the sigma_norm
 * \param   sigma               3D output array
 * 
 */
static void flocking_sigma_grad(float z[], float sigma[]);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void flocking_calculate_cohesion_separation(flocking_t *flocking, float u_cohesion_separation[], float relative_position[], float cd, float a_ij)
{
    uint8_t i;
    float sigma;
    float dist, dist_alpha, z;
    float n_ij[3];

    dist = vectors_norm(relative_position);
    dist_alpha = flocking_sigma_norm(dist);
    z = dist_alpha-flocking->d_alpha;
    sigma = (flocking->a+flocking->b)/2.0f * (z+flocking->c) / sqrt(1.0f+SQR(z+flocking->c)) + (flocking->a-flocking->b)/2.0f;
    flocking_sigma_grad(relative_position, n_ij);
    for (i = 0; i < 3; i++)
    {
        u_cohesion_separation[i] += cd*a_ij*sigma*n_ij[i];
    }
}

static void flocking_calculate_alignment(flocking_t *flocking, float u_alignment[], float const neighbors_velocity[], float cd)
{
    uint8_t i;
    
    for (i = 0; i < 3; i++)
    {
        u_alignment[i] += cd*(neighbors_velocity[i]-flocking->position_estimation->vel[i])+flocking->position_estimation->vel[i];
    }
}

static void flocking_calculate_migration(flocking_t *flocking, float u_migration[])
{
    uint8_t i;
    float q[3], p[3];
    
    local_position_t goal = flocking->navigation->goal.waypoint;
    
    for (i = 0; i < 3; i++)
    {
        q[i] = -flocking->c1*(flocking->position_estimation->local_position.pos[i]-goal.pos[i]);
        p[i] = -flocking->c2*flocking->position_estimation->vel[i];
        u_migration[i] = q[i]+p[i];
    }
}

static float flocking_sigma_norm(float z)
{
    float sigma_norm, epsilon = 0.1f;
    sigma_norm = 1.0f/epsilon*(sqrt(1.0f+epsilon*SQR(z))-1.0f);
    
    return sigma_norm;
}

static void flocking_sigma_grad(float z[], float sigma[])
{
    uint8_t i;
    float denom_inverse, epsilon = 0.1f;
    denom_inverse = 1.0f/(1.0f+epsilon*flocking_sigma_norm(vectors_norm(z)));
    for (i = 0; i < 3; i++)
    {
        sigma[i] = z[i]*denom_inverse;
    }
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool flocking_init(flocking_t *flocking, flocking_conf_t flocking_config, const Neighbors *neighbors, const Position_estimation* position_estimation, const ahrs_t *ahrs, const Navigation *navigation)
{
    bool init_success = true;

    flocking->neighbors = neighbors;
    flocking->position_estimation = position_estimation;
    flocking->ahrs = ahrs;
    flocking->navigation = navigation;
    
    flocking->d = flocking_config.d;
    flocking->range = 1.2f*flocking->d;
    flocking->range_alpha = flocking_sigma_norm(flocking->range);
    flocking->d_alpha = flocking_sigma_norm(flocking->d);
    flocking->a = flocking_config.a;
    flocking->b = flocking_config.b;
    if ((flocking->b-flocking->a) > 0.001f)
    {
        flocking->c = (flocking->b-flocking->a)/(2.0f*sqrt(flocking->a*flocking->b));
    }
    else
    {
        flocking->c = 0.0f; 
    }
    
    flocking->delta = 0.2f;
    flocking->c1 = flocking_config.c1;
    flocking->c2 = flocking_config.c2;
    //flocking->k1 = flocking_config.k1;
    //flocking->k2 = flocking_config.k2;
    //flocking->k3 = flocking_config.k3;
    //flocking->k123 = flocking->k1+flocking->k2+flocking->k3;
    //flocking->gain_control2vel = flocking->neighbors->config_.cruise_speed/(flocking->k3*flocking->neighbors->config_.cruise_speed/flocking->k123);
    
    print_util_dbg_print("Flocking initialized.\r\n");

    return init_success;
}

mav_result_t flocking_set_parameters_value(flocking_t* flocking, mavlink_command_long_t* packet)
{
    mav_result_t result;
    
    print_util_dbg_print("Setting new flocking parameters.\r\n");
    
    result = MAV_RESULT_ACCEPTED;
    
    if (packet->param1 < (2.0f * flocking->neighbors->config_.safe_size_vhc))
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    if (packet->param2 < 0.001f)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }

    if (packet->param3 < 0.001f)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    if (packet->param4 < 0.001f)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    if (packet->param5 < 0.001f)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    //if (packet->param6 < 0.0f)
    //{
        //result = MAV_RESULT_TEMPORARILY_REJECTED;
    //}
    //
    //if (packet->param7 < 0.0f)
    //{
        //result = MAV_RESULT_TEMPORARILY_REJECTED;
    //}
    
    if (result == MAV_RESULT_ACCEPTED)
    {
        flocking->d = packet->param1;
        flocking->a = packet->param2;
        flocking->b = packet->param3;
        flocking->c1 = packet->param4;
        flocking->c2 = packet->param5;
        //flocking->k1 = packet->param5;
        //flocking->k2 = packet->param6;
        //flocking->k3 = packet->param7;
        
        flocking->range = 1.2f*flocking->d;
        flocking->range_alpha = flocking_sigma_norm(flocking->range);
        flocking->d_alpha = flocking_sigma_norm(flocking->d);
        
        
        if ((flocking->b-flocking->a) > 0.001f)
        {
            flocking->c = (flocking->b-flocking->a)/(2.0f*sqrt(flocking->a*flocking->b));
        }
        else
        {
            flocking->c = 0.0f;
        }
        
        //flocking->k123 = flocking->k1+flocking->k2+flocking->k3;
        //flocking->gain_control2vel = flocking->neighbors->config_.cruise_speed/(flocking->k3*flocking->neighbors->config_.cruise_speed/flocking->k123);
    }
    
    return result;
}

void flocking_compute_new_velocity(flocking_t *flocking, float const optimal_velocity[], float new_velocity[])
{
    uint8_t ind, i, Ni = 0;
    float relative_position[3], neighbors_velocity[3];
    float u_mp[3] = {0.0f, 0.0f, 0.0f};
    float u_vm[3] = {0.0f, 0.0f, 0.0f};
    float u_nav[3] = {0.0f, 0.0f, 0.0f};
    //float u[3];

    float new_velocity_gf[3];
    
    float dist = 0.0f;
    float dist_alpha, velocity_norm;
    float z, cd = 0.0f, a_ij;//, A = 0.0f;
    
    for (ind = 0; ind < flocking->neighbors->number_of_neighbors_; ind++)
    {
        for (i = 0; i < 3; i++)
        {
            relative_position[i] = flocking->neighbors->neighbors_list_[ind].extrapolated_position[i] - flocking->position_estimation->local_position.pos[i];
            neighbors_velocity[i] = flocking->neighbors->neighbors_list_[ind].velocity[i];
        }
        dist = vectors_norm(relative_position);
        dist_alpha = flocking_sigma_norm(dist);
        z = dist_alpha/flocking->range_alpha;
        
        if (z < flocking->delta)
        {
            a_ij = 1.0f;
            Ni++;
        }
        else if (z < 1.0f)
        {
            a_ij = 1.0f/2.0f*(1.0f+cos(PI*(z-flocking->delta)/(1.0f-flocking->delta)));
            Ni++;
        } 
        else
        {
            a_ij = 0.0f;
        }       
        
        if (z < 1.0f)
        {
            float norm_product_sqr = vectors_norm_sqr(optimal_velocity)*vectors_norm_sqr(neighbors_velocity);
            if (norm_product_sqr > 0.0025f)
            {
                cd = (vectors_scalar_product(optimal_velocity, neighbors_velocity)/sqrt(norm_product_sqr)+1.0f)/2.0f;
                //cd belongs to the interval [0 (opposite direction); 1 (same direction)]
            }
            else
            {
                cd = 0.0f;
            }
            if (dist_alpha < flocking->d_alpha)         //repulsion
            {
                flocking_calculate_cohesion_separation(flocking, u_mp, relative_position, 1.0f, a_ij);
                //A += a_ij;
            }else{
                flocking_calculate_cohesion_separation(flocking, u_mp, relative_position, cd, a_ij);
                //A += a_ij*cd;
            }
            
            flocking_calculate_alignment(flocking, u_vm, neighbors_velocity, cd);
        } 
    }
    flocking_calculate_migration(flocking, u_nav);

    float norm_u_nav = vectors_norm(u_nav);
    for(i = 0; i < 3; i++)
    {   
        if (Ni > 0) //Normalization
        {
            u_vm[i] /= Ni;
            u_mp[i] /=Ni;
        }
        //if (A > 0.000001)
        //{
            //u_mp[i] /= A;
        //}
        //else{
            //u_mp[i] = 0;
        //}
        u_nav[i] *= maths_f_min(1, flocking->neighbors->config_.cruise_speed/norm_u_nav);
        //u_nav[i] *= maths_f_min(1, flocking->k123/flocking->k3*flocking->neighbors->config_.cruise_speed/vectors_norm(u_nav));
        
        new_velocity_gf[i] = u_mp[i]+u_vm[i]+u_nav[i] + flocking->position_estimation->vel[i];
        //u[i] = (flocking->k1*u_mp[i]+flocking->k2*u_vm[i]+flocking->k3*u_nav[i])/flocking->k123;
        //new_velocity_gf[i] = flocking->gain_control2vel*u[i]; //to remove if second line used
    }
    
    velocity_norm = vectors_norm(new_velocity_gf);
    if(velocity_norm > flocking->neighbors->config_.cruise_speed)
    {
        for (i = 0; i < 3; i++)
        {
            new_velocity_gf[i] /= velocity_norm;
            new_velocity_gf[i] *= flocking->neighbors->config_.cruise_speed;
        }
    }
    
    aero_attitude_t attitude_yaw = coord_conventions_quat_to_aero(flocking->ahrs->qe);
    attitude_yaw.rpy[0] = 0.0f;
    attitude_yaw.rpy[1] = 0.0f;
    attitude_yaw.rpy[2] = -attitude_yaw.rpy[2];
    quat_t q_rot = coord_conventions_quaternion_from_aero(attitude_yaw);

    quaternions_rotate_vector(q_rot, new_velocity_gf, new_velocity);
}