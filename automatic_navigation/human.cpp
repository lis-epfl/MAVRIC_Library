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
 * \file human.cpp
 * 
 * \author MAV'RIC Team
 * \author Gaëtan Burri
 * 
 * \brief This file computes a collision-free trajectory for the human friendly
 * algorithm
 * 
 ******************************************************************************/


#include "control/human.hpp"
#include "automatic_navigation/navigation.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/quaternions.h"
#include "util/quick_trig.h"
#include "util/maths.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool human_init(human_t *human, human_conf_t human_config, const Neighbors* neighbors, const Position_estimation* position_estimation, const ahrs_t *ahrs, const Navigation* navigation)
{
    bool init_success = true;

    human->neighbors = neighbors;
    human->position_estimation = position_estimation;
    human->ahrs = ahrs;
    human->navigation = navigation;
        
    human->time_horizon = human_config.time_horizon;
    human->time_interval = human_config.time_interval;
    human->brake_time = 5.0f;
    
    human->angle_range_theta = human_config.angle_range_theta;
    human->angle_resolution_theta = human_config.angle_resolution_theta;
    human->angle_range_phi = human_config.angle_range_phi;
    human->angle_resolution_phi = human_config.angle_resolution_phi;
    
    uint8_t i= 0;
    float theta, phi;
    if ( floor(2*human->angle_range_theta/human->angle_resolution_theta+1) > MAX_THETA_LOOKUP)
    {
        human->angle_resolution_theta = 2*human->angle_range_theta/(MAX_THETA_LOOKUP-1);
        print_util_dbg_print("Resolution of theta too small! Angle step increased to ");
        print_util_dbg_print_num(human->angle_resolution_theta, 10);
        print_util_dbg_print("\r\n");
    }
    if ( floor(2*human->angle_range_phi/human->angle_resolution_phi+1) > MAX_THETA_LOOKUP)
    {
        human->angle_resolution_phi= 2*human->angle_range_phi/(MAX_THETA_LOOKUP-1);
        print_util_dbg_print("Resolution of phi too small! Angle step increased to ");
        print_util_dbg_print_num(human->angle_resolution_phi, 10);
        print_util_dbg_print("\r\n");
    }
    for (theta = -human->angle_range_theta; theta <= human->angle_range_theta; theta+=human->angle_resolution_theta)
    {   
        human->cos_theta[i] = cos(maths_deg_to_rad(theta));
        human->sin_theta[i] = sin(maths_deg_to_rad(theta));
        i++;    
    }
    i = 0;
    for (phi = -human->angle_range_phi; phi <= human->angle_range_phi; phi+=human->angle_resolution_phi)
    {
        human->cos_phi[i] = cos(maths_deg_to_rad(phi));
        human->sin_phi[i] = sin(maths_deg_to_rad(phi));
        i++;
    }
    
    human->loop_count_human = 0;
    
    print_util_dbg_print("HUMAN initialized.\r\n");

    return init_success;
}

mav_result_t human_set_parameters_value(human_t* human, mavlink_command_long_t* packet)
{
    mav_result_t result;
    
    print_util_dbg_print("Setting new human-like parameters.\r\n");
    
    result = MAV_RESULT_ACCEPTED;
    
    if (packet->param1 > 100.0f || packet->param1 < 1.0f)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    if (packet->param2 > 10.0f || packet->param2 < 0.1f)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    if (packet->param3 > 90.0f || packet->param3 < 0.0f)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    if (packet->param5 > 90.0f || packet->param5 < 0.0f)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    if (packet->param4 < 0.5f || (2*packet->param3/packet->param4+1) > MAX_THETA_LOOKUP)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    if (packet->param6 < 0.5f || (2*packet->param5/packet->param6+1) > MAX_PHI_LOOKUP)
    {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    float T = 120.0f + (2.0f*packet->param3/packet->param4+1.0f)*(2.0f*packet->param5/packet->param6+1.0f)*(4.25f+(packet->param7-1.0f)*(1.0f+1.25f*packet->param1/packet->param2));
    
    if (T > 3100.0f)
    {
        print_util_dbg_print("T = ");
        print_util_dbg_print_num(T, 10);
        print_util_dbg_print("\r\n");
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }
    
    if (result == MAV_RESULT_ACCEPTED)
    {
        
        human->time_horizon = packet->param1;
        human->time_interval = packet->param2;
        human->angle_range_theta = packet->param3;
        human->angle_resolution_theta = packet->param4;
        human->angle_range_phi = packet->param5;
        human->angle_resolution_phi = packet->param6;
        
        uint8_t i = 0;
        float theta, phi;
        for (theta = -human->angle_range_theta; theta <= human->angle_range_theta; theta+=human->angle_resolution_theta)
        {
            human->cos_theta[i] = cos(maths_deg_to_rad(theta));
            human->sin_theta[i] = sin(maths_deg_to_rad(theta));
            i++;
        }
        i = 0;
        for (phi = -human->angle_range_phi; phi <= human->angle_range_phi; phi+=human->angle_resolution_phi)
        {
            human->cos_phi[i] = cos(maths_deg_to_rad(phi));
            human->sin_phi[i] = sin(maths_deg_to_rad(phi));
            i++;
        }
    }

    return result;
}

void human_compute_new_velocity(human_t *human, float new_velocity[])
{
    uint8_t ind, i, j, k;
    float theta, phi, t;
    float theta_speed, phi_speed;
    float dist, f, f_best, best_dist_sqr, remain_dist_sqr;
    
    quat_t q_temp,q1, q2, q_local;
    
    float relative_velocity[3], relative_position[3], relative_futur_position[3], futur_position[3];
    float goal_position[3], current_position[3], relative_goal_position[3];
    float best_direction[3] = {0,0,0}, direction[3], remain_dist[3];
    float H, dist2goal;
    float v_nom = human->neighbors->config_.cruise_speed, velocity_norm;
    
    local_position_t goal = human->navigation->goal;
    
    for (i = 0; i < 3; i++)
    {
        q1.v[i] = 0.0f;
        q2.v[i] = 0.0f;
        q_local.v[i] = 0.0f;
    }

    if (vectors_norm_sqr(human->position_estimation->vel) > 0.0001f)
    {
        theta_speed = atan2(human->position_estimation->vel[1],human->position_estimation->vel[0]);
        phi_speed = quick_trig_asin(human->position_estimation->vel[2]/vectors_norm(human->position_estimation->vel));
        q1.s = quick_trig_cos(theta_speed/2);
        q1.v[2] = quick_trig_sin(theta_speed/2);
        q2.s = quick_trig_cos(-phi_speed/2);
        q2.v[1] = quick_trig_sin(-phi_speed/2);
        q_local = quaternions_multiply(q1, q2);
    } 
    else
    {
        q_local.s = 1;
    }
    
    for (i = 0; i < 3; i++)
    {
        goal_position[i] = goal.pos[i];
        current_position[i] = human->position_estimation->local_position.pos[i];
        relative_goal_position[i] = goal_position[i]-current_position[i];
    }
    
    //initialize the best direction as the direction to the goal
    dist2goal = vectors_norm(relative_goal_position);
    if (dist2goal > 0.01f)
    {
        best_direction[0] = relative_goal_position[0]/dist2goal;
        best_direction[1] = relative_goal_position[1]/dist2goal;
        best_direction[2] = relative_goal_position[2]/dist2goal;
    }
    else
    {
        best_direction[0] = 0;
        best_direction[1] = 0;
        best_direction[2] = 0; 
    }
    
    H = maths_f_min(dist2goal,v_nom*human->time_horizon); //size of the "sphere" of view/prevision
    f_best = H;
    best_dist_sqr = SQR(2*dist2goal); //initialize the best remaining distance to the goal as 2 times the distance to the goal
    
    j = 0;
    for (theta = -human->angle_range_theta; theta <= human->angle_range_theta; theta+=human->angle_resolution_theta)
    {
        k = 0;
        for (phi = -human->angle_range_phi; phi <= human->angle_range_phi; phi+=human->angle_resolution_phi)
        {
            f = H;
            direction[0] = human->cos_phi[k]*human->cos_theta[j];
            direction[1] = human->cos_phi[k]*human->sin_theta[j];
            direction[2] = human->sin_phi[k];
            q_temp = quaternions_create_from_vector(direction);
            q_temp = quaternions_local_to_global(q_local,q_temp);
            for (i = 0; i < 3; i++)
            {
                direction[i] = q_temp.v[i];
            }
            
            for (ind = 0; ind < human->neighbors->number_of_neighbors_; ind++)
            {
                dist = H;
                for (i = 0; i < 3; i++)
                {
                    relative_position[i] = human->neighbors->neighbors_list_[ind].extrapolated_position[i]-human->position_estimation->local_position.pos[i];
                    relative_velocity[i] = human->neighbors->neighbors_list_[ind].velocity[i]-v_nom*direction[i];
                }
                    for(t = human->time_interval; t <= human->time_horizon; t+=human->time_interval)
                    {
                        for (i = 0; i < 3; i++)
                        {
                            relative_futur_position[i] = relative_position[i]+relative_velocity[i]*t;
                        }
                        if (vectors_norm_sqr(relative_futur_position) <= (human->neighbors->near_miss_dist_sqr()+SQR(human->neighbors->config_.cruise_speed*human->time_interval)))
                        {
                            dist = v_nom*(t-human->time_interval);
                            //print_util_dbg_print("COL!!!");
                            break;
                        }
                    } //END of the loop "time"
                f = maths_f_min(dist,f);
            } //END of the loop "neighbors"
            
            for (i = 0; i < 3; i++)
            {
                futur_position[i] = current_position[i]+f*direction[i];
                remain_dist[i] = goal_position[i]-futur_position[i];
            }
            remain_dist_sqr = vectors_norm_sqr(remain_dist);
            if (remain_dist_sqr < best_dist_sqr)
            {
                best_dist_sqr = remain_dist_sqr;
                f_best = f;
                if (dist2goal > 0.1f)
                {
                    for (i = 0; i < 3; i++)
                    {
                        best_direction[i] = direction[i];
                    }
                }
            }
            k++;
        }
        j++;
    } //END of the 4 loops
    
    velocity_norm = maths_f_min(v_nom, f_best/human->brake_time);
    //velocity_norm = maths_f_min(velocity_norm, dist2goal/5*v_nom);  //start to reduce speed at 5 meters from goal
    
    for (i = 0; i < 3; i++)
    {
        new_velocity[i] = velocity_norm*best_direction[i];
    }
    q_temp = quaternions_create_from_vector(new_velocity);
    q_temp = quaternions_global_to_local(human->ahrs->qe, q_temp);
    for (i = 0; i < 3; i++)
    {
        new_velocity[i] = q_temp.v[i];
    }
}