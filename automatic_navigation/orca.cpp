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
 * \file orca.cpp
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * 
 * \brief This file computes a collision-free trajectory for the ORCA algorithm
 * 
 ******************************************************************************/


#include "automatic_navigation/orca.hpp"
#include "communication/mav_modes.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/quaternions.h"
}

#define RVO_EPSILON 0.0001f

const uint8_t max_number_of_planes = 10;


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   computes the solution of a 1D linear program
 * 
 * \param   planes              The array of all planes
 * \param   index               The starting index of the plane
 * \param   line                The intersecting line between the two conflicting planes
 * \param   optimal_velocity    A 3D array
 * \param   new_velocity        The 3D output array
 * \param   direction_opt       Whether we are solving a 4D or a 3D linear program
 *
 * \return  whether the linear program has a solution or not
 */
static bool orca_linear_program1(plane_t const planes[], uint8_t const index, line_t const line, float const max_speed, float const optimal_velocity[], float new_velocity[], bool const direction_opt);

/**
 * \brief   computes the solution of a 2D linear program
 *
 * \param   planes              The array of all planes
 * \param   ind                 The index of the plane
 * \param   max_speed           The norm of the max velocity
 * \param   optimal_velocity    A 3D array
 * \param   new_velocity        The 3D output array
 * \param   direction_opt       Whether we are solving a 4D or a 3D linear program
 *
 * \return  whether the linear program has a solution or not
 */
static bool orca_linear_program2(plane_t const planes[], uint8_t const ind, float const max_speed, float const optimal_velocity[], float new_velocity[], bool const direction_opt);

/**
 * \brief   computes the solution of a 3D linear program
 *
 * \param   planes              The array of all planes
 * \param   plane_size          The number of planes
 * \param   optimal_velocity    A 3D array
 * \param   max_speed           The norm of the max velocity
 * \param   new_velocity        The 3D output array
 * \param   direction_opt       Whether we are solving a 4D or a 3D linear program
 *
 * \return the number of the last plane to be evaluated, if smaller than the number of planes, linear program is infeasible
 */
static float orca_linear_program3(plane_t const planes[], uint8_t const plane_size, float const optimal_velocity[], float const max_speed, float new_velocity[], bool const direction_opt);

/**
 * \brief   computes the solution of a 3D linear program
 *
 * \param   planes              The array of all planes
 * \param   plane_size          The number of planes
 * \param   ind                 The index of the plane for which the 3D linear program is infeasible
 * \param   max_speed           The norm of the max velocity
 * \param   new_velocity        The 3D output array
 * \param   max_num_of_planes   The maximum number of planes
 */
static void orca_linear_program4(plane_t const planes[], uint8_t const plane_size, uint8_t const ind, float const max_speed, float new_velocity[]);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static bool orca_linear_program1(plane_t const planes[], uint8_t const index, line_t const line, float const max_speed, float const optimal_velocity[], float new_velocity[], bool const direction_opt)
{
    uint8_t i;
    
    float dot_product = vectors_scalar_product(line.point,line.direction);
    float discriminant = SQR(dot_product) + SQR(max_speed) - vectors_norm_sqr(line.point);
    
    if (discriminant < 0.0f)
    {
        // Max speed sphere fully invalidates line
        return false;
    }
    
    float sqrt_discriminant = maths_fast_sqrt(discriminant);
    float t_left = -dot_product - sqrt_discriminant;
    float t_right = -dot_product + sqrt_discriminant;
    
    uint8_t index2;
    for ( index2 = 0; index2 < index; index2++)
    {
        float diff_points[3];
        
        for (i = 0; i < 3; i++)
        {
            diff_points[i] = planes[index2].point[i] - line.point[i];
        }
        
        float numerator = vectors_scalar_product(diff_points, planes[index2].normal);
        float denominator = vectors_scalar_product(line.direction, planes[index2].normal);
        
        if (SQR(denominator) <= RVO_EPSILON)
        {
            // Lines line is (almost) parallel to plane i
            if (numerator > 0.0f)
            {
                return false;
            }
            else
            {
                continue;
            }
        }
        
        float t = numerator / denominator;
        
        if (denominator >= 0.0f)
        {
            // Plane i bounds line on the left
            t_left = maths_f_max(t_left, t);
        }
        else
        {
            // Plane i bounds line on the right
            t_right = maths_f_min(t_right, t);
        }
        
        if (t_left > t_right)
        {
            return false;
        }
    }
    
    if (direction_opt)
    {
        // Optimize direction
        if (vectors_scalar_product(optimal_velocity, line.direction) > 0.0f)
        {
            // Take right extreme
            for (i = 0; i < 3; i++)
            {
                new_velocity[i] = line.point[i] + t_right * line.direction[i];
            }
        }
        else
        {
            // Take left extreme
            for (i = 0; i < 3; i++)
            {
                new_velocity[i] = line.point[i] + t_left * line.direction[i];
            }
        }
    }
    else
    {
        // Optimize closest point
        float diff_vel_point[3];
        for (i = 0; i < 3; i++)
        {
            diff_vel_point[i] = optimal_velocity[i] - line.point[i];
        }
        
        float t = vectors_scalar_product(line.direction, diff_vel_point);

        if (t < t_left)
        {
            for (i = 0; i < 3; i++)
            {
                new_velocity[i] = line.point[i] + t_left * line.direction[i];
            }
        }
        else
        {
            if (t > t_right)
            {
                for (i = 0; i < 3; i++)
                {
                    new_velocity[i] = line.point[i] + t_right * line.direction[i];
                }
            }
            else
            {
                for (i = 0; i < 3; i++)
                {
                    new_velocity[i] = line.point[i] + t * line.direction[i];
                }
            }
        }
    }
    return true;
}

static bool orca_linear_program2(plane_t const planes[], uint8_t const ind, float const max_speed, float const optimal_velocity[], float new_velocity[], bool const direction_opt)
{
    uint8_t i;
    uint8_t index;
    
    float plane_dist = vectors_scalar_product(planes[ind].point,planes[ind].normal);
    float plane_dist_sq = SQR(plane_dist);
    float radius_sq = SQR(max_speed);
    
    if (plane_dist_sq > radius_sq)
    {
        // Max speed sphere fully invalidates plane planeNo
        return false;
    }
    
    float plane_radius_sq = radius_sq - plane_dist_sq;
    
    float plane_center[3];
    for (i = 0; i < 3; i++)
    {
        plane_center[i] = plane_dist * planes[ind].normal[i];
    }
    
    if (direction_opt)
    {
        // Project direction optVelocity on plane ind
        float plane_opt_velocity[3];
        float scalar_product = vectors_scalar_product(optimal_velocity,planes[ind].normal);
        
        for(i = 0;i < 3; i++)
        {
            plane_opt_velocity[i] = optimal_velocity[i] - scalar_product * planes[ind].normal[i];
        }
        
        float plane_opt_velocity_length_sq = vectors_norm_sqr(plane_opt_velocity);
        
        if (plane_opt_velocity_length_sq <= RVO_EPSILON)
        {
            for(i = 0;i < 3; i++)
            {
                new_velocity[i] = plane_center[i];
            }
        }
        else
        {
            float sqrt_plane = maths_fast_sqrt(plane_radius_sq / plane_opt_velocity_length_sq);
            
            for(i = 0;i < 3; i++)
            {
                new_velocity[i] = plane_center[i] + sqrt_plane * plane_opt_velocity[i];
            }
        }
    }
    else
    {
        // Project point optVelocity on plane ind
        float diff_pts_vel[3];
        
        for(i = 0;i <3 ; i++)
        {
            diff_pts_vel[i] = planes[ind].point[i] - optimal_velocity[i];
        }
        
        float scalar_product = vectors_scalar_product(diff_pts_vel,planes[ind].normal);
        
        for(i = 0;i < 3; i++)
        {
            new_velocity[i] = optimal_velocity[i] + scalar_product * planes[ind].normal[i];
        }
        // If outside planeCircle, project on planeCircle
        if (vectors_norm_sqr(new_velocity) > radius_sq)
        {
            float plane_result[3];
            
            for(i = 0;i < 3; i++)
            {
                plane_result[i] = new_velocity[i] - plane_center[i];
            }
            
            float plane_result_length_sq = vectors_norm_sqr(plane_result);
            
            float plane_sqrt;
            if (plane_result_length_sq > 0.0f)
            {
                plane_sqrt = maths_fast_sqrt(plane_radius_sq / plane_result_length_sq);
            }
            else
            {
                plane_sqrt = 1.0f;
            }
            
            for (i = 0; i < 3; i++)
            {
                new_velocity[i] = plane_center[i] + plane_sqrt * plane_result[i];
            }
        }
    }
    
    for ( index = 0; index < ind; index++)
    {
        float diff_pts_new_vel[3];
        for (i = 0; i < 3; i++)
        {
            diff_pts_new_vel[i] = planes[index].point[i] - new_velocity[i];
        }
        if (vectors_scalar_product(planes[index].normal,diff_pts_new_vel)>0.0f)
        {
            /* Result does not satisfy constraint index. Compute new optimal result. */
            /* Compute intersection line of plane index and plane ind. */
            float cross_product[3];
            CROSS(planes[index].normal,planes[ind].normal,cross_product);
            
            if (vectors_norm_sqr(cross_product) <= RVO_EPSILON)
            {
                /* Planes ind and index are (almost) parallel, and plane index fully invalidates plane ind. */
                return false;
            }
            
            line_t line;
            float norm_cross_product = vectors_norm(cross_product);
            for (i = 0; i < 3; i++)
            {
                line.direction[i] = cross_product[i] / norm_cross_product;
            }
            float line_normal[3];
            CROSS(line.direction,planes[ind].normal,line_normal);
            
            float diff_points[3];
            for (i = 0; i < 3; i++)
            {
                diff_points[i] = planes[index].point[i] - planes[ind].point[i];
            }
            float scalar_product_points_normal = vectors_scalar_product(diff_points,planes[index].normal);
            float scalar_product_normals = vectors_scalar_product(line_normal,planes[index].normal);
            for (i = 0; i < 3; i++)
            {
                line.point[i] = planes[ind].point[i] + (scalar_product_points_normal / scalar_product_normals) * line_normal[i];
            }
            
            if (!(orca_linear_program1(planes,index,line,max_speed,optimal_velocity,new_velocity,direction_opt)))
            {
                return false;
            }
        }
    }
    return true;
}


static float orca_linear_program3(plane_t const planes[], uint8_t const plane_size, float const optimal_velocity[], float const max_speed, float new_velocity[], bool const direction_opt)
{
    uint8_t i;
    
    if (direction_opt)
    {
        /* Optimize direction. Note that the optimization velocity is of unit length in this case. */
        float norm_optimal_velocity = vectors_norm(optimal_velocity);
        for (i = 0; i < 3; i++)
        {
            new_velocity[i] = optimal_velocity[i] / norm_optimal_velocity * max_speed;
        }
    }
    else
    {
        if (vectors_norm_sqr(optimal_velocity) > SQR(max_speed))
        {
            /* Optimize closest point and outside circle. */
            float norm_optimal_velocity = vectors_norm(optimal_velocity);
            for (i = 0; i < 3; i++)
            {
                new_velocity[i] = optimal_velocity[i] / norm_optimal_velocity * max_speed;
            }
        }
        else
        {
            for (i = 0; i < 3; i++)
            {
                new_velocity[i] = optimal_velocity[i];
            }
        }
    }
    
    uint8_t ind;
    
    for (ind=0;ind<plane_size;ind++)
    {
        float diff_point_vel[3];
        for (i = 0; i < 3; i++)
        {
            diff_point_vel[i] = planes[ind].point[i] - new_velocity[i];
        }
        if (vectors_scalar_product(planes[ind].normal, diff_point_vel ) > 0.0f)
        {
            /* Result does not satisfy constraint ind. Compute new optimal result. */
            float temp_result[3];
            for (i = 0; i < 3; i++)
            {
                temp_result[i] = new_velocity[i];
            }
            if (!(orca_linear_program2(planes,ind,max_speed,optimal_velocity,new_velocity,direction_opt)))
            {
                for (i = 0; i < 3; i++)
                {
                    new_velocity[i] = temp_result[i];
                }
                
                return ind;
            }
        }
    }
    return plane_size;
}

static void orca_linear_program4(plane_t const planes[], uint8_t const plane_size, uint8_t const ind, float const max_speed, float new_velocity[])
{
    uint8_t i;
    
    uint8_t index,index2;
    
    plane_t proj_planes[max_number_of_planes];
    
    float distance = 0.0f;
    
    for (index = ind;index < plane_size;index++)
    {
        float diff_point_vel[3];
        for (i = 0; i < 3; i++)
        {
            diff_point_vel[i] = planes[index].point[i] - new_velocity[i];
        }
        if (vectors_scalar_product(planes[index].normal,diff_point_vel)>distance)
        {
            /* Result does not satisfy constraint of plane i. */
            
            for (index2 = 0;index2<index;index2++)
            {
                plane_t plane;
                float cross_product[3];
                vectors_cross_product(planes[index2].normal, planes[index].normal, cross_product);
                
                if (vectors_norm_sqr(cross_product)<=RVO_EPSILON)
                {
                    /* Plane index and plane index2 are (almost) parallel. */
                    if (vectors_scalar_product(planes[index].normal, planes[index2].normal) > 0.0f)
                    {
                        /* Plane index and plane index2 point in the same direction. */
                        continue;
                    }
                    else
                    {
                        /* Plane index and plane index2 point in opposite direction. */
                        for (i = 0; i < 3; i++)
                        {
                            plane.point[i] = 0.5f * (planes[index].point[i] + planes[index2].point[i]);
                        }
                    }
                }
                else
                {
                    float line_normal[3];
                    vectors_cross_product(cross_product,planes[index].normal,line_normal);
                    
                    float diff_points[3];
                    for (i = 0; i < 3; i++)
                    {
                        diff_points[i] = planes[index2].point[i] - planes[index].point[i];
                    }
                    
                    float scalar_prod_pts_normal = vectors_scalar_product(diff_points, planes[index2].normal);
                    float scalar_prod_normals = vectors_scalar_product(line_normal, planes[index2].normal);
                    for (i = 0; i < 3; i++)
                    {
                        plane.point[i] = planes[index].point[i] + (scalar_prod_pts_normal / scalar_prod_normals) * line_normal[i];
                    }
                }
                
                for (i = 0; i < 3; i++)
                {
                    plane.normal[i] = planes[index2].normal[i] - planes[index].normal[i];
                }
                float norm_normal = vectors_norm(plane.normal);
                for (i = 0; i < 3; i++)
                {
                    plane.normal[i] = plane.normal[i] / norm_normal;
                }
                
                proj_planes[index2] = plane;
            }
            
            float temp_result[3];
            for (i = 0; i < 3; i++)
            {
                temp_result[i] = new_velocity[i];
            }
            if (orca_linear_program3(proj_planes,index2,planes[index].normal,max_speed,new_velocity,true)<index2)
            {
                for (i = 0; i < 3; i++)
                {
                    new_velocity[i] = temp_result[i];
                }
            }
            float diff_point_vel[3];
            for (i = 0; i < 3; i++)
            {
                diff_point_vel[i] = planes[index].point[i] - new_velocity[i];
            }
            distance = vectors_scalar_product(planes[index].normal,diff_point_vel);
        }
    }
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool orca_init(orca_t *orca, orca_conf_t orca_config, Neighbors* neighbors, const Position_estimation* position_estimation, const ahrs_t *ahrs, const State* state)
{
    bool init_success = true;
    
    orca->neighbors = neighbors;
    orca->position_estimation = position_estimation;
    orca->ahrs = ahrs;
    orca->state = state;
        
    orca->time_horizon = orca_config.time_horizon;
    orca->inv_time_horizon = 1.0f / orca->time_horizon;
    
    orca->loop_count_orca = 0;
    orca->loop_count_collisions = 0;
    
    orca->comfort_slider = orca_config.comfort_slider;
    
    print_util_dbg_print("[ORCA] initialised.\r\n");
    
    return init_success;
}

mav_result_t orca_set_parameters_value(orca_t* orca, mavlink_command_long_t* packet)
{
    mav_result_t result = MAV_RESULT_ACCEPTED;
    
    print_util_dbg_print("Setting new ORCA parameters.\r\n");
    
    if(packet->param1 < 0.0001f)
    {
        result = MAV_RESULT_FAILED;
    }
    
    if (packet->param2 > 1.0f)
    {
        //orca->comfort_slider = 1.0f;
        result = MAV_RESULT_FAILED;
    }
    else if (packet->param2 < 0.0f)
    {
        //orca->comfort_slider = 0.0;
        result = MAV_RESULT_FAILED;
    }
    
    if(result == MAV_RESULT_ACCEPTED)
    {
        orca->time_horizon = packet->param1;
        orca->comfort_slider = packet->param2;
    }
    
    return result;
}


void orca_compute_new_velocity(orca_t *orca, float const optimal_velocity[], float new_velocity[])
{
    uint8_t ind, i;
    
    plane_t planes[max_number_of_planes];
    
    quat_t q_neighbor, q_neighbor_bf;
    
    float relative_position[3], relative_velocity[3];
    float combined_radius, dist_sq, combined_radius_sq, dot_product, w_length, w_length_sq;
    
    float w[3], unit_w[3], u[3], neighor_bf[3];
    
    float performance_velocity[3];
    for (i = 0; i < 3; i++)
    {
        performance_velocity[i] = optimal_velocity[i];
    }
    
    // Create agent ORCA planes
    for (ind=0; ind < orca->neighbors->number_of_neighbors_; ind++)
    {
        // Linear extrapolation of the position of the neighbor between two received messages
        for (i = 0; i < 3; i++)
        {
            relative_position[i] = orca->neighbors->neighbors_list_[ind].extrapolated_position[i] - orca->position_estimation->local_position.pos[i];
            relative_velocity[i] = orca->position_estimation->vel[i] - orca->neighbors->neighbors_list_[ind].velocity[i];
        }
        
        q_neighbor = quaternions_create_from_vector(relative_velocity);
        q_neighbor_bf = quaternions_global_to_local(orca->ahrs->qe,q_neighbor);
        
        neighor_bf[0] = q_neighbor_bf.v[0];
        neighor_bf[1] = q_neighbor_bf.v[1];
        neighor_bf[2] = q_neighbor_bf.v[2];
        
        for (i = 0; i < 3; i++)
        {
            relative_velocity[i] = neighor_bf[i];
        }
        
        q_neighbor = quaternions_create_from_vector(relative_position);
        q_neighbor_bf = quaternions_global_to_local(orca->ahrs->qe,q_neighbor);
        
        neighor_bf[0] = q_neighbor_bf.v[0];
        neighor_bf[1] = q_neighbor_bf.v[1];
        neighor_bf[2] = q_neighbor_bf.v[2];
        
        for (i = 0; i < 3; i++)
        {
            relative_position[i] = neighor_bf[i];
        }
        
        dist_sq = vectors_norm_sqr(relative_position);
        combined_radius = orca->neighbors->config_.safe_size_vhc + orca->neighbors->neighbors_list_[ind].size;
        combined_radius_sq = SQR(combined_radius);
        
        if (dist_sq > combined_radius_sq)
        {
            // No collisions
            for (i = 0; i < 3; i++)
            {
                w[i] = relative_velocity[i] - orca->inv_time_horizon * relative_position[i];
            }
            w_length_sq = vectors_norm_sqr(w);
            
            dot_product = vectors_scalar_product(w,relative_position);
            
            if ((dot_product < 0.0f)&&(SQR(dot_product) > (combined_radius_sq * w_length_sq)))
            {
                // Project on cut-off circle
                w_length = maths_fast_sqrt(w_length_sq);
                for (i = 0; i < 3; i++)
                {
                    unit_w[i] = w[i] / w_length;
                    planes[ind].normal[i] = unit_w[i];
                    u[i] = (combined_radius * orca->inv_time_horizon - w_length) * unit_w[i];
                }
            }
            else
            {
                // Project on cone
                float a = dist_sq;
                float b = vectors_scalar_product(relative_position, relative_velocity);
                float cross_product[3];
                vectors_cross_product(relative_position,relative_velocity,cross_product);
                float c = vectors_norm_sqr(relative_velocity) - vectors_norm_sqr(cross_product) / (dist_sq - combined_radius_sq);
                float t = (b + maths_fast_sqrt(SQR(b) - a * c)) / a;
                
                for (i = 0; i < 3; i++)
                {
                    w[i] = relative_velocity[i] - t * relative_position[i];
                }
                
                w_length = vectors_norm(w);
                
                for (i = 0; i < 3; i++)
                {
                    unit_w[i] = w[i] / w_length;
                    planes[ind].normal[i] = unit_w[i];
                    u[i] = (combined_radius * t - w_length) * unit_w[i];
                }
            }
        }
        else
        {
            // Collisions
            
            orca->loop_count_collisions++;
            orca->loop_count_collisions %= 1000;
            if ( (orca->loop_count_collisions == 0)&&(mav_modes_is_hil(orca->state->mav_mode())) ) 
            {
                print_util_dbg_print("Collision! ");
                print_util_dbg_print("Distance with neighbor ");
                print_util_dbg_print_num(ind,10);
                print_util_dbg_print("(x100):");
                print_util_dbg_print_num(sqrt(dist_sq) * 100.0f,10);
                print_util_dbg_print("\r\n");
            }
            
             float inv_time_step = 1.0f / orca->neighbors->config_.orca_time_step_s; //PROBLEM wrong time step
            
            for (i = 0; i < 3; i++)
            {
                w[i] = relative_velocity[i] - inv_time_step * relative_position[i];
            }
            
            w_length = vectors_norm(w);
            
            for (i = 0; i < 3; i++)
            {
                unit_w[i] = w[i] / w_length;
                planes[ind].normal[i] = unit_w[i];
                u[i] = (combined_radius * inv_time_step - w_length) * unit_w[i];
            }
        }
        
        for (i = 0; i < 3; i++)
        {
            planes[ind].point[i] = orca->position_estimation->vel_bf[i] + 0.5f * u[i];
        }
        
    }
    
    float plane_fail = orca_linear_program3(planes,orca->neighbors->number_of_neighbors_, optimal_velocity, orca->neighbors->config_.max_speed, performance_velocity, false);
    
    if (plane_fail < orca->neighbors->number_of_neighbors_)
    {
        orca_linear_program4(planes,orca->neighbors->number_of_neighbors_,plane_fail,orca->neighbors->config_.max_speed,performance_velocity);
    }
    
    float comfort_velocity[3];
    
    plane_fail = orca_linear_program3(planes,orca->neighbors->number_of_neighbors_,orca->position_estimation->vel_bf,orca->neighbors->config_.max_speed,comfort_velocity,false);
    if (plane_fail < orca->neighbors->number_of_neighbors_)
    {
        orca_linear_program4(planes,orca->neighbors->number_of_neighbors_,plane_fail,orca->neighbors->config_.max_speed,comfort_velocity);
    }
    
    for (i = 0; i < 3; i++)
    {
        new_velocity[i] = orca->comfort_slider * comfort_velocity[i] + (1.0f - orca->comfort_slider) * performance_velocity[i];
    }
    
    orca->loop_count_orca++;
    orca->loop_count_orca %= 1000;
    float orca_diff[3];
    
    for (i = 0; i < 3; i++)
    {
        orca_diff[i] = optimal_velocity[i] - new_velocity[i];
    }
    
    if ( (orca->loop_count_orca == 0)&&mav_modes_is_hil(orca->state->mav_mode()) ) 
    {
        print_util_dbg_print("Orca diffvel:");
        print_util_dbg_print_vector(orca_diff,2);
        print_util_dbg_print(", Optimal:");
        print_util_dbg_print_vector(optimal_velocity,2);
        print_util_dbg_print(", New:");
        print_util_dbg_print_vector(new_velocity,2);
        print_util_dbg_print("\r\n");
    }
    /*else
    {
        if (vectors_norm_sqr(orca_diff)>0.2)
        {
            print_util_dbg_print("Orca diffvel:");
            print_util_dbg_print_vector(orca_diff,2);
            print_util_dbg_print(", Optimal:");
            print_util_dbg_print_vector(optimal_velocity,2);
            print_util_dbg_print(", New:");
            print_util_dbg_print_vector(new_velocity,2);
            print_util_dbg_print("\n");
        }
    }*/
}

