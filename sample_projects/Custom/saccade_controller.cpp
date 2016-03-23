/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file saccade_controller.c
 *
 * \author Darius Merk
 *
 * \brief   Saccade controller for indoors collision free navigation based on optic flow
 *
 ******************************************************************************/


#include "saccade_controller.hpp"

extern "C"
{
    #include "util/maths.h"
}





//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Saccade_controller::Saccade_controller(Serial& serial_flow_left, Serial& serial_flow_right, saccade_controller_conf_t config)
{
    flow_init(&flow_right_, &serial_flow_right);
    flow_init(&flow_left_, &serial_flow_left);
    gain_            = config.gain_;
    threshold_       = config.threshold_;
    goal_direction_  = config.goal_direction_;
}



bool Saccade_controller::init(void)
{
    return true;
}

/*void Saccade_controller::set_direction(heading_angle)
{
  Voir comment définir ca
}*/

bool Saccade_controller::update()
{

    
    float comanv_x = 1.0f;                      //x component of the comanv
    float comanv_y = 1.0f;                      //y component of the comanv
    
    
    // 125 points along the 160 pixels of the camera, start at pixel number 17 finish at number 142 such that the total angle covered by the 125 points is 140.625 deg.
    
    float angle_between_points = (140.625 / N_points);
   
    
    // Intermediate variables : can is the norm of the comanv vector, nearest object direction gives the angle in radians to the nearest object, cad gives the collision avoidance direction, opposite to the nearest object direction.
    
    //float can = 0.0f;
    float nearest_object_direction = 0.0f;
    //float cad  = 0.0f;
    float movement_direction = 0.0f;
    
    //Sigmoid function for direction choice, it takes the can, a threshold and a gain and describes how important it is for the drone to perform a saccade
    
    float weighted_function = 1.0f;
 
    // Quaternion given to attitude controller for the saccade
    quat_t quat_yaw_command;
   
    
    //Random number generation for the noise, the value of the noise is between 0 and 0.5. A new number is generated at each time.
    //ATTENTION CHECK THAT THE NOISE IS RANDOM AND ISN'T 10 TIMES THE SAME IN 1S FOR EXAMPLE
    
    //srand(time(NULL));
    float noise = 0.0f;
    //float noise = (rand() % 50)/100.;
    
    
    
    //Update the optic flow vectors
    
    flow_update(&flow_left_);
    flow_update(&flow_right_);
    
    //Calculate for both left and right the sum of the relative nearnesses which are each given by
    //RN = OF/sin(angle), then calculate the comanv's x and y components, to then calculate can and NOD.
    
    for(int i=0;i<N_points-1;++i)
    {
       
        azimuth_[i] = (-160.875 + i * angle_between_points)* (PI / 180.);
        azimuth_[i + N_points] = (19.125 + i * angle_between_points)* (PI / 180.);
        
        relative_nearness_[i] = flow_left_.of.x[i]/sin(azimuth_[i]);
        relative_nearness_[i + N_points] = flow_right_.of.x[i]/sin(azimuth_[i + N_points]);
        
    
        comanv_x += cos(azimuth_[i]) * relative_nearness_[i] + cos(azimuth_[i + N_points]) * relative_nearness_[i + N_points];
        comanv_y += sin(azimuth_[i]) * relative_nearness_[i] + sin(azimuth_[i + N_points]) * relative_nearness_[i + N_points];

    }
    
    
    
   //Calculation of the can and cad
    
    can_ = sqrt(pow(comanv_x , 2) + pow(comanv_y , 2));

    nearest_object_direction = atan(comanv_y/comanv_x);
    
    weighted_function = 1 / (1 + pow(can_/threshold_ , - gain_));
    
    cad_ = nearest_object_direction + PI;
    
    //Calculation of the movement direction (in radians)
    
    movement_direction = weighted_function * cad_ + (1-weighted_function) * (goal_direction_ + noise);
    
    //Transformation of the movement direction into a quaternion
    
    aero_attitude_t attitude;
    attitude.rpy[0] = 0.0f;
    attitude.rpy[1] = 0.0f;
    attitude.rpy[2] = movement_direction;
    
    quat_yaw_command = coord_conventions_quaternion_from_aero(attitude);
    
    //Attitude command given by the required movement direction
    
    attitude_command->quat = quat_yaw_command;
    
    return true;
}