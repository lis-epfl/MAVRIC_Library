/*******************************************************************************
 * Copyfront (c) 2009-2016, MAV'RIC Development Team
 * All fronts reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyfront notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyfront notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyfront holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYFRONT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYFRONT HOLDER OR CONTRIBUTORS BE
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
 #include "hal/common/time_keeper.hpp"

extern "C"
{
    #include "util/maths.h"
    #include "util/quick_trig.h"
}




//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Saccade_controller::Saccade_controller( Flow& flow_front,
                                        Flow& flow_back,
                                        const ahrs_t& ahrs,
                                        Position_estimation& position_estimation,
                                        saccade_controller_conf_t config ):
  flow_front_(flow_front),
  flow_back_(flow_back),
  ahrs_(ahrs),
  position_estimation_(position_estimation)

{
    // Init members
    gain_            = config.gain_;
    threshold_       = config.threshold_;
    goal_direction_  = config.goal_direction_;
    pitch_           = config.pitch_;

    last_saccade_ = 0;

    can_ = 0;
    cad_ = 0;

    weighted_function_ = 0;

    saccade_state_ = PRESACCADE;

    attitude_command_.rpy[0]  = 0;
    attitude_command_.rpy[1]  = 0;
    attitude_command_.rpy[2]  = 0;
    intersaccade_time_ = 1000;
    is_time_initialized_ = false;

    derotation_constant_ = (640*180)/(190*PI*195);

    // 125 points along the 160 pixels of the camera, start at pixel number 17 finish at number 142 such that the total angle covered by the 125 points is 140.625 deg.
    float angle_between_points = (180. / N_points);
    // float angle_between_points = (180. / N_points);

    // Init azimuth angles
    for (uint32_t i = 0; i < N_points; ++i)
    {

        azimuth_[i]             = (91.5 + i * angle_between_points) * (PI / 180.0f);
        // azimuth_[i]             = (88.5 - i * angle_between_points) * (PI / 180.0f);
        sin_azimuth_[i] = quick_trig_sin(azimuth_[i]);
        inv_sin_azimuth_[i]     = 1.0f/sin_azimuth_[i];

        azimuth_[i + N_points]  = (  -88.5 + i * angle_between_points) * (PI / 180.0f);
        // azimuth_[i + N_points]  = (  -268.5 + i * angle_between_points) * (PI / 180.0f);
        sin_azimuth_[i + N_points] = quick_trig_sin(azimuth_[i + N_points]);
        inv_sin_azimuth_[i + N_points]  = 1.0f/sin_azimuth_[i + N_points];

        cos_azimuth_[i] = quick_trig_cos(azimuth_[i]);
        cos_azimuth_[i + N_points] = quick_trig_cos(azimuth_[i + N_points]);

        derotated_flow_front_[i] = 0.0f;
        derotated_flow_back_[i] = 0.0f;

    }

    yaw_velocity_buffer_.put_lossy(0.0f);
    yaw_velocity_buffer_.put_lossy(0.0f);
    yaw_velocity_buffer_.put_lossy(0.0f);

    last_derotation_yaw_velocity_ = 0.0f;

    velocity_command_.xyz[0] = 0.0f;
    velocity_command_.xyz[1] = 0.0f;
    velocity_command_.xyz[2] = 0.0f;

    pid_controller_init(&altitude_pid_,&config.pid_config);

    altitude_value_ = -0.7f;

    velocity_value_ = 0.3f;

    movement_direction_ = 0.0f;

    can_cad_filter_ = 0.95f;
}



bool Saccade_controller::init(void)
{
    return true;
}




bool Saccade_controller::update()
{
    // Random number generation for the noise, the value of the noise is between 0 and 0.5. A new number is generated at each time.
    // ATTENTION CHECK THAT THE NOISE IS RANDOM AND ISN'T 10 TIMES THE SAME IN 1S FOR EXAMPLE
    // float noise = 0.0f;

    flow_back_.update();
    flow_front_.update();

    // yaw_velocity_buffer_.put_lossy(ahrs_.angular_speed[2]);
    // yaw_velocity_buffer_ .get(last_derotation_yaw_velocity_);


    // Calculate for both back and front the sum of the relative nearnesses
    // which are each given by RN = OF/sin(angle),
    // for(uint32_t i = 0; i < N_points; ++i)
    // {
    //     if(flow_back_.of.x[i] != 0)
    //     {
    //         derotated_flow_back_[i] = flow_back_.of.x[i] +last_derotation_yaw_velocity_*derotation_constant_;
    //     }

    //     else if(flow_front_.of.x[i] != 0)
    //     {
    //         derotated_flow_front_[i] = flow_front_.of.x[i] + last_derotation_yaw_velocity_*derotation_constant_;
    //     }

    //     relative_nearness_[i] = 0.0f;
    //     relative_nearness_[i + N_points] = 0.0f;


    // }

    // for (uint32_t i = 0; i < N_points; ++i)
    // {

    //     if(i<36)
    //     {
    //         if(derotated_flow_back_[i]> 0)
    //         {
    //             relative_nearness_[i]   = maths_f_abs(derotated_flow_back_[i] * inv_sin_azimuth_[i]);
    //         }

    //         else if (derotated_flow_front_[i] < 0)
    //         {
    //             relative_nearness_[i + N_points]  = maths_f_abs(derotated_flow_front_[i] * inv_sin_azimuth_[i]);
    //         }
    //     }

    //     else if(i>36)
    //     {
    //         if(derotated_flow_back_[i] < 0)
    //         {
    //             relative_nearness_[i]   = maths_f_abs(derotated_flow_back_[i] * inv_sin_azimuth_[i]);
    //         }

    //         else if (derotated_flow_front_[i] > 0)
    //         {
    //             relative_nearness_[i + N_points]  = maths_f_abs(derotated_flow_front_[i] * inv_sin_azimuth_[i + N_points]);
    //         }
    //     }
    // }

    for(uint32_t i = 0; i < N_points; ++i)
    {
        relative_nearness_[i] = 0.0f;
        relative_nearness_[i + N_points] = 0.0f;
    }

    for (uint32_t i = 0; i < N_points; ++i)
    {   
        
        if(i<34)
        {
            if(flow_back_.of.x[i]>0)
            {
                relative_nearness_[i]   = maths_f_abs(flow_back_.of.x[i] * inv_sin_azimuth_[i]);
            }

            else if (flow_front_.of.x[i] < 0)
            {
                relative_nearness_[i+ N_points]  = maths_f_abs(flow_front_.of.x[i] * inv_sin_azimuth_[i+ N_points]);
            }   
        }

        else if(i>33)
        {
            if(flow_back_.of.x[i] < 0) 
            {
                relative_nearness_[i]   = maths_f_abs(flow_back_.of.x[i] * inv_sin_azimuth_[i]);
            }
            
            else if (flow_front_.of.x[i] > 0)
            {
                relative_nearness_[i+ N_points]  = maths_f_abs(flow_front_.of.x[i] * inv_sin_azimuth_[i+ N_points]);
            }
        }
    }



    // Calculate the comanv's x and y components, to then calculate can and NOD.
    float comanv_x = 0.0f;
    float comanv_y = 0.0f;
    for (uint32_t i = 0; i < 2*N_points; i++)
    {
        comanv_x += cos_azimuth_[i] * relative_nearness_[i];
        comanv_y += sin_azimuth_[i] * relative_nearness_[i];

    }


    // Intermediate variables :
    // can is the norm of the comanv vector,
    // nearest object direction gives the angle in radians to the azimuth_[i])


    // Calculation of the CAN and the CAD

    can_ = can_cad_filter_ * maths_fast_sqrt(comanv_x * comanv_x + comanv_y * comanv_y)+ (1 - can_cad_filter_) * can_  ;
    // can_ = maths_fast_sqrt(comanv_x * comanv_x + comanv_y * comanv_y);
    // Sigmoid function for direction choice, it takes the can, a threshold and
    // a gain and describes how important it is for the drone to perform a saccade

    weighted_function_ = 1/(1 + pow(can_/threshold_ , - gain_));


    // // Intermediate variable that gives the angle to the COMANV
    // float nearest_object_direction = 0.0f;
    // if (comanv_x != 0.0f)
    // {
    //     nearest_object_direction = atan2(comanv_y, comanv_x);
    // }

    // cad_ = nearest_object_direction + PI;


    //Calculate the components of the unit vector opposite of the nearest object direction.

    float cad_x_unit = - comanv_x / can_;
    float cad_y_unit = - comanv_y / can_;

    cad_ = can_cad_filter_ * atan2(cad_y_unit,cad_x_unit) + (1 - can_cad_filter_) * cad_;
    // cad_ = atan2(cad_y_unit,cad_x_unit);
    // Goal direction in local frame
    float goal_lf[3];
    goal_lf[0] = quick_trig_cos(goal_direction_);
    goal_lf[1] = quick_trig_sin(goal_direction_);
    goal_lf[2] = 0.0f;

    // Goal direction in body frame

    float goal_bf[3];
    quaternions_rotate_vector(quaternions_inverse(ahrs_.qe), goal_lf, goal_bf);

    // Normalization of the goal direction vector.
    float goal_bf_norm = maths_fast_sqrt(goal_bf[0] * goal_bf[0]+ goal_bf[1] * goal_bf[1] + goal_bf[2] * goal_bf[2]);

    if(goal_bf_norm != 0){

    goal_bf[0] = goal_bf[0] / goal_bf_norm;
    goal_bf[1] = goal_bf[1] / goal_bf_norm;
    goal_bf[2] = goal_bf[2] / goal_bf_norm;

    }

    //Current heading information used to see if the drone has finished the saccade and declaration of error variable

    aero_attitude_t current_rpy = coord_conventions_quat_to_aero(ahrs_.qe);

    float altitude_error = altitude_value_ - (position_estimation_.local_position.pos[2]);
    // float altitude_error = 0;

    velocity_command_.xyz[2] = pid_controller_update(&altitude_pid_, altitude_error);

    float heading_error = 0.0f;

    //Noise for the goal direction

    float noise = 0.0f;

    float movement_direction_x = 0.0f;
    float movement_direction_y = 0.0f;
    float begin_time = 0.0f;


    //Decide between saccade and intersaccade states

    switch (saccade_state_)
    {

        case PRESACCADE:

            if(!is_time_initialized_)
            {
                begin_time = time_keeper_get_ms();
                is_time_initialized_ = true;
            }

            movement_direction_ = atan2(goal_lf[1],goal_lf[0]);



            attitude_command_.rpy[2]  = movement_direction_;
            // attitude_command_.quat    = coord_conventions_quaternion_from_rpy(attitude_command_.rpy);

            heading_error = maths_f_abs( maths_calc_smaller_angle(attitude_command_.rpy[2]-current_rpy.rpy[2]) );
            if(heading_error<0.1)
            {
               velocity_command_.xyz[0] = velocity_value_;

                if(time_keeper_get_ms()-begin_time > 1000)
                {
                    saccade_state_            = INTERSACCADE;
                }

                // if(time_keeper_get_ms()-begin_time > 3000)
            }



        break;
        // This is the case where we are performing a saccade

        case SACCADE:

            heading_error = maths_f_abs( maths_calc_smaller_angle(attitude_command_.rpy[2]-current_rpy.rpy[2]) );

            if ( heading_error < 0.1)
            // if (time_keeper_get_ms() - last_saccade_ > intersaccade_time_ + 2000)
            {
                velocity_command_.xyz[0] = velocity_value_;
                // velocity_command_.xyz[1] = 0.2 * quick_trig_sin(movement_direction);

                last_saccade_             = time_keeper_get_ms();
                saccade_state_            = INTERSACCADE;
            }
        break;

        // In this case, we are now in intersaccadic phase

        case INTERSACCADE:

            if (time_keeper_get_ms() - last_saccade_ > intersaccade_time_)
            {

                // Calculation of the movement direction (in radians)
                movement_direction_x = weighted_function_ * cad_x_unit + (1-weighted_function_) * (goal_bf[0]  + noise);
                movement_direction_y = weighted_function_ * cad_y_unit + (1-weighted_function_) * (goal_bf[1]  + noise);
                movement_direction_ = atan2(movement_direction_y,movement_direction_x);

                velocity_command_.xyz[0] = 0;
                velocity_command_.xyz[1] = 0;

                // movement_total_ +=movement_direction;
                attitude_command_.rpy[0]  = 0;
                attitude_command_.rpy[1]  = 0;
                attitude_command_.rpy[2]  += movement_direction_;
                // attitude_command_.rpy[2]  += 0.75;
                // attitude_command_.quat    = coord_conventions_quaternion_from_rpy(attitude_command_.rpy);

                saccade_state_            = SACCADE;
            }
        break;
    }


    return true;
}
