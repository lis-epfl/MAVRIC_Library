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
                                        State& state,
                                        saccade_controller_conf_t config ):
  flow_front_(flow_front),
  flow_back_(flow_back),
  ahrs_(ahrs),
  position_estimation_(position_estimation),
  state_(state)

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
    intersaccade_time_ = 2000;
    is_time_initialized_ = false;

    velocity_command_.xyz[0] = 0.0f;
    velocity_command_.xyz[1] = 0.0f;
    velocity_command_.xyz[2] = 0.0f;

    pid_controller_init(&altitude_pid_,&config.pid_config);

    altitude_value_ = -0.7f;

    velocity_value_ = 0.3f;

    movement_direction_ = 0.0f;

    can_filter_ = 1.0f;
    cad_filter_ = 0.077f;

    float back_cam_positions [N_points] = {-1.5920,-1.5382,-1.4840,-1.4333,-1.3777,-1.3255,-1.2724,-1.2228,-1.1676,-1.1160,-1.0631,-1.0091,-0.9589,-0.9025,-0.8501,-0.8020,-0.7477,-0.6925,-0.6421,-0.5853,-0.5337,-0.4816,-0.4292,-0.3765,-0.3237,-0.2707,-0.2177,-0.1647,-0.1118,-0.0589,-0.0001,0.0527,0.1056,0.1586,0.2115,0.2645,0.3175,0.3704,0.4231,0.4755,0.5277,0.5851,0.6362,0.6923,0.7475,0.7964,0.8498,0.9074,0.9587,1.0138,1.0678,1.1205,1.1767,1.2271,1.2811,1.3340,1.3904,1.4416,1.4963,1.5297};
    float front_cam_positions [N_points] = {-1.5717,-1.5188,-1.4654,-1.4162,-1.3621,-1.3120,-1.2568,-1.2057,-1.1539,-1.1014,-1.0530,-0.9989,-0.9490,-0.8982,-0.8466,-0.7942,-0.7408,-0.6866,-0.6371,-0.5869,-0.5361,-0.4847,-0.4327,-0.3803,-0.3274,-0.2742,-0.2207,-0.1671,-0.1192,-0.0653,-0.0113,0.0366,0.0906,0.1444,0.1982,0.2458,0.2992,0.3522,0.4049,0.4571,0.5088,0.5600,0.6161,0.6659,0.7204,0.7741,0.8269,0.8788,0.9299,0.9851,1.0345,1.0880,1.1407,1.1926,1.2486,1.2992,1.3494,1.4037,1.4575,1.5064};

    // Timer for the presaccade state

    begin_time_ = 0.0f;



    //For derotation

    // derotation_constant_ = (640*180)/(190*PI*195);


    // Init azimuth angles
    for (uint32_t i = 0; i < N_points; ++i)
    {
        azimuth_[i]             = back_cam_positions[i] + PI;
        sin_azimuth_[i] = quick_trig_sin(azimuth_[i]);
        inv_sin_azimuth_[i]     = 1.0f/sin_azimuth_[i];

        azimuth_[i + N_points]             = front_cam_positions[i];
        sin_azimuth_[i + N_points] = quick_trig_sin(azimuth_[i + N_points]);
        inv_sin_azimuth_[i + N_points]  = 1.0f/sin_azimuth_[i + N_points];

        cos_azimuth_[i] = quick_trig_cos(azimuth_[i]);
        cos_azimuth_[i + N_points] = quick_trig_cos(azimuth_[i + N_points]);

        flow_front_filtered_[i] = 0.0f;
        flow_back_filtered_[i] = 0.0f;

        //For derotation

        // derotated_flow_front_[i] = 0.0f;
        // derotated_flow_back_[i] = 0.0f;

    }
    //For derotation

    // yaw_velocity_buffer_.put_lossy(0.0f);
    // yaw_velocity_buffer_.put_lossy(0.0f);
    // yaw_velocity_buffer_.put_lossy(0.0f);
    // last_derotation_yaw_velocity_ = 0.0f;

}



bool Saccade_controller::init(void)
{
    return true;
}




bool Saccade_controller::update()
{

    flow_back_.update();
    flow_front_.update();

    for(uint32_t i = 0; i < N_points; ++i)
    {
        relative_nearness_[i] = 0.0f;
        relative_nearness_[i + N_points] = 0.0f;
        flow_back_filtered_[i] = cad_filter_ * flow_back_.of.x[i] + (1-cad_filter_) * flow_back_filtered_[i];
        flow_front_filtered_[i] = cad_filter_ * flow_front_.of.x[i] + (1-cad_filter_) * flow_front_filtered_[i];
    }

    //For derotation

    // yaw_velocity_buffer_.put_lossy(ahrs_.angular_speed[2]);
    // yaw_velocity_buffer_ .get(last_derotation_yaw_velocity_);

    // Calculate for both back and front the sum of the relative nearnesses
    // which are each given by RN = OF/sin(angle),
    // for(uint32_t i = 0; i < N_points; ++i)
    // {
    //     if(flow_back_filtered_[i] != 0)
    //     {
    //         derotated_flow_back_[i] = flow_back_filtered_[i] +last_derotation_yaw_velocity_*derotation_constant_;
    //     }

    //     else if(flow_front_filtered_[i] != 0)
    //     {
    //         derotated_flow_front_[i] = flow_front_filtered_[i] + last_derotation_yaw_velocity_*derotation_constant_;
    //     }
    // }

    // for (uint32_t i = 0; i < N_points; ++i)
    // {

    //     if(i<34)
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

    //     else if(i>33)
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



    for (uint32_t i = 0; i < N_points; ++i)
    {
        // if(azimuth_[i] < 0 or azimuth_[i+N_points] < PI)
        if(i<31)
        {
            if(flow_back_filtered_[i]>0)
            {
                relative_nearness_[i]   = maths_f_abs(flow_back_filtered_[i] * inv_sin_azimuth_[i]);
            // relative_nearness_[i]   = flow_back_filtered_[i];
            }

            if (flow_front_filtered_[i] < 0)
            {
                relative_nearness_[i+ N_points]  = maths_f_abs(flow_front_filtered_[i] * inv_sin_azimuth_[i+ N_points]);
                // relative_nearness_[i+ N_points]  = flow_front_filtered_[i];

            }
        }

        // else if(azimuth_[i] > 0 or azimuth_[i+N_points] > PI)
        if(i>30)
        {
            if(flow_back_filtered_[i] < 0)
            {
                relative_nearness_[i]   = maths_f_abs(flow_back_filtered_[i] * inv_sin_azimuth_[i]);
            }

            if (flow_front_filtered_[i] > 0)
            {
                relative_nearness_[i+ N_points]  = maths_f_abs(flow_front_filtered_[i] * inv_sin_azimuth_[i+ N_points]);
                // relative_nearness_[i+ N_points]  = flow_front_filtered_[i];

            }
        }

        // relative_nearness_[i]   = maths_f_abs(flow_back_filtered_[i] * inv_sin_azimuth_[i]);
        // relative_nearness_[i+ N_points]  = maths_f_abs(flow_front_filtered_[i] * inv_sin_azimuth_[i+ N_points]);
    }

    relative_nearness_[30] = 0;


    // Calculate the comanv's x and y components, to then calculate can and NOD.
    float comanv_x = 0.0f;
    float comanv_y = 0.0f;
    int counter_vect = 0;
    // int max_nearness = 0;
    // int max_azimuth = 0;

    for (uint32_t i = 0; i < 2*N_points; i++)
    {
        comanv_x += cos_azimuth_[i] * relative_nearness_[i];
        comanv_y += sin_azimuth_[i] * relative_nearness_[i];

        if(relative_nearness_[i] != 0)
        {
            counter_vect =counter_vect + 1;
        }

        // if(relative_nearness_[i] > max_nearness)
        // {
        //     max_nearness = relative_nearness_[i];
        //     max_azimuth = i;
        // }
    }


    // Intermediate variables :
    // can is the norm of the comanv vector,



    // Calculation of the CAN and the CAD

    can_ = (can_filter_ * maths_fast_sqrt(comanv_x * comanv_x + comanv_y * comanv_y)+ (1 - can_filter_) * can_ )/counter_vect ;

    // Sigmoid function for direction choice, it takes the can, a threshold and
    // a gain and describes how important it is for the drone to perform a saccade

    weighted_function_ = 1/(1 + pow(can_/threshold_ , - gain_));


    //Calculate the components of the unit vector opposite of the nearest object direction.

    float cad_x_unit = - comanv_x / can_;
    float cad_y_unit = - comanv_y / can_;

    cad_ = atan2(cad_y_unit,cad_x_unit);

    // // Goal direction in local frame
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

    float heading_error = 0.0f;

    //Altitude PID controller

    float altitude_error = altitude_value_ - (position_estimation_.local_position.pos[2]);

    velocity_command_.xyz[2] = pid_controller_update(&altitude_pid_, altitude_error);


    //Movement direction x and y components

    float movement_direction_x = 0.0f;
    float movement_direction_y = 0.0f;


    if(can_ > 0.5)
    {
        intersaccade_time_ = 500/can_;
    }
    else if(can_<=0.5)
    {
        intersaccade_time_ = 1200;
    }


    //Decide between saccade and intersaccade states

    switch (saccade_state_)
    {

        case PRESACCADE:


            if (state_.is_auto())
            {

                movement_direction_ = atan2(goal_lf[1],goal_lf[0]);

                attitude_command_.rpy[2]  = movement_direction_;

                heading_error = maths_f_abs( maths_calc_smaller_angle(attitude_command_.rpy[2]-current_rpy.rpy[2]) );

                if(heading_error<0.1)
                {
                    velocity_command_.xyz[0] = 0.0f;
                    velocity_command_.xyz[1] = 0.0f;

                    if(!is_time_initialized_)
                    {
                        begin_time_ = time_keeper_get_ms();
                        is_time_initialized_ = true;
                    }

                    if(time_keeper_get_ms()-begin_time_ > 2000)
                    {
                        saccade_state_            = INTERSACCADE;
                    }
                }
            }

        break;


        // This is the case where we are performing a saccade

        case SACCADE:

            heading_error = maths_calc_smaller_angle(attitude_command_.rpy[2]-current_rpy.rpy[2]);

            velocity_command_.xyz[0] = velocity_value_ * quick_trig_cos(heading_error) * (1.0f - maths_f_abs(heading_error) / PI );
            velocity_command_.xyz[1] = velocity_value_ * quick_trig_sin(heading_error) * (1.0f - maths_f_abs(heading_error) / PI );

            if ( maths_f_abs(heading_error ) < 0.1)
            {
                velocity_command_.xyz[0] = velocity_value_;
                velocity_command_.xyz[1] = 0;

                last_saccade_             = time_keeper_get_ms();

                for(uint32_t i = 0; i < N_points; ++i)
                {
                    flow_back_filtered_[i] = 0;
                    flow_front_filtered_[i] = 0;
                }

                saccade_state_            = INTERSACCADE;
            }

        break;

        // In this case, we are now in intersaccadic phase

        case INTERSACCADE:

            if (time_keeper_get_ms() - last_saccade_ > intersaccade_time_)
            {

                float yaw_angle = coord_conventions_get_yaw(ahrs_.qe);

                // Calculation of the movement direction (in radians)
                movement_direction_x = weighted_function_ * cad_x_unit + (1-weighted_function_) * (goal_bf[0]);
                movement_direction_y = weighted_function_ * cad_y_unit + (1-weighted_function_) * (goal_bf[1]);
                movement_direction_ = atan2(movement_direction_y,movement_direction_x);

                //If we want to stop the drone before performing the saccade, but doesn't seem to give good results

                // velocity_command_.xyz[0] = 0;
                // velocity_command_.xyz[1] = 0;

                // if(time_keeper_get_ms() - last_saccade_ > intersaccade_time_ + 700)
                // {
                    // attitude_command_.rpy[0]  = 0;
                    // attitude_command_.rpy[1]  = 0;
                    // attitude_command_.rpy[2]  = movement_direction_;

                    // saccade_state_            = SACCADE;
                // }

                attitude_command_.rpy[0]  = 0;
                attitude_command_.rpy[1]  = 0;
                attitude_command_.rpy[2]  = movement_direction_ + yaw_angle;

                velocity_command_.xyz[0] = velocity_value_ * quick_trig_cos(movement_direction_);
                velocity_command_.xyz[1] = velocity_value_ * quick_trig_sin(movement_direction_);


                saccade_state_            = SACCADE;


            }

        break;
    }


    return true;
}
