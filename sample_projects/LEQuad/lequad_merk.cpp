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
 * \file lequad_merk.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/


#include "sample_projects/LEQuad/lequad_merk.hpp"
#include "sample_projects/LEQuad/lequad.hpp"



LEQuad_merk::LEQuad_merk(Imu& imu, Barometer& barometer, Gps& gps, Sonar& sonar, Serial& serial_mavlink,
                         Satellite& satellite, Led& led, File& file_flash, Battery& battery,
                         Servo& servo_0, Servo& servo_1, Servo& servo_2, Servo& servo_3, Servo& servo_4,
                         Servo& servo_5, Servo& servo_6, Servo& servo_7, File& file1, File& file2,
                         Flow& flow_front, Flow& flow_back, const conf_t& config):
    LEQuad(imu, barometer, gps_mocap_, sonar, serial_mavlink, satellite, led, file_flash,
           battery, servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7,
           file1, file2, config.lequad_config),
    saccade_controller_(flow_front, flow_back, ahrs, position_estimation, state),
    flow_front_(flow_front), flow_back_(flow_back),
    gps_mocap_(mavlink_communication.message_handler()),
    ahrs_ekf_mocap_(mavlink_communication.message_handler(), ahrs_ekf)
{
    // Init gps_mocap
    gps_mocap_.init();
    ahrs_ekf_mocap_.init();

    init_saccade();
    init_camera();
}

// -------------------------------------------------------------------------
// Saccade controller
// -------------------------------------------------------------------------
bool LEQuad_merk::init_saccade(void)
{
    bool ret = true;

    // Module
    saccade_controller_.init();

    // DOWN telemetry
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.gain_,               "WT_FCT_GAIN"   );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.threshold_,          "WT_FCT_THRSOLD");
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.goal_direction_,     "GL_DIRECTION"  );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.pitch_,              "PITCH_CONTROL" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.intersaccade_time_,  "INTSAC_TIME" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.altitude_value_,     "ALTI_VAL" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.velocity_value_,  "INTSAC_VEL" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.altitude_pid_.p_gain,    "ALT_PID_KP" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.altitude_pid_.clip_min,  "ALT_PID_MIN" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.altitude_pid_.clip_max,  "ALT_PID_MAX" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.cad_filter_,  "CAD_FILTER" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.can_filter_,  "CAN_FILTER" );

    // ret &= onboard_parameters->add_parameter_float(&saccade_controller_.altitude_pid_.


    // Task
    ret &= scheduler.add_task(4000, (Scheduler_task::task_function_t)&task_saccade_controller_update, (Scheduler_task::task_argument_t)&saccade_controller_);

    ret &= mavlink_communication.add_msg_send(MAVLINK_MSG_ID_DEBUG_VECT, 100000,(Mavlink_communication::send_msg_function_t)&saccade_telemetry_send_vector, &saccade_controller_);    
    return ret;
}

// -------------------------------------------------------------------------
// Cameras
// -------------------------------------------------------------------------

void flow_telemetry_send(const LEQuad_merk* LQm, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg);
void flow_telemetry_send(const LEQuad_merk* LQm, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{

    static uint8_t step = 0;
    step = (step + 1) % 2;
    // static const uint32_t N_points = 60;
    float of[60];
    char name[7];

    // switch (step)
    // {
    //   case 0:
    //       strcpy(name, "OF_0");
    //       for (uint32_t i = 0; i < 60; i++)
    //       {
    //           // left 0 to 59
    //           of[i] = LQm->flow_front_.of.x[i];
    //       }
    //   break;

    //   case 1:
    //       strcpy(name, "OF_1");
    //       for (uint32_t i = 0; i < N_points - 60; i++)
    //       {
    //           // left 60 to 78
    //           of[i] = LQm->flow_front_.of.x[i + 60];
    //       }
    //       for (uint32_t i = 0; i < 120-N_points ; i++)
    //       {
    //           // right 0 to 40
    //           of[i + N_points - 60] = LQm->flow_back_.of.x[i];
    //       }
    //   break;

    //   case 2:
    //       strcpy(name, "OF_2");
    //       for (uint32_t i = 0; i < 2 * N_points - 120 ; i++)
    //       {
    //           // right 41 to 78
    //           of[i] = LQm->flow_back_.of.x[i + 120 - N_points];
    //       }

    //       // for (uint32_t i = 0; i < 2; i++)
    //       // {

    //       //     of[i + 2 * N_points - 120] = 0.0f;//ahrs.angular_speed[2];
    //       // }

    //       for (uint32_t i = 0; i < 180 - 2 * N_points; i++)
    //       {

    //           of[i + 2 * N_points - 120] = 0.0f;
    //       }
    //   break;
    // }
    switch (step)
    {
      case 0:
          strcpy(name, "OF_0");
          for (uint32_t i = 0; i < 60; i++)
          {
              // left 0 to 59
              of[i] = LQm->flow_front_.of.x[i];
          }
      break;

      case 1:
          strcpy(name, "OF_1");
          for (uint32_t i = 0; i < 60 ; i++)
          {
              // right 0 to 40
              of[i] = LQm->flow_back_.of.x[i];
          }
      break;
    }

        mavlink_msg_big_debug_vect_pack(  mavlink_stream->sysid(),
                                      mavlink_stream->compid(),
                                      msg,
                                                  name,
                                      time_keeper_get_us(),
                                      of );
}

bool tasks_flow(LEQuad_merk* LQm);
bool tasks_flow(LEQuad_merk* LQm)
{
    bool success = true;

    success &= LQm->flow_front_.update();
    success &= LQm->flow_back_.update();

    return success;
}

bool LEQuad_merk::init_camera(void)
{
    bool ret = true;


    // Task
    // ret &= mavlink_communication.add_msg_send(MAVLINK_MSG_ID_OPTICAL_FLOW, 10000,(Mavlink_communication::send_msg_function_t)&flow_telemetry_send, this);
    ret &= scheduler.add_task(4000, (Scheduler_task::task_function_t)&tasks_flow,   (Scheduler_task::task_argument_t)this);
    

    return ret;
}


// -------------------------------------------------------------------------
// Main task
// -------------------------------------------------------------------------
bool LEQuad_merk::main_task(void)
{
    // Update estimation
    imu.update();
    ahrs_ekf.update();
    position_estimation.update();

    // Do control
    if (state.is_armed())
    {
        if (state.is_auto())
        {
            // Copy paste control from saccade controller
            controls.rpy[X]   = 0.0f;
            controls.rpy[Y]   = 0.0f;
            // controls.rpy[Z]   = 0.0f;
            controls.thrust   = -0.25f;
            controls.tvel[X]  = saccade_controller_.velocity_command_.xyz[X];
            controls.tvel[Y]  = saccade_controller_.velocity_command_.xyz[Y];
            controls.tvel[Z]  = saccade_controller_.velocity_command_.xyz[Z];
            controls.theading = saccade_controller_.attitude_command_.rpy[YAW];

            controls.control_mode = VELOCITY_COMMAND_MODE;
            controls.yaw_mode     = YAW_ABSOLUTE;

            // Do control
            stabilisation_copter_cascade_stabilise(&stabilisation_copter);
            servos_mix_quadcopter_diag_update(&servo_mix);
        }
        else if (state.is_guided())
        {
            manual_control.get_velocity_vector(&controls);

            // Copy paste control from saccade controller
            controls.tvel[Z] = saccade_controller_.velocity_command_.xyz[Z];
            controls.theading = saccade_controller_.movement_direction_;

            controls.control_mode = VELOCITY_COMMAND_MODE;
            controls.yaw_mode     = YAW_RELATIVE;
            // controls.yaw_mode     = YAW_ABSOLUTE;

            stabilisation_copter_cascade_stabilise(&stabilisation_copter);
            servos_mix_quadcopter_diag_update(&servo_mix);

            saccade_controller_.saccade_state_ = PRESACCADE;

            saccade_controller_.is_time_initialized_ = false;
        }
        else if (state.is_stabilize())
        {
            manual_control.get_velocity_vector(&controls);

            controls.control_mode = VELOCITY_COMMAND_MODE;
            controls.yaw_mode     = YAW_RELATIVE;

            stabilisation_copter_cascade_stabilise(&stabilisation_copter);
            servos_mix_quadcopter_diag_update(&servo_mix);

            saccade_controller_.saccade_state_ = PRESACCADE;

            saccade_controller_.is_time_initialized_ = false;

        }
        else if (state.is_manual())
        {
            manual_control.get_control_command(&controls);

            controls.control_mode = ATTITUDE_COMMAND_MODE;
            controls.yaw_mode     = YAW_RELATIVE;

            stabilisation_copter_cascade_stabilise(&stabilisation_copter);
            servos_mix_quadcopter_diag_update(&servo_mix);

            saccade_controller_.saccade_state_ = PRESACCADE;

            saccade_controller_.is_time_initialized_ = false;
        }
        else
        {
            servo_0.failsafe();
            servo_1.failsafe();
            servo_2.failsafe();
            servo_3.failsafe();
        }
    }
    else
    {
        servo_0.failsafe();
        servo_1.failsafe();
        servo_2.failsafe();
        servo_3.failsafe();
    }

    return true;
}
