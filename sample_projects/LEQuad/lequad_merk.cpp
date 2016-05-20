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
                         Flow& flow1, Flow& flow2, const conf_t& config):
    LEQuad(imu, barometer, gps_mocap_, sonar, serial_mavlink, satellite, led, file_flash,
           battery, servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7,
           file1, file2, config.lequad_config),
    gps_mocap_(mavlink_communication.message_handler()),
    saccade_controller_(flow1, flow2, ahrs, position_estimation)
{
    // Init gps_mocap
    gps_mocap_.init();

    init_saccade();
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
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.altitude_pid_.p_gain,    "ALT_PID_KP" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.altitude_pid_.clip_min,  "ALT_PID_MIN" );
    ret &= mavlink_communication.onboard_parameters().add_parameter_float(&saccade_controller_.altitude_pid_.clip_max,  "ALT_PID_MAX" );
    // ret &= onboard_parameters->add_parameter_float(&saccade_controller_.altitude_pid_.


    // Task
    ret &= scheduler.add_task(4000, (Scheduler_task::task_function_t)&task_saccade_controller_update, (Scheduler_task::task_argument_t)&saccade_controller_);

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
            controls.rpy[Z]   = 0.0f;
            controls.thrust   = -0.26f;
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

            controls.control_mode = VELOCITY_COMMAND_MODE;
            controls.yaw_mode     = YAW_RELATIVE;
            // controls.yaw_mode     = YAW_ABSOLUTE;

            stabilisation_copter_cascade_stabilise(&stabilisation_copter);
            servos_mix_quadcopter_diag_update(&servo_mix);
        }
        else if (state.is_stabilize())
        {
            manual_control.get_velocity_vector(&controls);

            controls.control_mode = VELOCITY_COMMAND_MODE;
            controls.yaw_mode     = YAW_RELATIVE;

            stabilisation_copter_cascade_stabilise(&stabilisation_copter);
            servos_mix_quadcopter_diag_update(&servo_mix);
        }
        else if (state.is_manual())
        {
            manual_control.get_control_command(&controls);

            controls.control_mode = ATTITUDE_COMMAND_MODE;
            controls.yaw_mode     = YAW_RELATIVE;

            stabilisation_copter_cascade_stabilise(&stabilisation_copter);
            servos_mix_quadcopter_diag_update(&servo_mix);
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
