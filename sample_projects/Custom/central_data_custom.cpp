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
 * \file central_data.c
 *
 * \author MAV'RIC Team
 *
 * \brief Place where the central data is stored and initialized
 *
 ******************************************************************************/


#include "sample_projects/Custom/central_data_custom.hpp"

extern "C"
{
#include "util/print_util.h"
}


Central_data_custom::Central_data_custom(Imu& imu, Barometer& barometer, Gps& gps,
                          Sonar& sonar, Serial& serial_mavlink, Satellite& satellite,
                          Led& led, File& file_flash, Battery& battery,
                          Servo& servo_0, Servo& servo_1, Servo& servo_2, Servo& servo_3,
                          Servo& servo_4, Servo& servo_5, Servo& servo_6, Servo& servo_7,
                          File& file1, File& file2,
                          Flow& flow_1, Flow& flow_2,
                          const conf_t& config):
    LEQuad_dronedome(imu, barometer, gps, sonar, serial_mavlink, satellite, led, file_flash, battery,
                 servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7, file1, file2, config),
    flow_1_(flow_1),
    flow_2_(flow_2),
    saccade_controller_(flow_1_, flow_2_, ahrs, altitude_,position_estimation,saccade_controller_default_config())
{}


bool Central_data_custom::init(void)
{
    bool init_success = true;

    // Init base class
    init_success &= LEQuad_dronedome::init();

    // -------------------------------------------------------------------------
    // Init servo mixing
    // -------------------------------------------------------------------------
    servos_mix_quadcopter_diag_conf_t my_servos_mix_quadcopter_diag_default_config;

    my_servos_mix_quadcopter_diag_default_config.motor_front_right_dir              = CW;
    my_servos_mix_quadcopter_diag_default_config.motor_front_left_dir               = CCW;
    my_servos_mix_quadcopter_diag_default_config.motor_rear_right_dir               = CCW;
    my_servos_mix_quadcopter_diag_default_config.motor_rear_left_dir                = CW;
    my_servos_mix_quadcopter_diag_default_config.min_thrust                         = -0.9f;
    my_servos_mix_quadcopter_diag_default_config.max_thrust                         = 1.0f;



    servos_mix_quadcotper_diag_init(&servo_mix,
                                          my_servos_mix_quadcopter_diag_default_config,
                                          &command.torque,
                                          &command.thrust,
                                          &servo_0,
                                          &servo_1,
                                          &servo_2,
                                          &servo_3);

    velocity_controller_copter_conf_t my_velocity_controller_copter_default_config;


    my_velocity_controller_copter_default_config.thrust_hover_point                    = -0.1f;
    // -----------------------------------------------------------------
    // ------ X PID -------------------------------------------------
    // -----------------------------------------------------------------
    my_velocity_controller_copter_default_config.pid_config[X]                         = {};
    my_velocity_controller_copter_default_config.pid_config[X].p_gain                  = 0.2f;
    my_velocity_controller_copter_default_config.pid_config[X].clip_min                = -0.5f;
    my_velocity_controller_copter_default_config.pid_config[X].clip_max                = 0.5f;
    my_velocity_controller_copter_default_config.pid_config[X].integrator              = {};
    my_velocity_controller_copter_default_config.pid_config[X].integrator.gain         = 0.1f;
    my_velocity_controller_copter_default_config.pid_config[X].integrator.clip_pre     = 1.0f;
    my_velocity_controller_copter_default_config.pid_config[X].integrator.accumulator  = 0.0f;
    my_velocity_controller_copter_default_config.pid_config[X].integrator.clip         = 0.1f;
    my_velocity_controller_copter_default_config.pid_config[X].differentiator          = {};
    my_velocity_controller_copter_default_config.pid_config[X].differentiator.gain     = 0.0f;
    my_velocity_controller_copter_default_config.pid_config[X].differentiator.previous = 0.0f;
    my_velocity_controller_copter_default_config.pid_config[X].differentiator.clip     = 0.0f;
    my_velocity_controller_copter_default_config.pid_config[X].soft_zone_width         = 0.2f;
    // -----------------------------------------------------------------
    // ------ Y PID ------------------------------------------------
    // -----------------------------------------------------------------
    my_velocity_controller_copter_default_config.pid_config[Y]                         = {};
    my_velocity_controller_copter_default_config.pid_config[Y].p_gain                  = 0.2f;
    my_velocity_controller_copter_default_config.pid_config[Y].clip_min                = -0.5f;
    my_velocity_controller_copter_default_config.pid_config[Y].clip_max                = 0.5f;
    my_velocity_controller_copter_default_config.pid_config[Y].integrator              = {};
    my_velocity_controller_copter_default_config.pid_config[Y].integrator.gain         = 0.1f;
    my_velocity_controller_copter_default_config.pid_config[Y].integrator.clip_pre     = 1.0f;
    my_velocity_controller_copter_default_config.pid_config[Y].integrator.accumulator  = 0.0f;
    my_velocity_controller_copter_default_config.pid_config[Y].integrator.clip         = 0.1f;
    my_velocity_controller_copter_default_config.pid_config[Y].differentiator          = {};
    my_velocity_controller_copter_default_config.pid_config[Y].differentiator.gain     = 0.0f;
    my_velocity_controller_copter_default_config.pid_config[Y].differentiator.previous = 0.0f;
    my_velocity_controller_copter_default_config.pid_config[Y].differentiator.clip     = 0.0f;
    my_velocity_controller_copter_default_config.pid_config[Y].soft_zone_width         = 0.2f;
    // ---------------------------------------------------------------------
    // ------ Z PID ---------------------------------------------------
    // ---------------------------------------------------------------------
    my_velocity_controller_copter_default_config.pid_config[Z]                         = {};
    my_velocity_controller_copter_default_config.pid_config[Z].p_gain                  = 0.20f;
    my_velocity_controller_copter_default_config.pid_config[Z].clip_min                = -0.9f;
    my_velocity_controller_copter_default_config.pid_config[Z].clip_max                = 0.65f;
    my_velocity_controller_copter_default_config.pid_config[Z].integrator              = {};
    my_velocity_controller_copter_default_config.pid_config[Z].integrator.gain         = 0.002f;
    my_velocity_controller_copter_default_config.pid_config[Z].integrator.clip_pre     = 1.0f;
    my_velocity_controller_copter_default_config.pid_config[Z].integrator.accumulator  = 0.0f;
    my_velocity_controller_copter_default_config.pid_config[Z].integrator.clip         = 0.2f;
    my_velocity_controller_copter_default_config.pid_config[Z].differentiator          = {};
    my_velocity_controller_copter_default_config.pid_config[Z].differentiator.gain     = 0.04f;
    my_velocity_controller_copter_default_config.pid_config[Z].differentiator.previous = 0.0f;
    my_velocity_controller_copter_default_config.pid_config[Z].differentiator.clip     = 0.04f;
    my_velocity_controller_copter_default_config.pid_config[Z].soft_zone_width         = 0.2f;

    velocity_controller_copter_init(&velocity_controller,
                                    my_velocity_controller_copter_default_config,
                                    &ahrs,
                                    &position_estimation,
                                    &command.velocity,
                                    &command.attitude,
                                    &command.thrust);

    time_keeper_delay_ms(50);

    //--------------------------------------------------------------------------
    // Init saccade controller
    //--------------------------------------------------------------------------
    saccade_controller_.init();

    // A METTRE ICI??
    // gps_mocap.init();


    return init_success;
}
