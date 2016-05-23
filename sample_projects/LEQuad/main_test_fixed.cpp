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
 * \file main.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief Main file
 *
 ******************************************************************************/

#include "simulation/dynamic_model_fixed_wing.hpp"
#include "simulation/wing_model.hpp"
#include "drivers/servo.hpp"
#include "hal/dummy/pwm_dummy.hpp"
#include "util/coord_conventions.h"
#include "hal/common/time_keeper.hpp"

#include <iostream>
#include <fstream>

extern "C"
{
#include "util/print_util.h"
}

std::ofstream logfile;

int main(int argc, char** argv)
{
    // Create log file

    logfile.open("log.csv");
    //logfile << "t,x,y,z,vx,vy,vz,roll,pitch,yaw" << std::endl;
    logfile << "rate" << std::endl;
    Wing_model left_drift(0.0f,quaternions_create(1.0f/sqrt(2.0f), 1.0f/sqrt(2.0f), 0.0f, 0.0f), -0.1780f, -0.30f, -0.04f, 0.015f, 0.14f,0);
    Wing_model right_drift(0.0f,quaternions_create(1.0f/sqrt(2.0f), 1.0f/sqrt(2.0f), 0.0f, 0.0f), -0.1780f, 0.30f, -0.04f, 0.015f, 0.14f,0);
    wing_model_forces_t left_force;
    wing_model_forces_t right_force;
    Wing_model left_flap(0.0f,quaternions_create(1.0f, 0.0f, 0.0f, 0.0f), -0.035f+0.0280f, -0.205f, 0.004f, 0.12f, 0.3f,1);
    Wing_model right_flap(0.0f,quaternions_create(1.0f, 0.0f, 0.0f, 0.0f), -0.035f+0.0280f, 0.205f, 0.004f, 0.12f, 0.3f,1);
    wing_model_forces_t left_flap_force;
    wing_model_forces_t right_flap_force;
    float wind[3]={0.0f, 0.0f, 0.0f}, rates[3]={0.0f, 0.0f, -10.0f};
    float angle = 10.0f;
    /*for(int i=-90; i<=90; i++)
    {
    wind[0] = -cos(-10.0f/180.0f*PI);
    wind[1] = sin(-10.0f/180.0f*PI);
     printf("%f, %f\n", wind[0], wind[2]);*/
    while(1)
    {
      wind[0] = -cos(angle/180.0f*PI)*10.0f;
      wind[1] = sin(angle/180.0f*PI)*10.0f;
      left_force = left_drift.compute_forces(wind, rates);
      right_force = right_drift.compute_forces(wind, rates);
      left_flap_force = left_flap.compute_forces(wind, rates);
      right_flap_force = right_flap.compute_forces(wind, rates);
      rates[2]+=left_force.torque[YAW]+right_force.torque[YAW]+left_flap_force.torque[YAW]+right_flap_force.torque[YAW];
      angle+=rates[2]*0.18f/PI;
      logfile << angle << std::endl;
      //logfile << i << ", " << left_force.torque[YAW]+right_force.torque[YAW]+left_flap_force.torque[YAW]+right_flap_force.torque[YAW] << std::endl;
    }
    // logfile.close();

    /*Pwm_dummy pwm;
    servo_conf_t config;
    config.trim = 0.0f;
    config.min = -1.0f;
    config.max = 1.0f;
    config.failsafe = 0.0f;
    config.repeat_freq = 1;
    config.pulse_center_us = 100;
    config.pulse_magnitude_us = 10;
    //Create servos
    Servo servo_flap_left(pwm, config);
    Servo servo_flap_right(pwm, config);
    Servo servo_motor(pwm, config);
    //Create dynamic model
    Dynamic_model_fixed_wing model(servo_motor, servo_flap_left, servo_flap_right);

    servo_motor.write(-1.0f);
    //servo_motor.write(-0.4f);

    local_position_t position;
    std::array<float, 3> velocity;
    std::array<float, 3> ang_velocity;
    quat_t orientation;
    float roll, pitch, yaw;

    //Begin simulation
    time_keeper_init();
    float t = time_keeper_get_s();
    float start = t;
    model.set_position(0.0f, 0.0f, -100.0f);
    model.set_speed(10.0f, 0.0f, 0.0f);
    for (uint32_t i = 0; i < 100000; i++)
    {
        //if(i%100000==0) printf("%d/100\n",i/10000);
        //printf("-------------------------\n");
        position = model.position_lf();
        velocity = model.velocity_lf();
        ang_velocity = model.angular_velocity_bf();
        orientation = model.attitude();
        float q0 = orientation.s;
        float q1 = orientation.v[0];
        float q2 = orientation.v[1];
        float q3 = orientation.v[2];
        roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
        pitch = asin(2*(q0*q2-q3*q1));
        yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
        //printf("--> %f\n", yaw/PI*180);

        // write to log file
        logfile << t-start << "," << position.pos[0] << "," << position.pos[1] << "," << position.pos[2] << ",";
        logfile << velocity[0] << "," << velocity[1] << "," << velocity[2] << ",";
        logfile << roll << "," << pitch << "," << yaw << std::endl;

        while(time_keeper_get_s() - t < 0.004f){}
        t = time_keeper_get_s();
        model.update();
    }*/
    return 0;
}
