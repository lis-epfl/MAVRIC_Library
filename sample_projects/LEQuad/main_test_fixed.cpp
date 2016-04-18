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
    logfile << "x,y,z,vx,vy,vz,r,p,y,pv,aoa1,aoa2,aoa3,aoa4,pt" << std::endl;
    // logfile.close();

    Pwm_dummy pwm;
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

    servo_motor.write(100.0f);
    //servo_motor.write(-0.4f);


    local_position_t position;
    std::array<float, 3> velocity;
    std::array<float, 3> ang_velocity;
    quat_t orientation;
    float roll, pitch, yaw;

    //Begin simulation
    time_keeper_init();
    float t = time_keeper_get_s();
    for (uint32_t i = 0; i < 1000; i++)
    {
        position = model.position_lf();
        velocity = model.velocity_lf();
        ang_velocity = model.angular_velocity_bf();
        orientation = model.attitude();
        float q0 = orientation.s;
        float q1 = orientation.v[0];
        float q2 = orientation.v[1];
        float q3 = orientation.v[2];
        printf("Position: (%f, %f, %f)\n", position.pos[0], position.pos[1], position.pos[2]);
        roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
        pitch = asin(2*(q0*q2-q3*q1));
        yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
        printf("Roll: %f Pitch: %f Yaw: %f \n", roll, pitch, yaw);
        printf("Velocity: (%f, %f, %f)\n", velocity[0], velocity[1], velocity[2]);
        printf("Angular velocity: (%f, %f, %f)\n\n", ang_velocity[0], ang_velocity[1], ang_velocity[2]);
        //std::cin.ignore();

        // write to log file
        logfile << position.pos[0] << "," << position.pos[1] << "," << position.pos[2] << ",";
        logfile << velocity[0] << "," << velocity[1] << "," << velocity[2] << ",";
        logfile << roll << "," << pitch << "," << yaw << "," << ang_velocity[2] << ",";

        while(time_keeper_get_s() - t < 0.004f){}
        t = time_keeper_get_s();
        model.update();
    }
    return 0;
}
