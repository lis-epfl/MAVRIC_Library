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
 * \file hexhog.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief MAV class
 *
 ******************************************************************************/


#ifndef HEXHOG_HPP_
#define HEXHOG_HPP_

#include <stdbool.h>
#include <stdint.h>

#include "sample_projects/LEQuad/lequad.hpp"
#include "control/servos_mix_6dof.hpp"

#include "drivers/px4flow_i2c.hpp"
#include "hal/common/time_keeper.hpp"

#include "sensing/ins.hpp"
#include "sensing/ins_kf.hpp"


/**
 * \brief MAV class
 */
class Hexhog: public LEQuad
{
public:
    /**
     * \brief   Configuration structure
     */
     struct conf_t
    {
        LEQuad::conf_t lequad_config;
        Servos_mix_6dof<6>::conf_t servos_mix_config;
    };

    /**
     * \brief   Default configuration
     *
     * \param   sysid       System id (default value = 1)
     *
     * \return  Config structure
     */
    static inline conf_t default_config(uint8_t sysid = 1);


    /**
     * \brief   Constructor
     */
    Hexhog( Imu& imu,
            Barometer& barometer,
            Gps& gps,
            Sonar& sonar,
            Serial& serial_mavlink,
            Satellite& satellite,
            Led& led,
            File& file_flash,
            Battery& battery,
            Servo& servo_0,
            Servo& servo_1,
            Servo& servo_2,
            Servo& servo_3,
            Servo& servo_4,
            Servo& servo_5,
            Servo& servo_6,
            Servo& servo_7,
            File& file1,
            File& file2,
            I2c& flow_i2c,
            const conf_t& config = default_config()):
        LEQuad(imu, barometer, gps, sonar, serial_mavlink, satellite, led, file_flash, battery,
              servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7,
              file1, file2, config.lequad_config),
        servos_mix_(command.torque, command.thrust3D,
                    std::array<Servo*, 6>{{&servo_0, &servo_1, &servo_2, &servo_3, &servo_4, &servo_5}},
                    config.servos_mix_config),
        bottom_flow_(flow_i2c, Px4flow_i2c::default_config()),
        ins_kf_(gps, barometer, sonar, bottom_flow_, ahrs)
    {
        init_flow();
        init_ins();
        init_controllers();
    };


protected:
    bool init_flow(void);
    bool init_ins(void);
    bool init_controllers(void);

    virtual bool main_task(void);


    Servos_mix_6dof<6> servos_mix_;

    Px4flow_i2c bottom_flow_;

    INS_kf ins_kf_;
    pid_controller_t altitude_pid_;

    velocity_controller_copter_t velocity_controller_;
    attitude_controller_t attitude_controller_;
};


void flow_telemetry_send(const Px4flow_i2c* flow, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
  mavlink_msg_optical_flow_pack( mavlink_stream->sysid(),
                                 mavlink_stream->compid(),
                                 msg,
                                 time_keeper_get_us(),
                                 0,       // uint8_t sensor_id,
                                 10.0f * flow->flow_x(),
                                 10.0f * flow->flow_y(),
                                 flow->velocity_x(),
                                 flow->velocity_y(),
                                 flow->flow_quality(),
                                 flow->ground_distance());
}


bool Hexhog::init_flow(void)
{
    bool ret = true;

    // DOWN telemetry
    ret &= mavlink_communication.add_msg_send(MAVLINK_MSG_ID_OPTICAL_FLOW,  50000, (Mavlink_communication::send_msg_function_t)&flow_telemetry_send, &bottom_flow_);

    // Task
    ret &= scheduler.add_task(20000, (Scheduler_task::task_function_t)&Px4flow_i2c::update_task, (Scheduler_task::task_argument_t)&bottom_flow_);
    // ret &= scheduler.add_task(10000, (Scheduler_task::task_function_t)&Px4flow_i2c::update_task, (Scheduler_task::task_argument_t)&bottom_flow_);

    return ret;
}


bool Hexhog::init_ins(void)
{
    bool ret = true;

    // DOWN telemetry
    ret &= mavlink_communication.add_msg_send(MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV,  50000, (Mavlink_communication::send_msg_function_t)&ins_telemetry_send, &ins_kf_);

    // Task
    // ret &= scheduler.add_task(4000, (Scheduler_task::task_function_t)&task_ins_kf_update, (Scheduler_task::task_argument_t)&ins_kf_);

    return ret;
}


bool Hexhog::init_controllers(void)
{
    bool ret = true;

    // Attitude
    attitude_controller_init(&attitude_controller_,
                             attitude_controller_default_config(),
                             &ahrs,
                             &command.attitude,
                             &command.rate,
                             &command.torque);

    // Velocity
    velocity_controller_copter_conf_t velocity_controller_config = velocity_controller_copter_default_config();
    velocity_controller_config.thrust_hover_point = 0.0f;
    velocity_controller_copter_init(&velocity_controller_,
                                    velocity_controller_config,
                                    &ahrs,
                                    &ins_kf_,
                                    &command.velocity,
                                    &command.attitude,
                                    &command.thrust);

    // Position
    pid_controller_conf_t pid_config = {};
    pid_config.p_gain   = 1.0f;
    pid_config.clip_min = -0.5f;
    pid_config.clip_max = 0.5f;
    pid_config.integrator.gain        = 0.0f;
    pid_config.integrator.accumulator = 0.0f;
    pid_config.integrator.clip_pre    = 0.0f;
    pid_config.integrator.clip        = 0.0f;
    pid_config.differentiator.gain      = 0.0f;
    pid_config.differentiator.previous  = 0.0f;
    pid_config.differentiator.clip      = 0.0f;
    pid_controller_init(&altitude_pid_, &pid_config);



    return ret;
}


bool Hexhog::main_task(void)
{
    // Update estimation
    imu.update();
    ahrs_ekf.update();
    ins_kf_.update();

    bool failsafe = false;

    // Do control
    if (state.is_armed())
    {
        switch (state.mav_mode().ctrl_mode())
        {
            case Mav_mode::GPS_NAV:
                manual_control.get_attitude_command(0.02f, &command.attitude, 1.0f);
                manual_control.get_velocity_command(&command.velocity, 1.0f);
                velocity_controller_copter_update(&velocity_controller_);

                command.thrust3D.xyz[Z] = command.thrust.thrust;
            break;

            case Mav_mode::POSITION_HOLD:
            case Mav_mode::VELOCITY:
                // Vertical velocity from altitude PID
                manual_control.get_velocity_command(&command.velocity, 1.0f);
                command.velocity.xyz[Z] = pid_controller_update(&altitude_pid_, (-0.5f - ins_kf_.position_lf()[Z]));

                // Run velocity control
                velocity_controller_copter_update(&velocity_controller_);

                // Attitude from remote
                manual_control.get_attitude_command(0.02f, &command.attitude, 1.0f);

                // Convert output to 3D thrust
                command.thrust3D.xyz[Z] = command.thrust.thrust;
            break;

            case Mav_mode::ATTITUDE:
                manual_control.get_velocity_command(&command.velocity, 1.0f);
                velocity_controller_copter_update(&velocity_controller_);

                manual_control.get_attitude_command(0.02f, &command.attitude, 1.0f);

                command.thrust3D.xyz[Z] = command.thrust.thrust;
            break;

            case Mav_mode::RATE:
                manual_control.get_attitude_command(0.02f, &command.attitude, 1.0f);
                manual_control.get_thrust_command(&command.thrust);
                command.thrust3D.xyz[Z] = command.thrust.thrust;
            break;

            default:
                failsafe = true;    // undefined behaviour -> failsafe
            break;
        }
    }
    else
    {
        failsafe = true;    // undefined behaviour -> failsafe
        command.attitude.quat = ahrs.qe;
        coord_conventions_rpy_from_quaternion(ahrs.qe, command.attitude.rpy);
    }

    // if behaviour defined, execute controller and mix; otherwise: set servos to failsafe
    if(!failsafe)
    {
        if (state.is_custom())
        {
            // Convert attitude command to lateral thrust
            command.thrust3D.xyz[Y]     = command.attitude.rpy[ROLL];
            command.thrust3D.xyz[X]     = - command.attitude.rpy[PITCH];

            // Level attitude
            command.attitude.rpy[ROLL]  = 0.0f;
            command.attitude.rpy[PITCH] = 0.0f;
            command.attitude.quat = coord_conventions_quaternion_from_rpy(command.attitude.rpy);
        }
        else
        {
            command.thrust3D.xyz[Y] = 0.0f;
            command.thrust3D.xyz[X] = 0.0f;
        }

        attitude_controller_.mode = ATTITUDE_CONTROLLER_MODE_DEFAULT;
        attitude_controller_update(&attitude_controller_);

        servos_mix_.update();
    }
    else
    {
        servo_0.failsafe();
        servo_1.failsafe();
        servo_2.failsafe();
        servo_3.failsafe();
        servo_4.failsafe();
        servo_5.failsafe();
    }

    return true;
}


Hexhog::conf_t Hexhog::default_config(uint8_t sysid)
{
    conf_t conf                                                = {};

    conf.lequad_config    = LEQuad::default_config();

    // Change the modes set from remote switches
    conf.lequad_config.remote_config.mode_config.safety_mode         = Mav_mode::RATE;
    conf.lequad_config.remote_config.mode_config.mode_switch_up      = Mav_mode::ATTITUDE;
    conf.lequad_config.remote_config.mode_config.mode_switch_middle  = Mav_mode::VELOCITY;
    conf.lequad_config.remote_config.mode_config.mode_switch_down    = Mav_mode::GPS_NAV;

    // Define correct servo mix
    conf.servos_mix_config = Servos_mix_6dof<6>::default_config();
    float s60 = 0.866025f;
    conf.servos_mix_config.mix = Mat<6, 6>({  0.0f,  1.0f,  1.0f,  1.0f, 0.0f, 1.0f,
                                               s60,  0.5f, -1.0f, -0.5f,  s60, 1.0f,
                                               s60, -0.5f,  1.0f, -0.5f, -s60, 1.0f,
                                              0.0f, -1.0f, -1.0f,  1.0f, 0.0f, 1.0f,
                                              -s60, -0.5f,  1.0f, -0.5f,  s60, 1.0f,
                                              -s60,  0.5f, -1.0f, -0.5f, -s60, 1.0f});

    return conf;
};

#endif /* HEXHOG_HPP_ */
