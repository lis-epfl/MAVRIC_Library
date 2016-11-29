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
 * \file dynamic_model_quad_diag.cpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *
 * \brief Simulated dynamics of a quadcopter in diag configuration
 *
 ******************************************************************************/


#include "simulation/dynamic_model_quad_diag.hpp"
#include "sensing/ins.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
}



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Dynamic_model_quad_diag::Dynamic_model_quad_diag(Servo& servo_rear_left,
                                                 Servo& servo_front_left,
                                                 Servo& servo_front_right,
                                                 Servo& servo_rear_right,
                                                 dynamic_model_quad_diag_conf_t config):
    servo_front_right_(servo_front_right),
    servo_front_left_(servo_front_left),
    servo_rear_right_(servo_rear_right),
    servo_rear_left_(servo_rear_left),
    config_(config),
    rotorspeeds_(std::array<float,4>{{0.0f, 0.0f, 0.0f, 0.0f}}),
    torques_bf_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    rates_bf_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    lin_forces_bf_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    acc_bf_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    vel_bf_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    vel_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    attitude_(quat_t{1.0f, {0.0f, 0.0f, 0.0f}}),
    last_update_us_(0.0f),
    dt_s_(0.004f)
{
    // Init local position
    local_position_[0]  = 0.0f;
    local_position_[1]  = 0.0f;
    local_position_[2]  = 0.0f;

    // Init global position
    coord_conventions_local_to_global_position(local_position_, INS::origin(), global_position_);
}


bool Dynamic_model_quad_diag::update(void)
{
    int32_t i;
    quat_t qtmp1, qvel_bf, qed;
    const quat_t up     = { 0.0f, {UPVECTOR_X, UPVECTOR_Y, UPVECTOR_Z} };

    // Update timing
    float now       = time_keeper_get_us();
    dt_s_           = (now - last_update_us_) / 1000000.0f;

    // Do nothing if updated too often
    if (dt_s_ < 0.001f)
    {
        return true;
    }

    last_update_us_ = now;

    // Clip dt if too large, this is not realistic but the simulation will be more precise
    if (dt_s_ > 0.1f)
    {
        dt_s_ = 0.1f;
    }

    // compute torques and forces based on servo commands
    forces_from_servos();

    // integrate torques to get simulated gyro rates (with some damping)
    rates_bf_[0] = maths_clip((1.0f - 0.1f * dt_s_) * rates_bf_[0] + dt_s_ * torques_bf_[0] / config_.roll_pitch_momentum, 10.0f);
    rates_bf_[1] = maths_clip((1.0f - 0.1f * dt_s_) * rates_bf_[1] + dt_s_ * torques_bf_[1] / config_.roll_pitch_momentum, 10.0f);
    rates_bf_[2] = maths_clip((1.0f - 0.1f * dt_s_) * rates_bf_[2] + dt_s_ * torques_bf_[2] / config_.yaw_momentum, 10.0f);


    qtmp1.s = 0.0f;
    for (i = 0; i < 3; i++)
    {
        qtmp1.v[i] = 0.5f * rates_bf_[i];
    }

    // apply step rotation
    qed = quaternions_multiply(attitude_, qtmp1);

    // TODO: correct this formulas when using the right scales
    attitude_.s = attitude_.s + qed.s * dt_s_;
    attitude_.v[0] += qed.v[0] * dt_s_;
    attitude_.v[1] += qed.v[1] * dt_s_;
    attitude_.v[2] += qed.v[2] * dt_s_;

    attitude_           = quaternions_normalise(attitude_);
    quat_t up_vec       = quaternions_global_to_local(attitude_, up);

    // velocity and position integration

    // check altitude - if it is lower than ground, clamp everything (this is in NED, assuming negative altitude)
    if (local_position_[Z] > -0.001)
    {
        // vel_[Z] = 0.0f;
        local_position_[Z] = 0.0f;

        // Add resistive force towards ground, proportionnal to distance to ground
        for (i = 0; i < 3; i++)
        {
            lin_forces_bf_[i] += up_vec.v[i] * config_.total_mass * config_.gravity + local_position_[Z] * 1000.0f;
        }

        // slow down... (will make velocity slightly inconsistent until next update cycle, but shouldn't matter much)
        for (i = 0; i < 3; i++)
        {
            vel_bf_[i] = 0.95f * vel_bf_[i];
        }

        //upright
        rates_bf_[0] =  up_vec.v[1];
        rates_bf_[1] =  - up_vec.v[0];
        rates_bf_[2] = 0;
    }

    for (i = 0; i < 3; i++)
    {
        qtmp1.v[i] = vel_[i];
    }
    qtmp1.s = 0.0f;
    qvel_bf = quaternions_global_to_local(attitude_, qtmp1);

    for (i = 0; i < 3; i++)
    {
        vel_bf_[i] = qvel_bf.v[i];

        // this is the "clean" acceleration without gravity
        acc_bf_[i] = lin_forces_bf_[i] / config_.total_mass - up_vec.v[i] * config_.gravity;

        vel_bf_[i] = vel_bf_[i] + acc_bf_[i] * dt_s_;
    }

    // calculate velocity in global frame
    // vel = qe *vel_bf * qe - 1
    qvel_bf.s = 0.0f; qvel_bf.v[0] = vel_bf_[0]; qvel_bf.v[1] = vel_bf_[1]; qvel_bf.v[2] = vel_bf_[2];
    qtmp1 = quaternions_local_to_global(attitude_, qvel_bf);
    vel_[0] = qtmp1.v[0]; vel_[1] = qtmp1.v[1]; vel_[2] = qtmp1.v[2];

    for (i = 0; i < 3; i++)
    {
        local_position_[i] = local_position_[i] + vel_[i] * dt_s_;
    }

    coord_conventions_local_to_global_position(local_position_, INS::origin(), global_position_);

    return true;
}


const float& Dynamic_model_quad_diag::last_update_us(void) const
{
    return last_update_us_;
}


const std::array<float, 3>& Dynamic_model_quad_diag::acceleration_bf(void) const
{
    return acc_bf_;
}


const std::array<float, 3>& Dynamic_model_quad_diag::velocity_lf(void) const
{
    return vel_;
}


const local_position_t& Dynamic_model_quad_diag::position_lf(void) const
{
    return local_position_;
}


const global_position_t& Dynamic_model_quad_diag::position_gf(void) const
{
    return global_position_;
}


const std::array<float, 3>& Dynamic_model_quad_diag::angular_velocity_bf(void) const
{
    return rates_bf_;
}


const quat_t& Dynamic_model_quad_diag::attitude(void) const
{
    return attitude_;
}



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Dynamic_model_quad_diag::forces_from_servos(void)
{
    float motor_command[4];
    float rotor_lifts[4], rotor_drags[4], rotor_inertia[4];
    float ldb;
    quat_t wind_gf  = {};
    wind_gf.s       = 0;
    wind_gf.v[0]    = config_.wind_x;
    wind_gf.v[1]    = config_.wind_y;
    wind_gf.v[2]    = 0.0f;

    quat_t wind_bf  =  quaternions_global_to_local(attitude_, wind_gf);

    float sqr_lateral_airspeed = SQR(vel_bf_[0] + wind_bf.v[0]) + SQR(vel_bf_[1] + wind_bf.v[1]);
    float lateral_airspeed = sqrt(sqr_lateral_airspeed);

    float old_rotor_speed;

    motor_command[0] = servo_rear_left_.read()   - config_.rotor_rpm_offset;
    motor_command[1] = servo_front_left_.read()  - config_.rotor_rpm_offset;
    motor_command[2] = servo_front_right_.read() - config_.rotor_rpm_offset;
    motor_command[3] = servo_rear_right_.read()  - config_.rotor_rpm_offset;

    for (int32_t i = 0; i < 4; i++)
    {

        // temporarily save old rotor speeds
        old_rotor_speed = rotorspeeds_[i];
        // estimate rotor speeds by low - pass filtering
        //rotorspeeds_[i] = (config_.rotor_lpf) * rotorspeeds_[i] + (1.0f - config_.rotor_lpf) * (motor_command[i] * config_.rotor_rpm_gain);
        rotorspeeds_[i] = (motor_command[i] * config_.rotor_rpm_gain);

        // calculate torque created by rotor inertia
        rotor_inertia[i] = (rotorspeeds_[i] - old_rotor_speed) / dt_s_ * config_.rotor_momentum;

        ldb = lift_drag_base(rotorspeeds_[i], sqr_lateral_airspeed, -vel_bf_[Z]);

        rotor_lifts[i] = ldb * config_.rotor_cl;
        rotor_drags[i] = ldb * config_.rotor_cd;
    }

    float mpos_x = config_.rotor_arm_length / 1.4142f;
    float mpos_y = config_.rotor_arm_length / 1.4142f;

    // torque around x axis (roll)
    torques_bf_[ROLL] = ((rotor_lifts[0]  + rotor_lifts[1])
                       - (rotor_lifts[2]  + rotor_lifts[3])) * mpos_y;

    // torque around y axis (pitch)
    torques_bf_[PITCH] = ((rotor_lifts[1]  + rotor_lifts[2])
                        - (rotor_lifts[0]  + rotor_lifts[3])) *  mpos_x;

    torques_bf_[YAW] = - (config_.motor_dir[0] * (10.0f * rotor_drags[0] + rotor_inertia[0])
                        + config_.motor_dir[1] * (10.0f * rotor_drags[1] + rotor_inertia[1])
                        + config_.motor_dir[2] * (10.0f * rotor_drags[2] + rotor_inertia[2])
                        + config_.motor_dir[3] * (10.0f * rotor_drags[3] + rotor_inertia[3])) * config_.rotor_diameter;

    lin_forces_bf_[X] = - (vel_bf_[X] - wind_bf.v[0]) * lateral_airspeed * config_.vehicle_drag;
    lin_forces_bf_[Y] = - (vel_bf_[Y] - wind_bf.v[1]) * lateral_airspeed * config_.vehicle_drag;
    lin_forces_bf_[Z] = - (rotor_lifts[0] + rotor_lifts[1] + rotor_lifts[2] + rotor_lifts[3]);
}


float Dynamic_model_quad_diag::lift_drag_base(float rpm, float sqr_lat_airspeed, float axial_airspeed)
{
    if (rpm < 0.1f)
    {
        return 0.0f;
    }
    float mean_vel = config_.rotor_diameter * PI * rpm / 60.0f;
    float exit_vel = rpm / 60.0f * config_.rotor_pitch;

    return (0.5f * config_.air_density * (mean_vel * mean_vel + sqr_lat_airspeed) * config_.rotor_foil_area  * (1.0f - (axial_airspeed / exit_vel)));
}
