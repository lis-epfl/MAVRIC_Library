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
 * \file dynamic_model_quad_diag.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Simulated dynamics of a quadcopter in diag configuration
 *
 ******************************************************************************/


#ifndef DYNAMIC_MODEL_QUAD_DIAG_HPP_
#define DYNAMIC_MODEL_QUAD_DIAG_HPP_


#include "simulation/dynamic_model.hpp"
#include "drivers/servo.hpp"
#include "util/constants.hpp"

/**
 * \brief Configuration for quad dynamic model
 */
typedef struct
{
    float rotor_lpf;                    ///< Low-pass filtered response of the rotors to simulate inertia and lag
    float rotor_rpm_gain;               ///< Gain linking the command to rpm
    float rotor_rpm_offset;             ///< Offset to convert servo commands to rpm

    float rotor_cd;                     ///< Rotor drag coefficient
    float rotor_cl;                     ///< Rotor lift coefficient
    float rotor_diameter;               ///< Mean rotor diameter in m
    float rotor_foil_area;              ///< Rotor foil area

    float rotor_pitch;                  ///< Rotor pitch
    float total_mass;                   ///< Vehicle mass in kg
    float vehicle_drag;                 ///< Coefficient of drag of the whole vehicle
    float roll_pitch_momentum;          ///< Roll and pitch angular momentum of the vehicle
    float yaw_momentum;                 ///< Yaw angular momentum constants (assumed to be independent)

    float rotor_momentum;               ///< Angular momentum of the rotor (for rotor inertia)
    float rotor_arm_length;             ///< Distance between CoG and motor (in meter)

    float wind_x;                       ///< X component of wind in global frame in m/s
    float wind_y;                       ///< Y component of wind in global frame in m/s

    float gravity;                      ///< Gravity value used for the simulated forces
    float air_density;                  ///< Air density in kg/m3

    std::array<float,4> motor_dir;
} dynamic_model_quad_diag_conf_t;


/**
 * \brief   Default configuration
 *
 * \return  Config structure
 */
static inline dynamic_model_quad_diag_conf_t dynamic_model_quad_diag_default_config();


/**
 * \brief   Simulated dynamics of a quadcopter in diag configuration
 */
class Dynamic_model_quad_diag: public Dynamic_model
{
public:
    /**
     * @brief   Constructor
     *
     * \param   servo_rear_left     Reference to rear left servo,
     * \param   servo_front_left    Reference to front left servo
     * \param   servo_front_right   Reference to front right servo
     * \param   servo_rear_right    Reference to rear right servo
     * \param   config              Configuration
     */
    Dynamic_model_quad_diag(Servo& servo_rear_left,
                            Servo& servo_front_left,
                            Servo& servo_front_right,
                            Servo& servo_rear_right,
                            dynamic_model_quad_diag_conf_t config = dynamic_model_quad_diag_default_config());


    /**
     * \brief   Main update function
     * \detail  Reads new values from sensor
     *
     * \return  Success
     */
    bool update(void);


    /**
     * \brief   Get last update time in microseconds
     *
     * \return  Update time
     */
    const float& last_update_us(void) const;


    /**
     * \brief   Get X, Y and Z components of acceleration in body frame in m/s^2
     *
     * \return  Value
     */
    const std::array<float, 3>& acceleration_bf(void) const;


    /**
     * \brief   Get X, Y and Z components of velocity in local frame
     *
     * \return  Value
     */
    const std::array<float, 3>& velocity_lf(void) const;


    /**
     * \brief   Get X, Y and Z position in local frame (centered on home)
     *
     * \return  Value
     */
    const local_position_t& position_lf(void) const;


    /**
     * \brief   Get X, Y and Z position in global frame
     *
     * \return  Value
     */
    const global_position_t& position_gf(void) const;


    /**
     * \brief   Get X, Y and Z components of angular velocity in body frame
     *
     * \return  Value
     */
    const std::array<float, 3>& angular_velocity_bf(void) const;


    /**
     * \brief   Get attitude quaternion
     *
     * \return  Value
     */
    const quat_t& attitude(void) const;

private:
    Servo& servo_front_right_;              ///< Reference to front right servo
    Servo& servo_front_left_;               ///< Reference to front left servo
    Servo& servo_rear_right_;               ///< Reference to rear right servo
    Servo& servo_rear_left_;                ///< Reference to rear left servo

    dynamic_model_quad_diag_conf_t config_; ///< Configuration

    std::array<float, 4> rotorspeeds_;      ///< Estimated rotor speeds
    std::array<float, 3> torques_bf_;       ///< 3D torques vector applied on the vehicle
    std::array<float, 3> rates_bf_;         ///< 3D angular rates vector
    std::array<float, 3> lin_forces_bf_;        ///< 3D linear forces vector in body frame
    std::array<float, 3> acc_bf_;           ///< 3D acceleration vector in body frame
    std::array<float, 3> vel_bf_;           ///< 3D velocity vector in body frame
    std::array<float, 3> vel_;              ///< 3D velocity vector in NED frame
    quat_t attitude_;                       ///< Estimated attitude

    local_position_t local_position_;       ///< Simulated local position
    global_position_t global_position_;     ///< Simulated global position

    float last_update_us_;                  ///< The last update in micro seconds
    float dt_s_;                            ///< The time delta since last update in seconds


    /**
     * \brief   Computes forces applied to the UAV from servo input
     */
    void forces_from_servos(void);


    /**
     * \brief Inverse function of mix_to_servos in stabilization to recover torques and forces
     *
     * \param   sim                 The pointer to the simulation model structure
     * \param   rpm                 The rotation per minute of the rotor
     * \param   sqr_lat_airspeed    The square of the lateral airspeed
     * \param   axial_airspeed      The axial airspeed
     *
     * \return The value of the lift / drag value without the lift / drag coefficient
     */
    float lift_drag_base(float rpm, float sqr_lat_airspeed, float axial_airspeed);
};



static inline dynamic_model_quad_diag_conf_t dynamic_model_quad_diag_default_config()
{
    dynamic_model_quad_diag_conf_t conf = {};

    conf.rotor_lpf              = 0.1f;                 ///< Low pass filter constant (adjusted for time) to express rotor inertia/lag. 1.0       = no inertia, 0.0 = infinite inertia
    conf.rotor_rpm_gain         = 4000.0f;              ///< The gain linking the rotor command to rpm
    conf.rotor_rpm_offset       = -1.0f;                ///< Offset to convert servo commands to rpm (servo command that corresponds to zero rpm)
    conf.rotor_cd               = 0.03f;                ///< Coefficient of drag of rotor blade
    conf.rotor_cl               = 1.0f;                 ///< Coefficient of lift of rotor blade
    conf.rotor_diameter         = 0.14f;                ///< Mean "effective" rotor diameter
    conf.rotor_foil_area        = 0.18f * 0.015f;       ///< Area of the propeller blades in m^2
    conf.rotor_pitch            = 0.15f;                ///< Rotor pitch in m/revolution (7x6" roughly 0.15m)
    conf.total_mass             = 0.35f;                ///< Vehicle mass in kg
    conf.vehicle_drag           = 0.01f;                ///< Vehicle drag coefficient * vehicle area
    conf.roll_pitch_momentum    = 0.1f * 0.17f / 1.4142f;   ///< Angular momentum constants (assumed to be independent) (in kg/m^2)
    conf.yaw_momentum           = 0.1f * 0.17f;         ///< Approximate motor arm mass * rotor arm length
    conf.rotor_momentum         = 0.005f * 0.03f;       ///< Rotor inertia  (5g off center mass * rotor radius)
    conf.rotor_arm_length       = 0.17f;                ///< Distance between CoG and motor (in meter)
    conf.wind_x                 = 0.0f;                 ///< Wind in x axis, global frame
    conf.wind_y                 = 0.0f;                 ///< Wind in y axis, global frame
    conf.gravity                = 9.8f;                 ///< Simulation gravity
    conf.air_density            = 1.2f;                 ///< Air density
    conf.motor_dir              = std::array<float,4>{{-1.0f, 1.0f , -1.0f, 1.0f}};

    return conf;
}


#endif /* DYNAMIC_MODEL_QUAD_DIAG_HPP_ */
