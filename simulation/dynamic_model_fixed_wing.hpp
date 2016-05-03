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
 * \file dynamic_model_fixed_wing.hpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Jacquemin
 * \author Julien Lecoeur
 *
 * \brief Simulated dynamics of a fixed wing UAV
 *
 ******************************************************************************/


#ifndef DYNAMIC_MODEL_FIXED_WING_HPP_
#define DYNAMIC_MODEL_FIXED_WING_HPP_


#include "simulation/dynamic_model.hpp"
#include "drivers/servo.hpp"
#include "simulation/wing_model.hpp"

extern "C"
{
#include "util/constants.h"
}


/**
 * \brief Configuration for fixed wing UAV
 */
typedef struct
{
    float rotor_lpf;                    ///< Low-pass filtered response of the rotors to simulate inertia and lag
    float rotor_rpm_gain;               ///< Gain linking the command to rpm*/
    float rotor_rpm_offset;             ///< Offset to convert servo commands to rpm
    float flap_offset;                  ///< Offset to convert servo commands to flap angle
    float flap_max;                     ///< Maximum angle of the flaps (In rad)

    float rotor_cd;                     ///< Rotor drag coefficient
    float rotor_cl;                     ///< Rotor lift coefficient
    float rotor_diameter;               ///< Mean rotor diameter in m
    float rotor_foil_area;              ///< Rotor foil area

    float rotor_pitch;                  ///< Rotor pitch*/
    float total_mass;                   ///< Vehicle mass in kg

    float roll_momentum;                ///< Roll angular momentum of the vehicle
    float pitch_momentum;               ///< Pitch angular momentum of the vehicle
    float yaw_momentum;                 ///< Yaw angular momentum constants (assumed to be independent)

    float rotor_momentum;               ///< Angular momentum of the rotor (for rotor inertia)

    float wind_x;                       ///< X component of wind in global frame in m/s
    float wind_y;                       ///< Y component of wind in global frame in m/s

    float home_coordinates[3];          ///< Home coordinates in global frame (GPS, latitude, longitude, altitude in degrees and meters)

    float gravity;                      ///< Gravity value used for the simulated forces
    float air_density;                  ///< Air density in kg/m3

} dynamic_model_fixed_wing_conf_t;


/**
 * \brief   Default configuration
 *
 * \return  Config structure
 */
static inline dynamic_model_fixed_wing_conf_t dynamic_model_fixed_wing_default_config();


/**
 * \brief   Simulated dynamics of a fixed wing UAV
 */
class Dynamic_model_fixed_wing: public Dynamic_model
{
public:
    /**
     * @brief   Constructor
     *
     * \param   servo_motor_        Reference to motor
     * \param   servo_flap_left     Reference to left flap
     * \param   servo_flap_right    Reference to right flap
     * \param   config              Configuration
     */
    Dynamic_model_fixed_wing(Servo& servo_motor,
            Servo& servo_flap_left,
            Servo& servo_flap_right,
            dynamic_model_fixed_wing_conf_t config = dynamic_model_fixed_wing_default_config());


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

    /**
     * \brief   Get x speed in BF
     *
     * \return Value
     */
     const float x_speed_bf(void) const;

private:
    Servo& servo_motor_;         ///< Reference to motor servo
    Servo& servo_flap_left_;     ///< Reference to left flap servo
    Servo& servo_flap_right_;    ///< Reference to right flap servo

    dynamic_model_fixed_wing_conf_t config_; ///< Configuration

    float motor_speed_;          ///< Speed of the motor

    Wing_model left_flap_;       ///< Left wing+flap
    Wing_model right_flap_;      ///< Right wing+flap
    Wing_model left_drift_;      ///< Left drift
    Wing_model right_drift_;     ///< Right drift

    std::array<float, 3> torques_bf_;       ///< 3D torques vector applied on the vehicle
    std::array<float, 3> rates_bf_;         ///< 3D angular rates vector
    std::array<float, 3> lin_forces_bf_;    ///< 3D linear forces vector in body frame
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
     * \brief   Computes forces applied by the motor
     *
     * \param   wind_bf         quaternion of the relative wind in body frame
     * \param   motor_command   the motor command issued by the servo
     *
     * \return  force structure with 3 forces and 3 torques
     */
    wing_model_forces_t compute_motor_forces(float wind_bf[3],float motor_command);

    /**
     * \brief   Computes the base coefficient to compute the lift an drag
     *
     * \param   rpm                 the number of rotations per minute of the blades
     * \param   sqr_lat_airspeed    the squared speed of the airflow parallel to the plane of rotation of the blades
     * \param   axial_airspeed      the speed of the airflow perpendicular to the plane of rotation of the blades
     *
     * \return  a float coefficient
    */
    float lift_drag_base(float rpm, float sqr_lat_airspeed, float axial_airspeed);
};



static inline dynamic_model_fixed_wing_conf_t dynamic_model_fixed_wing_default_config()
{
    dynamic_model_fixed_wing_conf_t conf = {};

    conf.rotor_lpf              = 0.1f;                 ///< Low pass filter constant (adjusted for time) to express rotor inertia/lag. 1.0       = no inertia, 0.0 = infinite inertia
    conf.rotor_rpm_gain         = 4000.0f;              ///< The gain linking the rotor command to rpm
    conf.rotor_rpm_offset       = -1.0f;                ///< Offset to convert servo commands to rpm (servo command that corresponds to zero rpm)
    conf.flap_offset            = 0.0f;                 ///< Offset to convert servo commands to flap angle (servo command that corresponds to zero angle)
    conf.flap_max               = 0.69813170079f;       ///< Maximum angle of the flaps (In rad) corresponds to a servo command of 1
    conf.rotor_cd               = 0.03f;                ///< Coefficient of drag of rotor blade
    conf.rotor_cl               = 1.0f;                 ///< Coefficient of lift of rotor blade
    conf.rotor_diameter         = 0.14f;                ///< Mean "effective" rotor diameter
    conf.rotor_foil_area        = 0.18f * 0.015f;       ///< Area of the propeller blades in m^2
    conf.rotor_pitch            = 0.15f;                ///< Rotor pitch in m/revolution (7x6" roughly 0.15m)
    conf.total_mass             = 0.4f;                ///< Vehicle mass in kg
    conf.roll_momentum          = 16438.7f/(1000.0f*1000.0f);///< Angular momentum constants (in kg/m^2)
    conf.pitch_momentum         = 3840.4f/(1000.0f*1000.0f);///< Angular momentum constants (in kg/m^2)
    conf.yaw_momentum           = 20260.4f/(1000.0f*1000.0f);///< Angular momentum constants (in kg/m^2)
    conf.rotor_momentum         = 0.005f * 0.03f;       ///< Rotor inertia  (5g off center mass * rotor radius)
    conf.wind_x                 = 0.0f;                 ///< Wind in x axis, global frame
    conf.wind_y                 = 0.0f;                 ///< Wind in y axis, global frame
    conf.gravity                = 9.8f;                 ///< Simulation gravity
    conf.air_density            = 1.2f;                 ///< Air density*/

    //default home location (EFPL Esplanade)
    conf.home_coordinates[X]    = 46.51852236174565f;   ///< Latitude of the simulation home waypoint
    conf.home_coordinates[Y]    = 6.566044801857777f;   ///< Longitude of the simulation home waypoint
    conf.home_coordinates[Z]    = 400.0f;               ///< Altitude of the simulation home waypoint

    return conf;
}


#endif /* DYNAMIC_MODEL_FIXED_WING_HPP_ */
