/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/** 
 * \file simulation.h
 *  
 * This file is an onboard Hardware-in-the-loop simulation. 
 * Simulates quad-copter dynamics based on simple aerodynamics/physics model and generates simulated raw IMU measurements
 */


#ifndef SIMULATION_H_
#define SIMULATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "qfilter.h"

#include "conf_platform.h"
#include "imu.h"
#include "servo_pwm.h"
#include "bmp085.h"
#include "position_estimation.h"

// default home location (EFPL Esplanade)
#define HOME_LONGITUDE 6.566044801857777f					///< Latitude of home location
#define HOME_LATITUDE 46.51852236174565f 					///< Longitude of home location
#define HOME_ALTITUDE 400.0f									///< Altitude of home location

/**
 * \brief The simulation model structure definition
 */
typedef struct
{
	float torques_bf[3];									///< The 3D torques vector applied on the vehicle
	float rates_bf[3];										///< The 3D angular rates vector
	float lin_forces_bf[3];									///< The 3D linear forces vector in body frame
	float vel_bf[3];										///< The 3D velocity vector in body frame
	float vel[3];											///< The 3D velocity vector in NED frame
	float pos[3];											///< The 3D position vector in NED frame
	qfilter_t *attitude_filter;								///< The simulated attitude estimation
	local_coordinates_t localPosition;						///< The simulated local position
	
	float simu_raw_scale[9];								///< The raw scales of the simulated IMU
	float simu_raw_biais[9];								///< The raw biaises of the simulated IMU
	
	float rotorspeeds[ROTORCOUNT];                          ///< The estimated rotor speeds
	float rotor_lpf;										///< The low-pass filtered response of the rotors to simulate inertia and lag
	float rotor_rpm_gain;									///< The gain linking the command to rpm
	float rotor_rpm_offset;									///< The offset to convert servo commands to rpm
	
	float rotor_cd;											///< The rotor drag coefficient
	float rotor_cl;											///< The rotor lift coefficient
	float rotor_diameter;									///< The mean rotor diameter in m
	float rotor_foil_area;									///< The rotor foil area
	
	float rotor_pitch;										///< The rotor pitch
	float total_mass;										///< The vehicle mass in kg
	float vehicle_drag;										///< The coefficient of drag of the whole vehicle
	float roll_pitch_momentum;								///< The roll and pitch angular momentum of the vehicle
	float yaw_momentum;										///< The yaw angular momentum constants (assumed to be independent)
	
	float rotor_momentum;									///< The angular momentum of the rotor (for rotor inertia)
	float rotor_arm_length;									///< The distance between CoG and motor (in meter)
	uint32_t last_update;									///< The last update in system ticks
	float dt;												///< The time base of current update
	float wind_x;											///< The x component of wind in global frame in m/s
	float wind_y;											///< The y component of wind in global frame in m/s
} simulation_model_t;


/**
 * \brief	Initialize the simulation module
 *
 * \param	sim				The pointer to the simulation model structure
 * \param	imu				The pointer to the real IMU structure to match the simulated IMU
 * \param	localPos		The pointer to the structure of the real local position estimation of the vehicle
 */
void simulation_init(simulation_model_t *sim, qfilter_t *attitude_filter, local_coordinates_t localPos);

/**
 * \brief	Computes artificial gyro and accelerometer values based on motor commands
 *
 * \param	sim				The pointer to the simulation model structure
 * \param	servo_commands	The pointer to the commands that would be sent to the servo (i.e. brushless motors)
 * \param	imu				The pointer to the "real" IMU structure to update the belief of the attitude
 * \param	localPos		The pointer to the structure of the "real" local position of the vehicle to update the belief of the position and velocity
 */
void simulation_update(simulation_model_t *sim, servo_output_t *servo_commands, Imu_Data_t *imu, position_estimator_t *pos_est);

/**
 * \brief	Simulates barometer outputs
 *
 * \param	sim				The pointer to the simulation model structure
 * \param	pressure		The pointer to the "real" barometer structure to update the belief of the barometer value
 */
void simulation_simulate_barometer(simulation_model_t *sim, pressure_data_t *pressure);

/**
 * \brief	Simulates GPS outputs
 *
 * \param	sim				The pointer to the simulation model structure
 * \param	gps				The pointer to the "real" GPS structure to update the belief of the GPS value
 */
void simulation_simulate_gps(simulation_model_t *sim, gps_Data_type_t *gps);

#ifdef __cplusplus
}
#endif

#endif /* SIMULATION_H_ */