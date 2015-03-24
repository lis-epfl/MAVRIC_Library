/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file simulation.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is an onboard Hardware-in-the-loop simulation. 
 * 
 * \detail Simulates quad-copter dynamics based on simple aerodynamics/physics 
 * model and generates simulated raw IMU measurements
 *
 ******************************************************************************/


#ifndef SIMULATION_H_
#define SIMULATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "qfilter.h"

#include "imu.h"
#include "servos.h"
#include "barometer.h"
#include "sonar.h"
#include "position_estimation.h"
#include "state.h"

#define AIR_DENSITY 1.2								///< The air density
#define ROTORCOUNT 4								///< Define number of motors


/**
 * \brief The vehicle simulation model structure definition
 */
typedef struct
{
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
	
	float wind_x;											///< The x component of wind in global frame in m/s
	float wind_y;											///< The y component of wind in global frame in m/s
	
	float home_coordinates[3];								///< The home coordinates in global frame (GPS, latitude, longitude, altitude in degrees and meters)
	float sim_gravity;										///< The value of gravity for the simulated forces
}simulation_config_t;

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
	ahrs_t ahrs;											///< The simulated attitude estimation
	local_coordinates_t local_position;						///< The simulated local position								
	
	sensor_calib_t calib_gyro;								///< The calibration values of the gyroscope
	sensor_calib_t calib_accelero;							///< The calibration values of the accelerometer
	sensor_calib_t calib_compass;							///< The calibration values of the compass
	
	float rotorspeeds[ROTORCOUNT];                          ///< The estimated rotor speeds

	simulation_config_t vehicle_config;						///< The vehicle configuration variables

	uint32_t last_update;									///< The last update in system ticks
	float dt;												///< The time base of current update
	
	imu_t* imu;												///< The pointer to the IMU structure
	position_estimation_t* pos_est;							///< The pointer to the position estimation structure
	barometer_t* pressure;									///< The pointer to the barometer structure
	gps_t* gps;												///< The pointer to the GPS structure
	sonar_t* sonar;											///< The pointer to the sonar structure
	const state_t* state;									///< The pointer to the state structure
	const servos_t* servos;									///< The pointer to the servos structure
	const ahrs_t *estimated_attitude;						///< The pointer to the attitude estimation structure
	bool* nav_plan_active;									///< The pointer to the waypoint set flag
} simulation_model_t;


/**
 * \brief	Initialize the simulation module
 *
 * \param	sim				The pointer to the simulation model structure
 * \param	sim_config		The pointer to the configuration structure
 * \param	ahrs			The pointer to the AHRS structure
 * \param	imu				The pointer to the real IMU structure to match the simulated IMU
 * \param	pos_est			The pointer to the position estimation structure of the vehicle
 * \param	pressure		The pointer to the pressure structure
 * \param	gps				The pointer to the GPS structure
 * \param	sonar			The pointer to the sonar structure
 * \param	state			The pointer to the state structure
 * \param	servos			The pointer to the servos structure
 * \param	waypoint_set	The pointer to the waypoint_set boolean value
 *
 * \return	True if the init succeed, false otherwise
 */
bool simulation_init(	simulation_model_t* sim, 
						const simulation_config_t* sim_config, 
						ahrs_t* ahrs, 
						imu_t* imu, 
						position_estimation_t* pos_est, 
						barometer_t* pressure, 
						gps_t* gps, 
						sonar_t* sonar, 
						state_t* state, 
						const servos_t* servos, 
						bool* waypoint_set);


/**
 * \brief	Sets the calibration to the "real" IMU values
 *
 * \param	sim				The pointer to the simulation model structure
 */
void simulation_calib_set(simulation_model_t *sim);


/**
 * \brief	Computes artificial gyro and accelerometer values based on motor commands
 *
 * \param	sim				The pointer to the simulation model structure
 */
void simulation_update(simulation_model_t *sim);


/**
 * \brief	Simulates barometer outputs
 *
 * \param	sim				The pointer to the simulation model structure
 */
void simulation_simulate_barometer(simulation_model_t *sim);


/**
 * \brief	Simulates GPS outputs
 *
 * \param	sim				The pointer to the simulation model structure
 */
void simulation_simulate_gps(simulation_model_t *sim);


/**
 * \brief	Gives a fake gps value
 * 
 * \param	sim				The pointer to the simulation model structure
 * \param	timestamp_ms	The time stamp in ms
 */
void simulation_fake_gps_fix(simulation_model_t* sim, uint32_t timestamp_ms);


/**
 * \brief	Simulates sonar outputs
 *
 * \param	sim				The pointer to the simulation model structure
 */
void simulation_simulate_sonar(simulation_model_t *sim);


/**
 * \brief	Changes from reality to simulation
 *
 * \param	sim				The pointer to the simulation model structure
 */
void simulation_switch_from_reality_to_simulation(simulation_model_t *sim);


/**
 * \brief	Changes from simulation to reality
 *
 * \param	sim				The pointer to the simulation model structure
 */
void simulation_switch_from_simulation_to_reality(simulation_model_t *sim);


#ifdef __cplusplus
}
#endif

#endif /* SIMULATION_H_ */