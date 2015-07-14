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
 * \file simulation.c
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

#include "simulation.h"
#include "time_keeper.h"
#include "coord_conventions.h"
#include "quaternions.h"
#include "maths.h"
#include "print_util.h"
#include "constants.h"

#include "conf_platform.h" 	// TODO: remove (use the module mix_to_servo to remove dependency to conf_platform)

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Computer the forces in the local frame for a "diagonal" quadrotor configuration
 *
 * \param	sim				The pointer to the simulation structure
 */
void forces_from_servos_diag_quad(simulation_model_t *sim);

/**
 * \brief	Computes the forces in the local frame of a "cross" quadrotor configuration
 *
 * \warning	This function is not implemented
 *
 * \param	sim				The pointer to the simulation structure
 */
void forces_from_servos_cross_quad(simulation_model_t *sim);

/**
 * \brief	Resets the simulation towards the "real" estimated position
 *
 * \param	sim				The pointer to the simulation model structure
 */
static void simulation_reset_simulation(simulation_model_t *sim);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

/** 
 * \brief Inverse function of mix_to_servos in stabilization to recover torques and forces
 *
 * \param	sim					The pointer to the simulation model structure
 * \param	rpm					The rotation per minute of the rotor
 * \param	sqr_lat_airspeed	The square of the lateral airspeed
 * \param	axial_airspeed		The axial airspeed
 *
 * \return The value of the lift / drag value without the lift / drag coefficient
 */
static inline float lift_drag_base(simulation_model_t *sim, float rpm, float sqr_lat_airspeed, float axial_airspeed) 
{
	if (rpm < 0.1f)
	{
		return 0.0f;
	}
	float mean_vel = sim->vehicle_config.rotor_diameter *PI * rpm / 60.0f;
	float exit_vel = rpm / 60.0f *sim->vehicle_config.rotor_pitch;
	           
	return (0.5f * AIR_DENSITY * (mean_vel * mean_vel +sqr_lat_airspeed) * sim->vehicle_config.rotor_foil_area  * (1.0f - (axial_airspeed / exit_vel)));
}

static void simulation_reset_simulation(simulation_model_t *sim)
{
	for (int32_t i = 0; i < 3; i++)
	{
		sim->rates_bf[i] = 0.0f;
		sim->torques_bf[i] = 0.0f;
		sim->lin_forces_bf[i] = 0.0f;
		sim->vel_bf[i] = 0.0f;
		sim->vel[i] = 0.0f;
	}
	
	sim->local_position = sim->pos_est->local_position;
	
	sim->ahrs = *sim->estimated_attitude;
	
	print_util_dbg_print("(Re)setting simulation. Origin: (");
	print_util_dbg_print_num(sim->pos_est->local_position.origin.latitude*10000000,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->pos_est->local_position.origin.longitude*10000000,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->pos_est->local_position.origin.altitude*1000,10);
	print_util_dbg_print("), Position: (x1000) (");
	print_util_dbg_print_num(sim->pos_est->local_position.pos[0]*1000,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->pos_est->local_position.pos[1]*1000,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->pos_est->local_position.pos[2]*1000,10);
	print_util_dbg_print(")\r\n");
	
	//sim->local_position.origin.latitude = HOME_LATITUDE;
	//sim->local_position.origin.longitude = HOME_LONGITUDE;
	//sim->local_position.origin.altitude = HOME_ALTITUDE;
	
	//sim->local_position.origin = sim->pos_est->local_position.origin;
	//sim->local_position.heading = sim->pos_est->local_position.heading;
}

void forces_from_servos_diag_quad(simulation_model_t* sim)
{
	float motor_command[4];
	float rotor_lifts[4], rotor_drags[4], rotor_inertia[4];
	float ldb;
	quat_t wind_gf = {.s = 0, .v = {sim->vehicle_config.wind_x, sim->vehicle_config.wind_y, 0.0f}};
	quat_t wind_bf = quaternions_global_to_local(sim->ahrs.qe, wind_gf);
	
	float sqr_lateral_airspeed = SQR(sim->vel_bf[0] + wind_bf.v[0]) + SQR(sim->vel_bf[1] + wind_bf.v[1]);
	float lateral_airspeed = sqrt(sqr_lateral_airspeed);
	
	float old_rotor_speed;
	
	for (int32_t i = 0; i < 4; i++)
	{
		motor_command[i] = (float)sim->servos->servo[i].value - sim->vehicle_config.rotor_rpm_offset;
		if (motor_command[i] < 0.0f) 
		{
			motor_command[i] = 0;
		}
		
		// temporarily save old rotor speeds
		old_rotor_speed = sim->rotorspeeds[i];
		// estimate rotor speeds by low - pass filtering
		//sim->rotorspeeds[i] = (sim->vehicle_config.rotor_lpf) * sim->rotorspeeds[i] + (1.0f - sim->vehicle_config.rotor_lpf) * (motor_command[i] * sim->vehicle_config.rotor_rpm_gain);
		sim->rotorspeeds[i] = (motor_command[i] * sim->vehicle_config.rotor_rpm_gain);
		
		// calculate torque created by rotor inertia
		rotor_inertia[i] = (sim->rotorspeeds[i] - old_rotor_speed) / sim->dt * sim->vehicle_config.rotor_momentum;
		
		ldb = lift_drag_base(sim, sim->rotorspeeds[i], sqr_lateral_airspeed, -sim->vel_bf[Z]);
		//ldb = lift_drag_base(sim, sim->rotorspeeds[i], sqr_lateral_airspeed, 0.0f);
		
		rotor_lifts[i] = ldb * sim->vehicle_config.rotor_cl;
		rotor_drags[i] = ldb * sim->vehicle_config.rotor_cd;
	}
	
	float mpos_x = sim->vehicle_config.rotor_arm_length / 1.4142f;
	float mpos_y = sim->vehicle_config.rotor_arm_length / 1.4142f;
	
	// torque around x axis (roll)
	sim->torques_bf[ROLL]  = ((rotor_lifts[M_FRONT_LEFT]  + rotor_lifts[M_REAR_LEFT]  ) 
						    - (rotor_lifts[M_FRONT_RIGHT] + rotor_lifts[M_REAR_RIGHT] )) * mpos_y;;

	// torque around y axis (pitch)
	sim->torques_bf[PITCH] = ((rotor_lifts[M_FRONT_LEFT]  + rotor_lifts[M_FRONT_RIGHT] )
							- (rotor_lifts[M_REAR_LEFT]   + rotor_lifts[M_REAR_RIGHT] ))*  mpos_x;

	sim->torques_bf[YAW]   = (M_FL_DIR * (10.0f * rotor_drags[M_FRONT_LEFT] + rotor_inertia[M_FRONT_LEFT])  + M_FR_DIR * (10.0f * rotor_drags[M_FRONT_RIGHT] + rotor_inertia[M_FRONT_RIGHT])
							+ M_RL_DIR * (10.0f * rotor_drags[M_REAR_LEFT] + rotor_inertia[M_REAR_LEFT])   + M_RR_DIR * (10.0f * rotor_drags[M_REAR_RIGHT] + rotor_inertia[M_REAR_RIGHT] ))*  sim->vehicle_config.rotor_diameter;
	

	
	sim->lin_forces_bf[X] = -(sim->vel_bf[X] - wind_bf.v[0]) * lateral_airspeed * sim->vehicle_config.vehicle_drag;  
	sim->lin_forces_bf[Y] = -(sim->vel_bf[Y] - wind_bf.v[1]) * lateral_airspeed * sim->vehicle_config.vehicle_drag;
	sim->lin_forces_bf[Z] = -(rotor_lifts[M_FRONT_LEFT]+ rotor_lifts[M_FRONT_RIGHT] +rotor_lifts[M_REAR_LEFT] +rotor_lifts[M_REAR_RIGHT]);

}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool simulation_init(simulation_model_t* sim, const simulation_config_t* sim_config, ahrs_t* ahrs, imu_t* imu, position_estimation_t* pos_est, barometer_t* pressure, gps_t* gps, sonar_t* sonar, state_t* state, const servos_t* servos, bool* waypoint_set)
{
	bool init_success = true;
	
	//Init dependencies
	sim->vehicle_config = *sim_config;
	sim->imu = imu;
	sim->pos_est = pos_est;
	sim->pressure = pressure;
	sim->gps = gps;
	sim->sonar = sonar;
	sim->state = state;
	sim->servos = servos;
	sim->nav_plan_active = &state->nav_plan_active;
	
	// set initial conditions to a given attitude_filter
	sim->estimated_attitude = ahrs;
	sim->ahrs = *ahrs;

	print_util_dbg_print("Attitude:");
	print_util_dbg_print_num(sim->ahrs.qe.s*100,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->ahrs.qe.v[0]*100,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->ahrs.qe.v[1]*100,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->ahrs.qe.v[2]*100,10);
	print_util_dbg_print("\r\n");
	

	for (int32_t i = 0; i < ROTORCOUNT; i++)
	{
		sim->rotorspeeds[i] = 0.0f;			
	}
	sim->last_update = time_keeper_get_micros();
	sim->dt = 0.01f;
	
	simulation_reset_simulation(sim);
	simulation_calib_set(sim);
	
	print_util_dbg_print("[HIL SIMULATION] initialised.\r\n");
	
	return init_success;
}

void simulation_calib_set(simulation_model_t *sim)
{
	for (int32_t i = 0;i < 3;i++)
	{
		//we take in sim the inverse of the imu scale_factor to limit number of division
		//while feeding raw_sensor.data[i]
		sim->calib_gyro.scale_factor[i]			= 1.0f/sim->imu->calib_gyro.scale_factor[i];
		sim->calib_accelero.scale_factor[i]		= 1.0f/sim->imu->calib_accelero.scale_factor[i];
		sim->calib_compass.scale_factor[i]		= 1.0f/sim->imu->calib_compass.scale_factor[i];
		
		sim->calib_gyro.bias[i]					= sim->imu->calib_gyro.bias[i];
		sim->calib_accelero.bias[i]				= sim->imu->calib_accelero.bias[i];
		sim->calib_compass.bias[i]				= sim->imu->calib_compass.bias[i];
		
		sim->calib_gyro.orientation[i]			= sim->imu->calib_gyro.orientation[i];
		sim->calib_accelero.orientation[i]		= sim->imu->calib_accelero.orientation[i];
		sim->calib_compass.orientation[i]		= sim->imu->calib_compass.orientation[i];
	}
	
	//reset the simulated ahrs
	sim->ahrs.qe.s = 1.0f;
	sim->ahrs.qe.v[0] = 0.0f;
	sim->ahrs.qe.v[1] = 0.0f;	
	sim->ahrs.qe.v[2] = 0.0f;
	
}

void simulation_update(simulation_model_t *sim)
{
	int32_t i;
	quat_t qtmp1, qvel_bf,  qed;
	const quat_t front = {.s = 0.0f, .v = {1.0f, 0.0f, 0.0f}};
	const quat_t up = {.s = 0.0f, .v = {UPVECTOR_X, UPVECTOR_Y, UPVECTOR_Z}};
	
	
	uint32_t now = time_keeper_get_micros();
	sim->dt = (now - sim->last_update) / 1000000.0f;
	if (sim->dt > 0.1f)
	{
		sim->dt = 0.1f;
	}
	
	sim->last_update = now;
	// compute torques and forces based on servo commands
	forces_from_servos_diag_quad(sim);
	
	// integrate torques to get simulated gyro rates (with some damping)
	sim->rates_bf[0] = maths_clip((1.0f - 0.1f * sim->dt) * sim->rates_bf[0] + sim->dt * sim->torques_bf[0] / sim->vehicle_config.roll_pitch_momentum, 10.0f);
	sim->rates_bf[1] = maths_clip((1.0f - 0.1f * sim->dt) * sim->rates_bf[1] + sim->dt * sim->torques_bf[1] / sim->vehicle_config.roll_pitch_momentum, 10.0f);
	sim->rates_bf[2] = maths_clip((1.0f - 0.1f * sim->dt) * sim->rates_bf[2] + sim->dt * sim->torques_bf[2] / sim->vehicle_config.yaw_momentum, 10.0f);
	
	
	for (i = 0; i < 3; i++)
	{
			qtmp1.v[i] = 0.5f * sim->rates_bf[i];
	}
	
	qtmp1.s = 0;

	// apply step rotation 
	qed = quaternions_multiply(sim->ahrs.qe,qtmp1);

	// TODO: correct this formulas when using the right scales
	sim->ahrs.qe.s = sim->ahrs.qe.s + qed.s * sim->dt;
	sim->ahrs.qe.v[0] += qed.v[0] * sim->dt;
	sim->ahrs.qe.v[1] += qed.v[1] * sim->dt;
	sim->ahrs.qe.v[2] += qed.v[2] * sim->dt;

	sim->ahrs.qe = quaternions_normalise(sim->ahrs.qe);
	sim->ahrs.up_vec = quaternions_global_to_local(sim->ahrs.qe, up);
	
	sim->ahrs.north_vec = quaternions_global_to_local(sim->ahrs.qe, front);	

	// velocity and position integration
	
	// check altitude - if it is lower than 0, clamp everything (this is in NED, assuming negative altitude)
	if (sim->local_position.pos[Z] >0)
	{
		sim->vel[Z] = 0.0f;
		sim->local_position.pos[Z] = 0.0f;

		// simulate "acceleration" caused by contact force with ground, compensating gravity
		for (i = 0; i < 3; i++)
		{
			sim->lin_forces_bf[i] = sim->ahrs.up_vec.v[i] * sim->vehicle_config.total_mass * sim->vehicle_config.sim_gravity;
		}
				
		// slow down... (will make velocity slightly inconsistent until next update cycle, but shouldn't matter much)
		for (i = 0; i < 3; i++)
		{
			sim->vel_bf[i] = 0.95f * sim->vel_bf[i];
		}
		
		//upright
		sim->rates_bf[0] =  sim->ahrs.up_vec.v[1]; 
		sim->rates_bf[1] =  - sim->ahrs.up_vec.v[0];
		sim->rates_bf[2] = 0;
	}
	
	sim->ahrs.qe = quaternions_normalise(sim->ahrs.qe);
	sim->ahrs.up_vec = quaternions_global_to_local(sim->ahrs.qe, up);
	
	sim->ahrs.north_vec = quaternions_global_to_local(sim->ahrs.qe, front);	
	for (i = 0; i < 3; i++)
	{
			qtmp1.v[i] = sim->vel[i];
	}
	qtmp1.s = 0.0f;
	qvel_bf = quaternions_global_to_local(sim->ahrs.qe, qtmp1);
	
	float acc_bf[3];
	for (i = 0; i < 3; i++)
	{
		sim->vel_bf[i] = qvel_bf.v[i];
		
		// following the convention in the IMU, this is the acceleration due to force, as measured
		sim->ahrs.linear_acc[i] = sim->lin_forces_bf[i] / sim->vehicle_config.total_mass;
		
		// this is the "clean" acceleration without gravity
		acc_bf[i] = sim->ahrs.linear_acc[i] - sim->ahrs.up_vec.v[i] * sim->vehicle_config.sim_gravity;
		
		sim->vel_bf[i] = sim->vel_bf[i] + acc_bf[i] * sim->dt;
	}
	
	// calculate velocity in global frame
	// vel = qe *vel_bf * qe - 1
	qvel_bf.s = 0.0f; qvel_bf.v[0] = sim->vel_bf[0]; qvel_bf.v[1] = sim->vel_bf[1]; qvel_bf.v[2] = sim->vel_bf[2];
	qtmp1 = quaternions_local_to_global(sim->ahrs.qe, qvel_bf);
	sim->vel[0] = qtmp1.v[0]; sim->vel[1] = qtmp1.v[1]; sim->vel[2] = qtmp1.v[2];
	
	for (i = 0; i < 3; i++)
	{
		sim->local_position.pos[i] = sim->local_position.pos[i] + sim->vel[i] * sim->dt;
	}

	// fill in simulated IMU values
	for (i = 0;i < 3; i++)
	{
		sim->imu->raw_gyro.data[sim->imu->calib_gyro.axis[i]] = (sim->rates_bf[i] * sim->calib_gyro.scale_factor[i] + sim->calib_gyro.bias[i]) * sim->calib_gyro.orientation[i];
		sim->imu->raw_accelero.data[sim->imu->calib_accelero.axis[i]] = ((sim->lin_forces_bf[i] / sim->vehicle_config.total_mass / sim->vehicle_config.sim_gravity) * sim->calib_accelero.scale_factor[i] + sim->calib_accelero.bias[i]) * sim->calib_accelero.orientation[i];
		sim->imu->raw_compass.data[sim->imu->calib_compass.axis[i]] = ((sim->ahrs.north_vec.v[i] ) * sim->calib_compass.scale_factor[i] + sim->calib_compass.bias[i])* sim->calib_compass.orientation[i];
	}

	sim->local_position.heading = coord_conventions_get_yaw(sim->ahrs.qe);
}

void simulation_simulate_barometer(simulation_model_t *sim)
{
	sim->pressure->altitude = sim->local_position.origin.altitude - sim->local_position.pos[Z];
	sim->pressure->vario_vz = sim->vel[Z];
	sim->pressure->last_update = time_keeper_get_millis();
	sim->pressure->altitude_offset = 0;
}
	
void simulation_simulate_gps(simulation_model_t *sim)
{
	global_position_t gpos = coord_conventions_local_to_global_position(sim->local_position);
	
	sim->gps->altitude = gpos.altitude;
	sim->gps->latitude = gpos.latitude;
	sim->gps->longitude = gpos.longitude;
	sim->gps->time_last_msg = time_keeper_get_millis();
	sim->gps->status = GPS_OK;
}

void simulation_fake_gps_fix(simulation_model_t* sim, uint32_t timestamp_ms)
{
	local_coordinates_t fake_pos;
	
	fake_pos.pos[X] = 10.0f;
	fake_pos.pos[Y] = 10.0f;
	fake_pos.pos[Z] = 0.0f;
	fake_pos.origin.latitude = sim->vehicle_config.home_coordinates[0];
	fake_pos.origin.longitude = sim->vehicle_config.home_coordinates[1];
	fake_pos.origin.altitude = sim->vehicle_config.home_coordinates[2];
	fake_pos.timestamp_ms = timestamp_ms;

	global_position_t gpos = coord_conventions_local_to_global_position(fake_pos);
	
	sim->gps->latitude = gpos.latitude;
	sim->gps->longitude = gpos.longitude;
	sim->gps->altitude = gpos.altitude;
	sim->gps->time_last_msg = time_keeper_get_millis();
	sim->gps->status = GPS_OK;
}

void simulation_simulate_sonar(simulation_model_t *sim)
{
	int16_t distance_cm = 0.5f - 100 * sim->local_position.pos[Z];
	float distance_m = (float)distance_cm / 100.0f;

	if ( distance_m > sim->sonar->min_distance && distance_m < sim->sonar->max_distance )
	{
		sim->sonar->current_distance = distance_m;
		sim->sonar->last_update = time_keeper_get_millis();
		sim->sonar->healthy = true;
	}
	else
	{
		sim->sonar->healthy = false;
	}
}

void simulation_switch_from_reality_to_simulation(simulation_model_t *sim)
{
	print_util_dbg_print("Switching from reality to simulation.\r\n");

	simulation_reset_simulation(sim);
	simulation_calib_set(sim);
	sim->pos_est->init_gps_position = false;
}
		

void simulation_switch_from_simulation_to_reality(simulation_model_t *sim)
{
	print_util_dbg_print("Switching from simulation to reality.\r\n");
	
	sim->pos_est->local_position.origin = sim->local_position.origin;
	for (uint32_t i = 0; i < 3; i++)
	{
		sim->pos_est->local_position.pos[i] = 0.0f;
	}
	sim->pos_est->init_gps_position = false;
}