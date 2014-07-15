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
 * \file simulation.c
 *  
 * This file is an onboard Hardware-in-the-loop simulation. 
 * Simulates quad-copter dynamics based on simple aerodynamics/physics model and generates simulated raw IMU measurements
 */


#include "conf_sim_model.h"
#include "time_keeper.h"
#include "coord_conventions.h"

#include "central_data.h"
#include "maths.h"


/**
 * \brief	Computes the forces in the local frame of a "cross" quadrotor configuration
 *
 * \warning	This function is not implemented
 *
 * \param	sim		The pointer to the simulation structure
 * \param	servos	The pointer to the servos structure
 */
void forces_from_servos_cross_quad(simulation_model_t *sim, servo_output_t *servos);

/**
 * \brief	Computer the forces in the local frame for a "diagonal" quadrotor configuration
 *
 * \param	sim		The pointer to the simulation structure
 * \param	servos	The pointer to the servos structure
 */
void forces_from_servos_diag_quad(simulation_model_t *sim, servo_output_t *servos);

void simulation_init(simulation_model_t* sim, qfilter_t* attitude_filter, Imu_Data_t* imu, position_estimator_t* pos_est, pressure_data_t* pressure, gps_Data_type_t* gps, state_structure_t* state_structure)
{
	int32_t i;
	
	sim->imu = imu;
	sim->pos_est = pos_est;
	sim->pressure = pressure;
	sim->gps = gps;
	sim->state_structure = state_structure;
	
	print_util_dbg_print("Init HIL simulation. \n");
	
	(*sim) = vehicle_model_parameters;
	
	simulation_reset_simulation(sim);
	
	// set initial conditions to a given attitude_filter
	sim->attitude_filter = *attitude_filter;

	for (i = 0; i < ROTORCOUNT; i++)
	{
		sim->rotorspeeds[i] = 0.0f;			
	}
	sim->last_update = time_keeper_get_micros();
	sim->dt = 0.01f;
	
	simulation_calib_set(sim);
	
}

void simulation_calib_set(simulation_model_t *sim)
{
	int32_t i;
	
	for (i = 0;i < 3;i++)
	{
		//sim->simu_raw_scale[i + GYRO_OFFSET]	= 1.0f / sim->imu->gyro_calib.scale[i];				// Because the inversion is done in imu_init
		//sim->simu_raw_scale[i + ACC_OFFSET]		= 1.0f / sim->imu->accelero_calib.scale[i];
		//sim->simu_raw_scale[i + MAG_OFFSET]		= 1.0f / sim->imu->compass_calib.scale[i];
		
		sim->simu_raw_scale[i + GYRO_OFFSET]	= sim->imu->gyro_calib.scale[i];
		sim->simu_raw_scale[i + ACC_OFFSET]		= sim->imu->accelero_calib.scale[i];
		sim->simu_raw_scale[i + MAG_OFFSET]		= sim->imu->compass_calib.scale[i];
		sim->simu_raw_biais[i + GYRO_OFFSET]	= sim->imu->gyro_calib.bias[i];
		sim->simu_raw_biais[i + ACC_OFFSET]		= sim->imu->accelero_calib.bias[i];
		sim->simu_raw_biais[i + MAG_OFFSET]		= sim->imu->compass_calib.bias[i];
	}
}

void simulation_reset_simulation(simulation_model_t *sim)
{
	int32_t i;
	
	for (i = 0; i < 3; i++)
	{
		sim->rates_bf[i] = 0.0f;
		sim->torques_bf[i] = 0.0f;
		sim->lin_forces_bf[i] = 0.0f;
		sim->vel_bf[i] = 0.0f;
		sim->vel[i] = 0.0f;
		//sim->localPosition.pos[i] = sim->pos_est->localPosition.pos[i];
	}
	
	sim->localPosition = sim->pos_est->localPosition;
	
	//sim->localPosition.origin.latitude = HOME_LATITUDE;
	//sim->localPosition.origin.longitude = HOME_LONGITUDE;
	//sim->localPosition.origin.altitude = HOME_ALTITUDE;
	
	//sim->localPosition.origin = sim->pos_est->localPosition.origin;
	//sim->localPosition.heading = sim->pos_est->localPosition.heading;
}

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
static inline float lift_drag_base(simulation_model_t *sim, float rpm, float sqr_lat_airspeed, float axial_airspeed) {
	if (rpm < 0.1f)
	{
		return 0.0f;
	}
	float mean_vel = sim->rotor_diameter *PI * rpm / 60.0f;
	float exit_vel = rpm / 60.0f *sim -> rotor_pitch;
	           
	return (0.5f * AIR_DENSITY * (mean_vel * mean_vel +sqr_lat_airspeed) * sim->rotor_foil_area  * (1.0f - (axial_airspeed / exit_vel)));
}


void forces_from_servos_diag_quad(simulation_model_t *sim, servo_output_t *servos){
	int32_t i;
	float motor_command[4];
	float rotor_lifts[4], rotor_drags[4], rotor_inertia[4];
	float ldb;
	UQuat_t wind_gf = {.s = 0, .v = {sim->wind_x, sim->wind_y, 0.0f}};
	UQuat_t wind_bf = quaternions_global_to_local(sim->attitude.qe, wind_gf);
	
	float sqr_lateral_airspeed = SQR(sim->vel_bf[0] + wind_bf.v[0]) + SQR(sim->vel_bf[1] + wind_bf.v[1]);
	float lateral_airspeed = sqrt(sqr_lateral_airspeed);
	
	for (i = 0; i < 4; i++)
	{
		float old_rotor_speed;
		motor_command[i] = (float)servos[i].value / SERVO_SCALE - sim->rotor_rpm_offset;
		if (motor_command[i] < 0.0f) 
		{
			motor_command[i] = 0;
		}
		
		// temporarily save old rotor speeds
		old_rotor_speed = sim->rotorspeeds[i];
		// estimate rotor speeds by low - pass filtering
		//sim->rotorspeeds[i] = (sim->rotor_lpf) * sim->rotorspeeds[i] + (1.0f - sim->rotor_lpf) * (motor_command[i] * sim->rotor_rpm_gain);
		sim->rotorspeeds[i] = (motor_command[i] * sim->rotor_rpm_gain);
		
		// calculate torque created by rotor inertia
		rotor_inertia[i] = (sim->rotorspeeds[i] - old_rotor_speed) / sim->dt * sim->rotor_momentum;
		
		ldb = lift_drag_base(sim, sim->rotorspeeds[i], sqr_lateral_airspeed, -sim->vel_bf[Z]);
		//ldb = lift_drag_base(sim, sim->rotorspeeds[i], sqr_lateral_airspeed, 0.0f);
		
		rotor_lifts[i] = ldb * sim->rotor_cl;
		rotor_drags[i] = ldb * sim->rotor_cd;
	}
	
	float mpos_x = sim->rotor_arm_length / 1.4142f;
	float mpos_y = sim->rotor_arm_length / 1.4142f;
	
	// torque around x axis (roll)
	sim->torques_bf[ROLL]  = ((rotor_lifts[M_FRONT_LEFT]  + rotor_lifts[M_REAR_LEFT]  ) 
						    - (rotor_lifts[M_FRONT_RIGHT] + rotor_lifts[M_REAR_RIGHT] )) * mpos_y;;

	// torque around y axis (pitch)
	sim->torques_bf[PITCH] = ((rotor_lifts[M_FRONT_LEFT]  + rotor_lifts[M_FRONT_RIGHT] )
							- (rotor_lifts[M_REAR_LEFT]   + rotor_lifts[M_REAR_RIGHT] ))*  mpos_x;

	sim->torques_bf[YAW]   = (M_FL_DIR * (10.0f * rotor_drags[M_FRONT_LEFT] + rotor_inertia[M_FRONT_LEFT])  + M_FR_DIR * (10.0f * rotor_drags[M_FRONT_RIGHT] + rotor_inertia[M_FRONT_RIGHT])
							+ M_RL_DIR * (10.0f * rotor_drags[M_REAR_LEFT] +rotor_inertia[M_REAR_LEFT])   + M_RR_DIR * (10.0f * rotor_drags[M_REAR_RIGHT] +rotor_inertia[M_REAR_RIGHT] ))*  sim->rotor_diameter;
	

	
	sim->lin_forces_bf[X] = -(sim->vel_bf[X] - wind_bf.v[0]) * lateral_airspeed * sim->vehicle_drag;  
	sim->lin_forces_bf[Y] = -(sim->vel_bf[Y] - wind_bf.v[1]) * lateral_airspeed * sim->vehicle_drag;
	sim->lin_forces_bf[Z] = -(rotor_lifts[M_FRONT_LEFT]+ rotor_lifts[M_FRONT_RIGHT] +rotor_lifts[M_REAR_LEFT] +rotor_lifts[M_REAR_RIGHT]);

}


void forces_from_servos_cross_quad(simulation_model_t *sim, servo_output_t *servos){
	//int32_t i;
	//float motor_command[4];
	
	//TODO: implement the correct forces
/*	motor_command[M_FRONT] = control->thrust + control->rpy[PITCH] + M_FRONT_DIR * control->rpy[YAW];
	motor_command[M_RIGHT] = control->thrust - control->rpy[ROLL] + M_RIGHT_DIR * control->rpy[YAW];
	motor_command[M_REAR]  = control->thrust - control->rpy[PITCH] + M_REAR_DIR * control->rpy[YAW];
	motor_command[M_LEFT]  = control->thrust + control->rpy[ROLL] + M_LEFT_DIR * control->rpy[YAW];
	for (i = 0; i < 4; i++) {
		if (motor_command[i] < MIN_THRUST) motor_command[i] = MIN_THRUST;
		if (motor_command[i] > MAX_THRUST) motor_command[i] = MAX_THRUST;
	}

	for (i = 0; i < 4; i++) {
		centralData->servos[i].value = SERVO_SCALE * motor_command[i];
	}
	*/
}

void simulation_update(simulation_model_t *sim, servo_output_t *servo_commands, Imu_Data_t *imu, position_estimator_t *pos_est) {
	int32_t i;
	UQuat_t qtmp1, qvel_bf,  qed;
	const UQuat_t front = {.s = 0.0f, .v = {1.0f, 0.0f, 0.0f}};
	const UQuat_t up = {.s = 0.0f, .v = {UPVECTOR_X, UPVECTOR_Y, UPVECTOR_Z}};
	
	
	uint32_t now = time_keeper_get_micros();
	sim->dt = (now - sim->last_update) / 1000000.0f;
	if (sim->dt > 0.1f)
	{
		sim->dt = 0.1f;
	}
	
	sim->last_update = now;
	// compute torques and forces based on servo commands
	#ifdef CONF_DIAG
	forces_from_servos_diag_quad(sim, servo_commands);
	#endif
	#ifdef CONF_CROSS
	forces_from_servos_cross_quad(sim, servo_commands);
	#endif
	
	// pid_control_integrate torques to get simulated gyro rates (with some damping)
	sim->rates_bf[0] = maths_clip((1.0f - 0.1f * sim->dt) * sim->rates_bf[0] + sim->dt * sim->torques_bf[0] / sim->roll_pitch_momentum, 10.0f);
	sim->rates_bf[1] = maths_clip((1.0f - 0.1f * sim->dt) * sim->rates_bf[1] + sim->dt * sim->torques_bf[1] / sim->roll_pitch_momentum, 10.0f);
	sim->rates_bf[2] = maths_clip((1.0f - 0.1f * sim->dt) * sim->rates_bf[2] + sim->dt * sim->torques_bf[2] / sim->yaw_momentum, 10.0f);
	
	
	for (i = 0; i < 3; i++)
	{
			qtmp1.v[i] = sim->rates_bf[i];
	}
	
	qtmp1.s = 0;

	// apply step rotation 
	qed = quaternions_multiply(sim->attitude.qe,qtmp1);

	sim->attitude.qe.s = sim->attitude.qe.s + qed.s * sim->dt;
	sim->attitude.qe.v[0] += qed.v[0] * sim->dt;
	sim->attitude.qe.v[1] += qed.v[1] * sim->dt;
	sim->attitude.qe.v[2] += qed.v[2] * sim->dt;

	sim->attitude.qe = quaternions_normalise(sim->attitude.qe);
	sim->attitude.up_vec = quaternions_global_to_local(sim->attitude.qe, up);
	
	sim->attitude.north_vec = quaternions_global_to_local(sim->attitude.qe, front);	

	// velocity and position integration
	
	// check altitude - if it is lower than 0, clamp everything (this is in NED, assuming negative altitude)
	if (sim->localPosition.pos[Z] >0)
	{
		sim->vel[Z] = 0.0f;
		sim->localPosition.pos[Z] = 0.0f;

		// simulate "acceleration" caused by contact force with ground, compensating gravity
		for (i = 0; i < 3; i++)
		{
			sim->lin_forces_bf[i] = sim->attitude.up_vec.v[i] * sim->total_mass * GRAVITY;
		}
				
		// slow down... (will make velocity slightly inconsistent until next update cycle, but shouldn't matter much)
		for (i = 0; i < 3; i++)
		{
			sim->vel_bf[i] = 0.95f * sim->vel_bf[i];
		}
		
		//upright
		sim->rates_bf[0] =  - (-sim->attitude.up_vec.v[1] ); 
		sim->rates_bf[1] =  - sim->attitude.up_vec.v[0];
		sim->rates_bf[2] = 0;
	}
	
	sim->attitude.qe = quaternions_normalise(sim->attitude.qe);
	sim->attitude.up_vec = quaternions_global_to_local(sim->attitude.qe, up);
	
	sim->attitude.north_vec = quaternions_global_to_local(sim->attitude.qe, front);	
	for (i = 0; i < 3; i++)
	{
			qtmp1.v[i] = sim->vel[i];
	}
	qtmp1.s = 0.0f;
	qvel_bf = quaternions_global_to_local(sim->attitude.qe, qtmp1);
	for (i = 0; i < 3; i++)
	{
		sim->vel_bf[i] = qvel_bf.v[i];
		
		// following the convention in the IMU, this is the acceleration due to force, as measured
		sim->attitude.a[i] = sim->lin_forces_bf[i] / sim->total_mass;
		
		// this is the "clean" acceleration without gravity
		sim->attitude.acc_bf[i] = sim->attitude.a[i] - sim->attitude.up_vec.v[i] * GRAVITY;
		
		sim->vel_bf[i] = sim->vel_bf[i] + sim->attitude.acc_bf[i] * sim->dt;
	}
	
	// calculate velocity in global frame
	// vel = qe *vel_bf * qe - 1
	qvel_bf.s = 0.0f; qvel_bf.v[0] = sim->vel_bf[0]; qvel_bf.v[1] = sim->vel_bf[1]; qvel_bf.v[2] = sim->vel_bf[2];
	qtmp1 = quaternions_local_to_global(sim->attitude.qe, qvel_bf);
	sim->vel[0] = qtmp1.v[0]; sim->vel[1] = qtmp1.v[1]; sim->vel[2] = qtmp1.v[2];
	
	for (i = 0; i < 3; i++)
	{
		sim->localPosition.pos[i] = sim->localPosition.pos[i] + sim->vel[i] * sim->dt;
	}

	// fill in simulated IMU values
	
	imu->raw_channels[GYRO_OFFSET + IMU_X] = sim->rates_bf[0] * sim->simu_raw_scale[GYRO_OFFSET + IMU_X] + sim->simu_raw_biais[GYRO_OFFSET + IMU_X];
	imu->raw_channels[GYRO_OFFSET + IMU_Y] = sim->rates_bf[1] * sim->simu_raw_scale[GYRO_OFFSET + IMU_Y] + sim->simu_raw_biais[GYRO_OFFSET + IMU_Y];
	imu->raw_channels[GYRO_OFFSET + IMU_Z] = sim->rates_bf[2] * sim->simu_raw_scale[GYRO_OFFSET + IMU_Z] + sim->simu_raw_biais[GYRO_OFFSET + IMU_Z];

	imu->raw_channels[ACC_OFFSET + IMU_X] = (sim->lin_forces_bf[0] / sim->total_mass / GRAVITY) * sim->simu_raw_scale[ACC_OFFSET + IMU_X] + sim->simu_raw_biais[ACC_OFFSET + IMU_X];
	imu->raw_channels[ACC_OFFSET + IMU_Y] = (sim->lin_forces_bf[1] / sim->total_mass / GRAVITY) * sim->simu_raw_scale[ACC_OFFSET + IMU_Y] + sim->simu_raw_biais[ACC_OFFSET + IMU_Y];
	imu->raw_channels[ACC_OFFSET + IMU_Z] = (sim->lin_forces_bf[2] / sim->total_mass / GRAVITY) * sim->simu_raw_scale[ACC_OFFSET + IMU_Z] + sim->simu_raw_biais[ACC_OFFSET + IMU_Z];
	// cheating... provide true upvector instead of simulated forces
	//imu->raw_channels[ACC_OFFSET + IMU_X] = sim->attitude.up_vec.v[0] * imu- > raw_scale[ACC_OFFSET + IMU_X] + imu->raw_bias[ACC_OFFSET + IMU_X];
	//imu->raw_channels[ACC_OFFSET + IMU_Y] = sim->attitude.up_vec.v[1] * imu->raw_scale[ACC_OFFSET + IMU_Y] + imu->raw_bias[ACC_OFFSET + IMU_Y];
	//imu->raw_channels[ACC_OFFSET + IMU_Z] = sim->attitude.up_vec.v[2] * imu->raw_scale[ACC_OFFSET + IMU_Z] + imu->raw_bias[ACC_OFFSET + IMU_Z];
	
	imu->raw_channels[MAG_OFFSET + IMU_X] = (sim->attitude.north_vec.v[0] ) * sim->simu_raw_scale[MAG_OFFSET + IMU_X] + sim->simu_raw_biais[MAG_OFFSET + IMU_X];
	imu->raw_channels[MAG_OFFSET + IMU_Y] = (sim->attitude.north_vec.v[1] ) * sim->simu_raw_scale[MAG_OFFSET + IMU_Y] + sim->simu_raw_biais[MAG_OFFSET + IMU_Y];
	imu->raw_channels[MAG_OFFSET + IMU_Z] = (sim->attitude.north_vec.v[2] ) * sim->simu_raw_scale[MAG_OFFSET + IMU_Z] + sim->simu_raw_biais[MAG_OFFSET + IMU_Z];
	
	//imu->dt = sim->dt;

	sim->localPosition.heading = coord_conventions_get_yaw(sim->attitude.qe);
	//pos_est->localPosition = sim->localPosition;
}

void simulation_simulate_barometer(simulation_model_t *sim, pressure_data_t *pressure)
{
	pressure->altitude = sim->localPosition.origin.altitude - sim->localPosition.pos[Z];
	pressure->vario_vz = sim->vel[Z];
	pressure->last_update = time_keeper_get_millis();
	pressure->altitude_offset = 0;
}
	
void simulation_simulate_gps(simulation_model_t *sim, gps_Data_type_t *gps)
{
	global_position_t gpos = coord_conventions_local_to_global_position(sim->localPosition);
	
	gps->altitude = gpos.altitude;
	gps->latitude = gpos.latitude;
	gps->longitude = gpos.longitude;
	gps->timeLastMsg = time_keeper_get_millis();
	gps->status = GPS_OK;
}

void simulation_fake_gps_fix(simulation_model_t* sim, uint32_t timestamp_ms)
{
	local_coordinates_t fake_pos;
	
	fake_pos.pos[X] = 10.0f;
	fake_pos.pos[Y] = 10.0f;
	fake_pos.pos[Z] = 0.0f;
	fake_pos.origin.latitude = HOME_LATITUDE;
	fake_pos.origin.longitude = HOME_LONGITUDE;
	fake_pos.origin.altitude = HOME_ALTITUDE;
	fake_pos.timestamp_ms = timestamp_ms;

	global_position_t gpos = coord_conventions_local_to_global_position(fake_pos);
	
	sim->gps->latitude = gpos.latitude;
	sim->gps->longitude = gpos.longitude;
	sim->gps->altitude = gpos.altitude;
	sim->gps->time_last_msg = time_keeper_get_millis();
	sim->gps->status = GPS_OK;
}

void simulation_switch_between_reality_n_simulation(simulation_model_t *sim, servo_output_t servos[])
{
	uint32_t i;
	
	// From simulation to reality
	if (sim->state_structure->simulation_mode == 0)
	{
		sim->pos_est->
		sim->pos_est->localPosition.origin = sim->localPosition.origin;
		for (i = 0;i < 3;i++)
		{
			sim->pos_est->localPosition.pos[i] = sim->localPosition.pos[i];
		}
		sim->pos_est->init_gps_position = false;
		sim->state_structure->mav_state = MAV_STATE_STANDBY;
		sim->state_structure->mav_mode = MAV_MODE_MANUAL_DISARMED;
		servo_pwm_failsafe(servos);
	}

	// From reality to simulation
	if (sim->state_structure->simulation_mode == 1)
	{
		simulation_reset_simulation(sim);
		simulation_calib_set(sim);
		centralData->position_estimator.init_gps_position = false;
	}
}