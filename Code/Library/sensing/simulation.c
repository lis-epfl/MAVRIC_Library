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

void init_simulation(simulation_model_t *sim, Imu_Data_t *imu, local_coordinates_t localPos) {
	int i;
	
	dbg_print("Init HIL simulation. \n");
	
	(*sim) = vehicle_model_parameters;
	for (i=0; i<3; i++)
	{
		sim->rates_bf[i] = 0.0f;
		sim->torques_bf[i] = 0.0f;
		sim->lin_forces_bf[i] = 0.0f;
		sim->vel_bf[i] = 0.0f;
		sim->localPosition.pos[i] = localPos.pos[i];
	}
	
	//sim->localPosition.origin.latitude = HOME_LATITUDE;
	//sim->localPosition.origin.longitude = HOME_LONGITUDE;
	//sim->localPosition.origin.altitude = HOME_ALTITUDE;
	
	sim->localPosition.origin = localPos.origin;
	
	// set initial conditions to given attitude (including scalefactors and biases for simulated IMU)
	sim->attitude = imu->attitude;

	for (i=0; i<ROTORCOUNT; i++)
	{
		sim->rotorspeeds[i] = 0.0;			
	}
	sim->last_update = get_micros();
	sim->dt = 0.01;
	
	for (i=0;i<9;i++)
	{
		sim->simu_raw_scale[i] = 1.0/imu->attitude.sf[i];
		sim->simu_raw_biais[i] = imu->attitude.be[i];
	}
}

/** 
 * \brief Inverse function of mix_to_servos in stabilization to recover torques and forces
 *
 * \param	sim					The pointer to the simulation model structure
 * \param	rpm					The rotation per minute of the rotor
 * \param	sqr_lat_airspeed	The square of the lateral airspeed
 * \param	axial_airspeed		The axial airspeed
 *
 * \return The value of the lift/drag value without the lift/drag coefficient
 */
static inline float lift_drag_base(simulation_model_t *sim, float rpm, float sqr_lat_airspeed, float axial_airspeed) {
	if (rpm < 0.1)
	{
		return 0.0;
	}
	float mean_vel = sim->rotor_diameter *PI * rpm/60.0;
	float exit_vel = rpm/60.0 *sim -> rotor_pitch;
	           
	return (0.5*AIR_DENSITY*(mean_vel*mean_vel +sqr_lat_airspeed) * sim->rotor_foil_area  * (1.0-(axial_airspeed/exit_vel)));
}


void forces_from_servos_diag_quad(simulation_model_t *sim, servo_output *servos){
	int i;
	float motor_command[4];
	float rotor_lifts[4], rotor_drags[4], rotor_inertia[4], rotor_lateral_drag[4];
	float ldb;
	UQuat_t wind_gf = {.s = 0, .v = {sim->wind_x, sim->wind_y, 0.0}};
	UQuat_t wind_bf = quat_global_to_local(sim->attitude.qe, wind_gf);
	
	float sqr_lateral_airspeed = SQR(sim->vel_bf[0]+wind_bf.v[0]) + SQR(sim->vel_bf[1]+wind_bf.v[1]);
	float lateral_airspeed = sqrt(sqr_lateral_airspeed);
	
	for (i=0; i<4; i++)
	{
		float old_rotor_speed;
		motor_command[i] = (float)servos[i].value/SERVO_SCALE - sim->rotor_rpm_offset;
		if (motor_command[i]<0.0) 
		{
			motor_command[i] = 0;
		}
		
		// temporarily save old rotor speeds
		old_rotor_speed = sim->rotorspeeds[i];
		// estimate rotor speeds by low-pass filtering
		//sim->rotorspeeds[i] = (sim->rotor_lpf) * sim->rotorspeeds[i] + (1.0-sim->rotor_lpf) * (motor_command[i] * sim->rotor_rpm_gain);
		sim->rotorspeeds[i] = (motor_command[i] * sim->rotor_rpm_gain);
		
		// calculate torque created by rotor inertia
		rotor_inertia[i] = (sim->rotorspeeds[i] - old_rotor_speed)/sim->dt * sim->rotor_momentum;
		
		ldb = lift_drag_base(sim, sim->rotorspeeds[i], sqr_lateral_airspeed, -sim->vel_bf[Z]);
		//ldb = lift_drag_base(sim, sim->rotorspeeds[i], sqr_lateral_airspeed, 0.0);
		
		rotor_lifts[i] = ldb * sim->rotor_cl;
		rotor_drags[i] = ldb * sim->rotor_cd;
	}
	
	float mpos_x = sim->rotor_arm_length / 1.4142;
	float mpos_y = sim->rotor_arm_length / 1.4142;
	
	// torque around x axis (roll)
	sim->torques_bf[ROLL]  = ((rotor_lifts[M_FRONT_LEFT]  + rotor_lifts[M_REAR_LEFT]  ) 
						    - (rotor_lifts[M_FRONT_RIGHT] + rotor_lifts[M_REAR_RIGHT] )) * mpos_y;;

	// torque around y axis (pitch)
	sim->torques_bf[PITCH] = ((rotor_lifts[M_FRONT_LEFT]  + rotor_lifts[M_FRONT_RIGHT] )
							- (rotor_lifts[M_REAR_LEFT]   + rotor_lifts[M_REAR_RIGHT] ))*  mpos_x;

	sim->torques_bf[YAW]   = (M_FL_DIR*(10.0*rotor_drags[M_FRONT_LEFT]+rotor_inertia[M_FRONT_LEFT])  + M_FR_DIR*(10.0*rotor_drags[M_FRONT_RIGHT]+rotor_inertia[M_FRONT_RIGHT])
							+ M_RL_DIR*(10.0*rotor_drags[M_REAR_LEFT] +rotor_inertia[M_REAR_LEFT])   + M_RR_DIR*(10.0*rotor_drags[M_REAR_RIGHT] +rotor_inertia[M_REAR_RIGHT] ))*  sim->rotor_diameter;
	

	
	sim->lin_forces_bf[X] = -(sim->vel_bf[X]-wind_bf.v[0])*lateral_airspeed* sim->vehicle_drag;  
	sim->lin_forces_bf[Y] = -(sim->vel_bf[Y]-wind_bf.v[1])*lateral_airspeed* sim->vehicle_drag;
	sim->lin_forces_bf[Z] = -(rotor_lifts[M_FRONT_LEFT]+ rotor_lifts[M_FRONT_RIGHT] +rotor_lifts[M_REAR_LEFT] +rotor_lifts[M_REAR_RIGHT]);

}


void forces_from_servos_cross_quad(simulation_model_t *sim, servo_output *servos){
	int i;
	float motor_command[4];
	
	//TODO: implement the correct forces
/*	motor_command[M_FRONT] = control->thrust + control->rpy[PITCH] + M_FRONT_DIR * control->rpy[YAW];
	motor_command[M_RIGHT] = control->thrust - control->rpy[ROLL] + M_RIGHT_DIR * control->rpy[YAW];
	motor_command[M_REAR]  = control->thrust - control->rpy[PITCH] + M_REAR_DIR * control->rpy[YAW];
	motor_command[M_LEFT]  = control->thrust + control->rpy[ROLL] + M_LEFT_DIR * control->rpy[YAW];
	for (i=0; i<4; i++) {
		if (motor_command[i]<MIN_THRUST) motor_command[i] = MIN_THRUST;
		if (motor_command[i]>MAX_THRUST) motor_command[i] = MAX_THRUST;
	}

	for (i=0; i<4; i++) {
		centralData->servos[i].value = SERVO_SCALE*motor_command[i];
	}
	*/
}

void simu_update(simulation_model_t *sim, servo_output *servo_commands, Imu_Data_t *imu, position_estimator_t *pos_est) {
	int i;
	UQuat_t qtmp1, qvel_bf,  qed;
	const UQuat_t front = {.s = 0.0, .v = {1.0, 0.0, 0.0}};
	const UQuat_t up = {.s = 0.0, .v = {UPVECTOR_X, UPVECTOR_Y, UPVECTOR_Z}};
	
	
	uint32_t now = get_micros();
	sim->dt = (now - sim->last_update)/1000000.0;
	if (sim->dt>0.1)
	{
		sim->dt = 0.1;
	}
	
	sim->last_update = now;
	// compute torques and forces based on servo commands
	#ifdef CONF_DIAG
	forces_from_servos_diag_quad(sim, servo_commands);
	#endif
	#ifdef CONF_CROSS
	forces_from_servos_cross_quad(sim, servo_commands);
	#endif
	
	// integrate torques to get simulated gyro rates (with some damping)
	sim->rates_bf[0] = clip((1.0-0.1*sim->dt)*sim->rates_bf[0] + sim->dt * sim->torques_bf[0] /sim->roll_pitch_momentum, 10.0);
	sim->rates_bf[1] = clip((1.0-0.1*sim->dt)*sim->rates_bf[1] + sim->dt * sim->torques_bf[1] /sim->roll_pitch_momentum, 10.0);
	sim->rates_bf[2] = clip((1.0-0.1*sim->dt)*sim->rates_bf[2] + sim->dt * sim->torques_bf[2] /sim->yaw_momentum, 10.0);
	
	
	for (i=0; i<3; i++)
	{
			qtmp1.v[i] = sim->rates_bf[i];
	}
	
	qtmp1.s = 0;

	// apply step rotation 
	qed = quat_multi(sim->attitude.qe,qtmp1);

	sim->attitude.qe.s = sim->attitude.qe.s+qed.s*sim->dt;
	sim->attitude.qe.v[0] += qed.v[0]*sim->dt;
	sim->attitude.qe.v[1] += qed.v[1]*sim->dt;
	sim->attitude.qe.v[2] += qed.v[2]*sim->dt;

	sim->attitude.qe = quat_normalise(sim->attitude.qe);
	sim->attitude.up_vec = quat_global_to_local(sim->attitude.qe, up);
	
	sim->attitude.north_vec = quat_global_to_local(sim->attitude.qe, front);	

	// velocity and position integration
	
	// check altitude - if it is lower than 0, clamp everything (this is in NED, assuming negative altitude)
	if (sim->localPosition.pos[Z] >0)
	{
		sim->vel[Z] = 0.0;
		sim->localPosition.pos[Z] = 0.0;

		// simulate "acceleration" caused by contact force with ground, compensating gravity
		for (i = 0; i<3; i++)
		{
			sim->lin_forces_bf[i] = sim->attitude.up_vec.v[i]*sim->total_mass *GRAVITY;
		}
				
		// slow down... (will make velocity slightly inconsistent until next update cycle, but shouldn't matter much)
		for (i = 0; i<3; i++)
		{
			sim->vel_bf[i] = 0.95*sim->vel_bf[i];
		}
		
		//upright
		sim->rates_bf[0] =  - (-sim->attitude.up_vec.v[1] ); 
		sim->rates_bf[1] =  - sim->attitude.up_vec.v[0];
		sim->rates_bf[2] = 0;
	}
	
	sim->attitude.qe = quat_normalise(sim->attitude.qe);
	sim->attitude.up_vec = quat_global_to_local(sim->attitude.qe, up);
	
	sim->attitude.north_vec = quat_global_to_local(sim->attitude.qe, front);	
	for (i=0; i<3; i++)
	{
			qtmp1.v[i] = sim->vel[i];
	}
	qtmp1.s = 0.0;
	qvel_bf = quat_global_to_local(sim->attitude.qe, qtmp1);
	for (i=0; i<3; i++)
	{
		sim->vel_bf[i] = qvel_bf.v[i];
		
		// following the convention in the IMU, this is the acceleration due to force, as measured
		sim->attitude.a[i] = sim->lin_forces_bf[i] / sim->total_mass;
		
		// this is the "clean" acceleration without gravity
		sim->attitude.acc_bf[i] = sim->attitude.a[i] - sim->attitude.up_vec.v[i] * GRAVITY;
		
		sim->vel_bf[i] = sim->vel_bf[i] + sim->attitude.acc_bf[i] * sim->dt;
	}
	
	// calculate velocity in global frame
	// vel = qe *vel_bf * qe-1
	qvel_bf.s = 0.0; qvel_bf.v[0] = sim->vel_bf[0]; qvel_bf.v[1] = sim->vel_bf[1]; qvel_bf.v[2] = sim->vel_bf[2];
	qtmp1 = quat_local_to_global(sim->attitude.qe, qvel_bf);
	sim->vel[0] = qtmp1.v[0]; sim->vel[1] = qtmp1.v[1]; sim->vel[2] = qtmp1.v[2];
	
	for (i=0; i<3; i++)
	{
		sim->localPosition.pos[i] = sim->localPosition.pos[i] + sim->vel[i] * sim->dt;
	}

	// fill in simulated IMU values
	
	imu->raw_channels[GYRO_OFFSET+IMU_X] = sim->rates_bf[0] * sim->simu_raw_scale[GYRO_OFFSET+IMU_X]+sim->simu_raw_biais[GYRO_OFFSET+IMU_X];
	imu->raw_channels[GYRO_OFFSET+IMU_Y] = sim->rates_bf[1] * sim->simu_raw_scale[GYRO_OFFSET+IMU_Y]+sim->simu_raw_biais[GYRO_OFFSET+IMU_Y];
	imu->raw_channels[GYRO_OFFSET+IMU_Z] = sim->rates_bf[2] * sim->simu_raw_scale[GYRO_OFFSET+IMU_Z]+sim->simu_raw_biais[GYRO_OFFSET+IMU_Z];

	imu->raw_channels[ACC_OFFSET+IMU_X] = (sim->lin_forces_bf[0] / sim->total_mass / GRAVITY)*sim->simu_raw_scale[ACC_OFFSET+IMU_X]+sim->simu_raw_biais[ACC_OFFSET+IMU_X];
	imu->raw_channels[ACC_OFFSET+IMU_Y] = (sim->lin_forces_bf[1] / sim->total_mass / GRAVITY)*sim->simu_raw_scale[ACC_OFFSET+IMU_Y]+sim->simu_raw_biais[ACC_OFFSET+IMU_Y];
	imu->raw_channels[ACC_OFFSET+IMU_Z] = (sim->lin_forces_bf[2] / sim->total_mass / GRAVITY)*sim->simu_raw_scale[ACC_OFFSET+IMU_Z]+sim->simu_raw_biais[ACC_OFFSET+IMU_Z];
	// cheating... provide true upvector instead of simulated forces
	//imu->raw_channels[ACC_OFFSET+IMU_X] = sim->attitude.up_vec.v[0] *imu->raw_scale[ACC_OFFSET+IMU_X]+imu->raw_bias[ACC_OFFSET+IMU_X];
	//imu->raw_channels[ACC_OFFSET+IMU_Y] = sim->attitude.up_vec.v[1] *imu->raw_scale[ACC_OFFSET+IMU_Y]+imu->raw_bias[ACC_OFFSET+IMU_Y];
	//imu->raw_channels[ACC_OFFSET+IMU_Z] = sim->attitude.up_vec.v[2] *imu->raw_scale[ACC_OFFSET+IMU_Z]+imu->raw_bias[ACC_OFFSET+IMU_Z];
	
	imu->raw_channels[COMPASS_OFFSET+IMU_X] = (sim->attitude.north_vec.v[0] )*sim->simu_raw_scale[COMPASS_OFFSET+IMU_X]+sim->simu_raw_biais[COMPASS_OFFSET+IMU_X];
	imu->raw_channels[COMPASS_OFFSET+IMU_Y] = (sim->attitude.north_vec.v[1] )*sim->simu_raw_scale[COMPASS_OFFSET+IMU_Y]+sim->simu_raw_biais[COMPASS_OFFSET+IMU_Y];
	imu->raw_channels[COMPASS_OFFSET+IMU_Z] = (sim->attitude.north_vec.v[2] )*sim->simu_raw_scale[COMPASS_OFFSET+IMU_Z]+sim->simu_raw_biais[COMPASS_OFFSET+IMU_Z];
	
	//imu->dt = sim->dt;

	sim->localPosition.heading = get_yaw(sim->attitude.qe);
	//pos_est->localPosition = sim->localPosition;
}

void simulate_barometer(simulation_model_t *sim, pressure_data *pressure)
{
	pressure->altitude = sim->localPosition.origin.altitude - sim->localPosition.pos[Z];
	pressure->vario_vz = sim->vel[Z];
	pressure->last_update = get_millis();
	pressure->altitude_offset = 0;
}
	
void simulate_gps(simulation_model_t *sim, gps_Data_type *gps)
{
	global_position_t gpos = local_to_global_position(sim->localPosition);
	
	gps->altitude = gpos.altitude;
	gps->latitude = gpos.latitude;
	gps->longitude = gpos.longitude;
	gps->timeLastMsg = get_millis();
	gps->status = GPS_OK;
}
