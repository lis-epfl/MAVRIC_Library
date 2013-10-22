/*
 * simulation.c
 *
 * Onboard Hardware-in-the-loop simulation 
 * Simulates quad-copter dynamics based on simple aerodynamics/physics model and generates simulated raw IMU measurements
 * 
 * Created: 06/08/2013 17:02:56
 *  Author: Felix Schill
 */ 

#include "conf_sim_model.h"
#include "time_keeper.h"
#include "coord_conventions.h"
#include "position_estimation.h"
#include "central_data.h"
void init_simulation(simulation_model_t *sim) {
	int i;
	(*sim)=vehicle_model_parameters;
	for (i=0; i<3; i++) {
		sim->rates_bf[i]=0;
		sim->torques_bf[i]=0;
		sim->lin_forces_bf[i]=0;
		sim->vel_bf[i]=0.0;
		sim->pos[i]=0.0;
		
	}
	for (i=0; i<ROTORCOUNT; i++) {
		sim->rotorspeeds[i]=0.0;			
	}
	sim->last_update=get_time();
	sim->dt=0.01;
}



// inverse function of mix_to_servos in stabilisation to recover torques and forces

static inline float lift_drag_base(simulation_model_t *sim, float rpm, float sqr_lat_airspeed, float axial_airspeed) {
	if (rpm < 0.1) return 0.0;
	float mean_vel=sim->rotor_diameter *PI * rpm/60.0;
	float exit_vel=rpm/60.0 *sim -> rotor_pitch;           
	return (0.5*AIR_DENSITY*(mean_vel*mean_vel +sqr_lat_airspeed) * sim->rotor_foil_area  * (1.0-(axial_airspeed/exit_vel)));
}


void forces_from_servos_diag_quad(simulation_model_t *sim, servo_output *servos){
	int i;
	float motor_command[4];
	float rotor_lifts[4], rotor_drags[4], rotor_inertia[4], rotor_lateral_drag[4];
	float ldb;
	
	float sqr_lateral_airspeed=sim->vel_bf[0]*sim->vel_bf[0] + sim->vel_bf[1]*sim->vel_bf[1];
	float lateral_airspeed=sqrt(sqr_lateral_airspeed);
	
	for (i=0; i<4; i++) {
		float old_rotor_speed;
		motor_command[i]=(float)servos[i].value/SERVO_SCALE - sim->rotor_rpm_offset;
		if (motor_command[i]<0.0) motor_command[i]=0;
		
		// temporarily save old rotor speeds
		old_rotor_speed=sim->rotorspeeds[i];
		// estimate rotor speeds by low-pass filtering
		//sim->rotorspeeds[i]=(1.0-sim->rotor_lpf) * sim->rotorspeeds[i] + (sim->rotor_lpf) * (motor_command[i] * sim->rotor_rpm_gain);
		sim->rotorspeeds[i]=(motor_command[i] * sim->rotor_rpm_gain);
		
		// calculate torque created by rotor inertia
		rotor_inertia[i]=(sim->rotorspeeds[i] - old_rotor_speed)/sim->dt * sim->rotor_momentum;
		
		ldb=lift_drag_base(sim, sim->rotorspeeds[i], sqr_lateral_airspeed, -sim->vel_bf[Z]);
		//ldb=lift_drag_base(sim, sim->rotorspeeds[i], sqr_lateral_airspeed, 0.0);
		
		rotor_lifts[i]=ldb * sim->rotor_cl;
		rotor_drags[i]=ldb * sim->rotor_cd;
	}
	
	float mpos_x=sim->rotor_arm_length / 1.4142;
	float mpos_y=sim->rotor_arm_length / 1.4142;
	
	// torque around x axis (roll)
	sim->torques_bf[ROLL]  = ((rotor_lifts[M_FRONT_LEFT]  + rotor_lifts[M_REAR_LEFT]  ) 
						    - (rotor_lifts[M_FRONT_RIGHT] + rotor_lifts[M_REAR_RIGHT] )) * mpos_y;;

	// torque around y axis (pitch)
	sim->torques_bf[PITCH] = ((rotor_lifts[M_FRONT_LEFT]  + rotor_lifts[M_FRONT_RIGHT] )
							- (rotor_lifts[M_REAR_LEFT]   + rotor_lifts[M_REAR_RIGHT] ))*  mpos_x;

	sim->torques_bf[YAW]   = (M_FL_DIR*(10.0*rotor_drags[M_FRONT_LEFT]+rotor_inertia[M_FRONT_LEFT])  + M_FR_DIR*(10.0*rotor_drags[M_FRONT_RIGHT]+rotor_inertia[M_FRONT_RIGHT])
							+ M_RL_DIR*(10.0*rotor_drags[M_REAR_LEFT] +rotor_inertia[M_REAR_LEFT])   + M_RR_DIR*(10.0*rotor_drags[M_REAR_RIGHT] +rotor_inertia[M_REAR_RIGHT] ))*  sim->rotor_diameter;
	

	
	sim->lin_forces_bf[X] = -sim->vel_bf[X]*lateral_airspeed* sim->vehicle_drag;  
	sim->lin_forces_bf[Y] = -sim->vel_bf[Y]*lateral_airspeed* sim->vehicle_drag;
	sim->lin_forces_bf[Z] = -(rotor_lifts[M_FRONT_LEFT]+ rotor_lifts[M_FRONT_RIGHT] +rotor_lifts[M_REAR_LEFT] +rotor_lifts[M_REAR_RIGHT]);

}


void forces_from_servos_cross_quad(simulation_model_t *sim, servo_output *servos){
	int i;
	float motor_command[4];
	
/*	motor_command[M_FRONT]= control->thrust + control->rpy[PITCH] + M_FRONT_DIR * control->rpy[YAW];
	motor_command[M_RIGHT] = control->thrust - control->rpy[ROLL] + M_RIGHT_DIR * control->rpy[YAW];
	motor_command[M_REAR] = control->thrust - control->rpy[PITCH] + M_REAR_DIR * control->rpy[YAW];
	motor_command[M_LEFT]  = control->thrust + control->rpy[ROLL] + M_LEFT_DIR * control->rpy[YAW];
	for (i=0; i<4; i++) {
		if (motor_command[i]<MIN_THRUST) motor_command[i]=MIN_THRUST;
		if (motor_command[i]>MAX_THRUST) motor_command[i]=MAX_THRUST;
	}

	for (i=0; i<4; i++) {
		centralData->servos[i].value=SERVO_SCALE*motor_command[i];
	}
	*/
}

void simu_update(simulation_model_t *sim, servo_output *servo_commands, Imu_Data_t *imu) {
	int i;
	central_data_t *central_data;
	double t=get_time();
	sim->dt=(t - sim->last_update);
	sim->last_update=t;
	central_data=get_central_data();
	// compute torques and forces based on servo commands
	#ifdef CONF_DIAG
	forces_from_servos_diag_quad(sim, servo_commands);
	#endif
	#ifdef CONF_CROSS
	forces_from_servos_cross_quad(sim, servo_commands);
	#endif
	
	// integrate torques to get simulated gyro rates (with some damping)
	//for (i=0; i<3; i++) {
	sim->rates_bf[0] = (1.0-0.01*sim->dt)*sim->rates_bf[0] + sim->dt * sim->torques_bf[0] /sim->roll_pitch_momentum;
	sim->rates_bf[1] = (1.0-0.01*sim->dt)*sim->rates_bf[1] + sim->dt * sim->torques_bf[1] /sim->roll_pitch_momentum;
	sim->rates_bf[2] = (1.0-0.01*sim->dt)*sim->rates_bf[2] + sim->dt * sim->torques_bf[2] /sim->yaw_momentum;
	//}
	

	// check altitude - if it is lower than 0, clamp everything (this is in NED, assuming negative altitude)
	if (imu->attitude.localPosition.pos[Z] >0) {
		imu->attitude.vel[Z]=0.0;
		imu->attitude.localPosition.pos[Z]=0.0;

		// simulate "acceleration" caused by contact force with ground, compensating gravity
		for (i=0; i<3; i++) {
			sim->lin_forces_bf[i]+=imu->attitude.up_vec.v[i]*sim->total_mass *GRAVITY;
		}
				
		// slow down... (will make velocity slightly inconsistent until next update cycle, but shouldn't matter much)
		for (i=0; i<3; i++) {
			imu->attitude.vel_bf[i]=0.9*imu->attitude.vel_bf[i];
		}
	}
	imu->raw_channels[GYRO_OFFSET+IMU_X]=sim->rates_bf[0] * imu->raw_scale[GYRO_OFFSET+IMU_X]+imu->raw_bias[GYRO_OFFSET+IMU_X];
	imu->raw_channels[GYRO_OFFSET+IMU_Y]=sim->rates_bf[1] * imu->raw_scale[GYRO_OFFSET+IMU_Y]+imu->raw_bias[GYRO_OFFSET+IMU_Y];
	imu->raw_channels[GYRO_OFFSET+IMU_Z]=sim->rates_bf[2] * imu->raw_scale[GYRO_OFFSET+IMU_Z]+imu->raw_bias[GYRO_OFFSET+IMU_Z];

	imu->raw_channels[ACC_OFFSET+IMU_X]=(sim->lin_forces_bf[0] / sim->total_mass / GRAVITY)*imu->raw_scale[ACC_OFFSET+IMU_X]+imu->raw_bias[ACC_OFFSET+IMU_X];
	imu->raw_channels[ACC_OFFSET+IMU_Y]=(sim->lin_forces_bf[1] / sim->total_mass / GRAVITY)*imu->raw_scale[ACC_OFFSET+IMU_Y]+imu->raw_bias[ACC_OFFSET+IMU_Y];
	imu->raw_channels[ACC_OFFSET+IMU_Z]=(sim->lin_forces_bf[2] / sim->total_mass / GRAVITY)*imu->raw_scale[ACC_OFFSET+IMU_Z]+imu->raw_bias[ACC_OFFSET+IMU_Z];

	imu->dt=sim->dt;
	qfilter(&imu->attitude, &imu->raw_channels, imu->dt, true);
	
	
	
	position_integration(&imu->attitude,imu->dt);


	//position_correction();
	
	
	for (i=0; i<3; i++){
		sim->vel_bf[i]=imu->attitude.vel_bf[i];
		sim->pos[i]=imu->attitude.localPosition.pos[i];		
	}
	

}