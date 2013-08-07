/*
 * simulation.c
 *
 * Created: 06/08/2013 17:02:56
 *  Author: sfx
 */ 

#include "conf_sim_model.h"
#include "time_keeper.h"


void init_simulation(simulation_model_t *sim) {
	int i;
	(*sim)=vehicle_model_parameters;
	for (i=0; i<3; i++) {
		sim->rates_bf[i]=0;
		sim->torques_bf[i]=0;
		sim->lin_forces_bf[i]=0;
		
	}
	for (i=0; i<ROTORCOUNT; i++) {
		sim->rotorspeeds[i]=0.0;			
	}
	sim->last_update=get_time();
	sim->dt=0.01;
}



// inverse function of mix_to_servos in stabilisation to recover torques and forces

void forces_from_servos_diag_quad(simulation_model_t *sim, servo_output *servos){
	int i;
	float motor_command[4];
	double t=get_time();
	sim->dt=(t - sim->last_update);
	sim->last_update=t;

	for (i=0; i<4; i++) {
		motor_command[i]=servos[i].value/SERVO_SCALE;
		sim->rotorspeeds[i]=(1.0-sim->dt*sim->rotor_lpf) * sim->rotorspeeds[i] + (sim->dt*sim->rotor_lpf) * (motor_command[i]-sim->rotor_rpm_offset) * sim->rotor_rpm_gain;
	}
/*
	motor_command[M_FRONT_RIGHT]= control->thrust + (-control->rpy[ROLL] + control->rpy[PITCH]) + M_FR_DIR * control->rpy[YAW];
	motor_command[M_FRONT_LEFT] = control->thrust + ( control->rpy[ROLL] + control->rpy[PITCH]) + M_FL_DIR * control->rpy[YAW];
	motor_command[M_REAR_RIGHT] = control->thrust + (-control->rpy[ROLL] - control->rpy[PITCH]) + M_RR_DIR * control->rpy[YAW];
	motor_command[M_REAR_LEFT]  = control->thrust + ( control->rpy[ROLL] - control->rpy[PITCH]) + M_RL_DIR * control->rpy[YAW];
	for (i=0; i<4; i++) {
		if (motor_command[i]<MIN_THRUST) motor_command[i]=MIN_THRUST;
		if (motor_command[i]>MAX_THRUST) motor_command[i]=MAX_THRUST;
	}

	for (i=0; i<4; i++) {
		board->servos[i].value=SERVO_SCALE*motor_command[i];
	}
*/
}


void rates_from_servos_cross_quad(simulation_model_t *sim, servo_output *servos){
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
		board->servos[i].value=SERVO_SCALE*motor_command[i];
	}
	*/
}