/*
 * stabilisation.c
 *
 * Created: 07/06/2012 21:08:01
 *  Author: sfx
 */ 

#include "stabilisation.h"
#include "time_keeper.h"
#include "servo_pwm.h"
#include "print_util.h"
#include "boardsupport.h"

Stabiliser_t rate_stabiliser, attitude_stabiliser;

board_hardware_t *board;

Stabiliser_t* get_rate_stabiliser() { return &rate_stabiliser;}
Stabiliser_t* get_attitude_stabiliser() { return &attitude_stabiliser;}

void init_rate_stabilisation(Stabiliser_t *stabiliser) {
	int i=0;
	// initialise roll and pitch controllers
	for (i=0; i<2; i++) {
		(stabiliser->rpy_controller[i]).p_gain=0.165;
		(stabiliser->rpy_controller[i]).last_update=get_time_ticks();	
		(stabiliser->rpy_controller[i]).clip_min=-0.9;
		(stabiliser->rpy_controller[i]).clip_max= 0.9;
		initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.2, 0.4, 0.5);
		initInt(&((stabiliser->rpy_controller)[i].integrator),0.9, 0.5, 0.65);
	}	
	// initialise yaw controller
	i=2;
	(stabiliser->rpy_controller)[i].p_gain=0.1;
	(stabiliser->rpy_controller)[i].last_update=get_time_ticks();	
	(stabiliser->rpy_controller)[i].clip_min=-0.9;
	(stabiliser->rpy_controller)[i].clip_max= 0.9;
	initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.02, 0.4, 0.5);
	initInt(&((stabiliser->rpy_controller)[i].integrator),0.0, 0.0, 0.1);
	
}

void init_angle_stabilisation(Stabiliser_t *stabiliser) {
	int i=0;
	// initialise roll and pitch controllers
	for (i=0; i<2; i++) {
		(stabiliser->rpy_controller[i]).p_gain=1.5;
		(stabiliser->rpy_controller[i]).last_update=get_time_ticks();	
		(stabiliser->rpy_controller[i]).clip_min=-1.2;
		(stabiliser->rpy_controller[i]).clip_max= 1.2;
		initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.00, 0.5, 0.1); // 0.05, 0.5, 0.05
		initInt(&((stabiliser->rpy_controller)[i].integrator),0.0, 0.0, 0.0);
	}	
	// initialise yaw controller
	i=2;
	(stabiliser->rpy_controller)[i].p_gain=2.5;
	(stabiliser->rpy_controller)[i].last_update=get_time_ticks();	
	(stabiliser->rpy_controller)[i].clip_min=-1.0;
	(stabiliser->rpy_controller)[i].clip_max= 1.0;
	initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.0, 0.5, 0.5);
	initInt(&((stabiliser->rpy_controller)[i].integrator),0.0, 0.0, 0.0);
	
}


void init_stabilisation() {
	board=get_board_hardware();
	board->controls.run_mode=MOTORS_OFF;
	board->controls.control_mode=ATTITUDE_COMMAND_MODE;
	init_rate_stabilisation(&rate_stabiliser);
	init_angle_stabilisation(&attitude_stabiliser);
}

void stabilise(Stabiliser_t *stabiliser, float *rpy_sensor_values, Control_Command_t *control_input) {
	int i;
	// run the pid controllers
	//stabiliser->output.rpy[0]=-0.05*imu->attitude.om[1] + 
	//	pid_update(&(stabiliser->rpy_controller[0]), control_input->rpy[0], imu->attitude.up_vec.v[0]);
	//stabiliser->output.rpy[1]= 0.05*imu->attitude.om[0] + 
	//	pid_update(&(stabiliser->rpy_controller[1]), control_input->rpy[1], imu->attitude.up_vec.v[1]);
	for (i=0; i<3; i++) {
		stabiliser->output.rpy[i]=	pid_update(&(stabiliser->rpy_controller[i]),  rpy_sensor_values[i], control_input->rpy[i]);
	}		
	stabiliser->output.thrust=control_input->thrust;
}




void quad_stabilise(Imu_Data_t *imu , Control_Command_t *control_input) {
	float rpy_angles[3];
	float rpy_rates[3];
	Control_Command_t *rate_input;
	int i;
	
	if (control_input->run_mode==MOTORS_OFF) {
		
		set_servos(&(servo_failsafe));
		return;
	}
	
	if (control_input->control_mode==ATTITUDE_COMMAND_MODE) {
		rpy_angles[0]=-imu->attitude.up_vec.v[1];
		rpy_angles[1]= imu->attitude.up_vec.v[0];
		rpy_angles[2]= 0;
		stabilise(&attitude_stabiliser, &rpy_angles, control_input);
		rate_input=&attitude_stabiliser.output;
	} else {
		rate_input=control_input;
		for (i=0; i<3; i++) {
			rate_input->rpy[i]=control_input->rpy[i] * 1.5;
		}		
	}
	rpy_rates[0]= imu->attitude.om[0];
	rpy_rates[1]= imu->attitude.om[1];
	rpy_rates[2]= imu->attitude.om[2];
	stabilise(&rate_stabiliser, &rpy_rates, rate_input);

	#ifdef CONF_DIAG
	mix_to_servos_diag_quad(&rate_stabiliser.output);
	#else
	#ifdef CONF_CROSS
	mix_to_servos_cross_quad(&rate_stabiliser.output);
	#endif
	#endif
	// send values to servo outputs
	set_servos(&(board->servos));
	
}

void mix_to_servos_diag_quad(Control_Command_t *control){
	int i;
	float motor_command[4];
	
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
}


void mix_to_servos_cross_quad(Control_Command_t *control){
	int i;
	float motor_command[4];
	
	motor_command[M_FRONT]= control->thrust + control->rpy[PITCH] + M_FRONT_DIR * control->rpy[YAW];
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
	
}