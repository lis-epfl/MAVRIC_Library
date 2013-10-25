/*
 * stabilisation.c
 *
 * Created: 07/06/2012 21:08:01
 *  Author: Felix Schill
 */ 

#include "stabilisation.h"
#include "time_keeper.h"
#include "servo_pwm.h"
#include "print_util.h"
#include "central_data.h"

Stabiliser_t rate_stabiliser, attitude_stabiliser, velocity_stabiliser;

central_data_t *centralData;

Stabiliser_t* get_rate_stabiliser() { return &rate_stabiliser;}
Stabiliser_t* get_attitude_stabiliser() { return &attitude_stabiliser;}
Stabiliser_t* get_velocity_stabiliser() { return &velocity_stabiliser;}

void init_rate_stabilisation(Stabiliser_t *stabiliser) {
	int i=0;
	// initialise roll and pitch controllers
	for (i=0; i<2; i++) {
		(stabiliser->rpy_controller[i]).p_gain=0.15;
		(stabiliser->rpy_controller[i]).last_update=get_time_ticks();	
		(stabiliser->rpy_controller[i]).clip_min=-0.9;
		(stabiliser->rpy_controller[i]).clip_max= 0.9;
		initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.2, 0.4, 0.5);
		initInt(&((stabiliser->rpy_controller)[i].integrator),0.5, 1.0, 0.65);
	}	
	// initialise yaw controller
	i=2;
	(stabiliser->rpy_controller)[i].p_gain=0.5;
	(stabiliser->rpy_controller)[i].last_update=get_time_ticks();	
	(stabiliser->rpy_controller)[i].clip_min=-0.9;
	(stabiliser->rpy_controller)[i].clip_max= 0.9;
	initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.0, 0.4, 0.5);
	initInt(&((stabiliser->rpy_controller)[i].integrator),0.5, 0.2, 0.1);
	
	// initialise thrust controller
	stabiliser->thrust_controller=passthroughController();
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

	//initialise thrust controller
	stabiliser->thrust_controller=passthroughController();
	
}

void init_velocity_stabilisation(Stabiliser_t * stabiliser) {
	int i = 0;
	// initialise roll velocity
	(stabiliser->rpy_controller[i]).p_gain=0.2; //0.1
	(stabiliser->rpy_controller[i]).last_update=get_time_ticks();
	(stabiliser->rpy_controller[i]).clip_min=-0.5; //-0.6
	(stabiliser->rpy_controller[i]).clip_max= 0.5; //0.6
	initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.0, 0.5, 0.5); // 0.1 0.5 0.5
	initInt(&((stabiliser->rpy_controller)[i].integrator),0.0, 0.0, 0.3); // 1.0 0.3 0.3
	
	// initialise pitch velocity
	i = 1;
	(stabiliser->rpy_controller[i]).p_gain=0.2; //0.1
	(stabiliser->rpy_controller[i]).last_update=get_time_ticks();
	(stabiliser->rpy_controller[i]).clip_min=-0.5; //-0.6
	(stabiliser->rpy_controller[i]).clip_max= 0.5; //0.6
	initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.0, 0.5, 0.5); // 0.1 0.5 0.5
	initInt(&((stabiliser->rpy_controller)[i].integrator),0.0, 0.0, 0.3); // 1.0 0.3 0.3
	
	// initialise yaw controller
	stabiliser->rpy_controller[2]=passthroughController();

	// initialise z velocity
	(stabiliser->thrust_controller).p_gain=0.5; //0.3
	(stabiliser->thrust_controller).last_update=get_time_ticks();
	(stabiliser->thrust_controller).clip_min=-0.9; //-0.9
	(stabiliser->thrust_controller).clip_max= 0.65; // 0.9
	initDiff(&((stabiliser->thrust_controller).differentiator), 0.0, 0.5, 0.2); // 0.1 0.5 0.2
	initInt(&((stabiliser->thrust_controller).integrator),3.0, 1.0, 1.0); // 1.0 1.0 0.5
}

void init_stabilisation() {
	//board=get_board_hardware();
	centralData = get_central_data();
	centralData->controls.run_mode=MOTORS_OFF;
	centralData->controls.control_mode=ATTITUDE_COMMAND_MODE_REL_YAW;
	init_rate_stabilisation(&rate_stabiliser);
	init_angle_stabilisation(&attitude_stabiliser);
	init_velocity_stabilisation(&velocity_stabiliser);
}

void stabilise(Stabiliser_t *stabiliser, float *errors) {
	int i;

	for (i=0; i<3; i++) {
		// 
		
		stabiliser->output.rpy[i]=	pid_update_dt(&(stabiliser->rpy_controller[i]),  errors[i], centralData->imu1.dt);
	}		
	stabiliser->output.thrust= pid_update_dt(&(stabiliser->thrust_controller),  errors[3], centralData->imu1.dt);

	//dbg_putfloat(stabiliser->output.thrust, 3); dbg_print("\n");
}




void quad_stabilise(Imu_Data_t *imu, position_estimator_t *pos_est, Control_Command_t *control_input) {
	float rpyt_errors[4];
	Control_Command_t input;
	int i;
	
	input=*control_input;

	switch (control_input->control_mode) {
	case VELOCITY_COMMAND_MODE:
		rpyt_errors[ROLL] = input.tvel[1] - pos_est->vel_bf[1];
		rpyt_errors[PITCH]=-(input.tvel[0] - pos_est->vel_bf[0]); 
		rpyt_errors[3]    =  -(input.tvel[2] - pos_est->vel[2]);
		
		rpyt_errors[YAW]= input.rpy[YAW];
		stabilise(&velocity_stabiliser, &rpyt_errors);
		
		//velocity_stabiliser.output.thrust = f_min(velocity_stabiliser.output.thrust,control_input->thrust);
		
		velocity_stabiliser.output.thrust += THRUST_HOVER_POINT;
		input=velocity_stabiliser.output;
	
	// -- no break here  - we want to run the lower level modes as well! -- 
	case ATTITUDE_COMMAND_MODE_ABS_YAW:
	case ATTITUDE_COMMAND_MODE_REL_YAW:
		// run absolute attitude controller
		rpyt_errors[0]= input.rpy[0] - (-imu->attitude.up_vec.v[1] ); 
		rpyt_errors[1]= input.rpy[1] - imu->attitude.up_vec.v[0];
		
		rpyt_errors[2]= input.rpy[2];
		if ((control_input->control_mode == ATTITUDE_COMMAND_MODE_ABS_YAW )) {
			rpyt_errors[2] =calc_smaller_angle(rpyt_errors[2]- pos_est->localPosition.heading);
			
		}
		rpyt_errors[3]= input.thrust;       // no feedback for thrust at this level
		
		stabilise(&attitude_stabiliser, &rpyt_errors);
		// use output of attitude controller to set rate setpoints for rate controller 
		input=attitude_stabiliser.output;
	
	// -- no break here  - we want to run the lower level modes as well! -- 
	case RATE_COMMAND_MODE:
		// get rate measurements from IMU (filtered angular rates)
		for (i=0; i<3; i++) {
			rpyt_errors[i]= input.rpy[i]- imu->attitude.om[i];
		}
		rpyt_errors[3] = input.thrust ;  // no feedback for thrust at this level
		// run rate stabiliser
		stabilise(&rate_stabiliser, &rpyt_errors );
	}
	// mix to servo outputs depending on configuration
	#ifdef CONF_DIAG
	mix_to_servos_diag_quad(&rate_stabiliser.output);
	#else
	#ifdef CONF_CROSS
	mix_to_servos_cross_quad(&rate_stabiliser.output);
	#endif
	#endif
	
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
		centralData->servos[i].value=SERVO_SCALE*motor_command[i];
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
		centralData->servos[i].value=SERVO_SCALE*motor_command[i];
	}
	
}