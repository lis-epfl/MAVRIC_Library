/*
 * stabilisation_copter.c
 *
 * Created: 07/06/2012 21:08:01
 *  Author: Felix Schill
 */ 

#include "stabilisation_copter.h"
#include "time_keeper.h"
#include "servo_pwm.h"
#include "print_util.h"
#include "central_data.h"

central_data_t *centralData;

void init_rate_stabilisation(Stabiliser_t *stabiliser) {
	int i=0;
	// initialise roll and pitch controllers
	for (i=0; i<2; i++) {
		(stabiliser->rpy_controller[i]).p_gain = 0.15;
		(stabiliser->rpy_controller[i]).last_update = get_time_ticks();	
		(stabiliser->rpy_controller[i]).clip_min = -0.9;
		(stabiliser->rpy_controller[i]).clip_max = 0.9;
		(stabiliser->rpy_controller[i]).soft_zone_width = 0.0;
		initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.2, 0.4, 0.5);
		initInt(&((stabiliser->rpy_controller)[i].integrator),0.5, 1.0, 0.65);
	}	
	// initialise yaw controller
	i = 2;
	(stabiliser->rpy_controller)[i].p_gain = 0.4;
	(stabiliser->rpy_controller)[i].last_update = get_time_ticks();	
	(stabiliser->rpy_controller)[i].clip_min = -0.9;
	(stabiliser->rpy_controller)[i].clip_max = 0.9;
	(stabiliser->rpy_controller[i]).soft_zone_width = 0.0;
	initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.0, 0.4, 0.5);
	initInt(&((stabiliser->rpy_controller)[i].integrator),0.5, 0.2, 0.1);
	
	// initialise thrust controller
	stabiliser->thrust_controller = passthroughController();
}

void init_angle_stabilisation(Stabiliser_t *stabiliser) {
	int i = 0;
	// initialise roll and pitch controllers
	for (i=0; i<2; i++) {
		(stabiliser->rpy_controller[i]).p_gain = 1.5;
		(stabiliser->rpy_controller[i]).last_update = get_time_ticks();	
		(stabiliser->rpy_controller[i]).clip_min = -1.2;
		(stabiliser->rpy_controller[i]).clip_max = 1.2;
		(stabiliser->rpy_controller[i]).soft_zone_width = 0.0;
		initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.00, 0.5, 0.1); // 0.05, 0.5, 0.05
		initInt(&((stabiliser->rpy_controller)[i].integrator),0.0, 0.0, 0.0);
	}	
	// initialise yaw controller
	i = 2;
	(stabiliser->rpy_controller)[i].p_gain = 1.5;
	(stabiliser->rpy_controller)[i].last_update = get_time_ticks();	
	(stabiliser->rpy_controller)[i].clip_min = -1.0;
	(stabiliser->rpy_controller)[i].clip_max = 1.0;
	(stabiliser->rpy_controller[i]).soft_zone_width = 0.0;

	initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.0, 0.5, 0.5);
	initInt(&((stabiliser->rpy_controller)[i].integrator), 0.0, 0.0, 0.0);

	//initialise thrust controller
	stabiliser->thrust_controller = passthroughController();
	
}

void init_velocity_stabilisation(Stabiliser_t * stabiliser) {
	int i = 0;
	// initialise roll velocity
	(stabiliser->rpy_controller[i]).p_gain = 0.2; //0.1
	(stabiliser->rpy_controller[i]).last_update = get_time_ticks();
	(stabiliser->rpy_controller[i]).clip_min = -0.5; //-0.6
	(stabiliser->rpy_controller[i]).clip_max = 0.5; //0.6
	
	(stabiliser->rpy_controller[i]).soft_zone_width = 0.3; //region of lowered error input gain to ignore noise close to target point

	initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.0, 0.5, 0.5); // 0.1 0.5 0.5
	initInt(&((stabiliser->rpy_controller)[i].integrator),0.0, 0.0, 0.3); // 1.0 0.3 0.3
	
	// initialise pitch velocity
	i = 1;
	(stabiliser->rpy_controller[i]).p_gain = 0.2; //0.1
	(stabiliser->rpy_controller[i]).last_update = get_time_ticks();
	(stabiliser->rpy_controller[i]).clip_min = -0.5; //-0.6
	(stabiliser->rpy_controller[i]).clip_max = 0.5; //0.6
	(stabiliser->rpy_controller[i]).soft_zone_width = 0.3; //region of lowered error input gain to ignore noise close to target point

	initDiff(&((stabiliser->rpy_controller)[i].differentiator), 0.0, 0.5, 0.5); // 0.1 0.5 0.5
	initInt(&((stabiliser->rpy_controller)[i].integrator), 0.0, 0.0, 0.3); // 1.0 0.3 0.3
	
	// initialise yaw controller
	stabiliser->rpy_controller[2] = passthroughController();
	
	// initialise z velocity
	(stabiliser->thrust_controller).p_gain = 0.4; //0.3
	(stabiliser->thrust_controller).last_update = get_time_ticks();
	(stabiliser->thrust_controller).clip_min = -0.9; //-0.9
	(stabiliser->thrust_controller).clip_max = 0.65; // 0.9
	(stabiliser->thrust_controller).soft_zone_width = 0.2; // region of lowered error input gain to ignore noise close to target point
	initDiff(&((stabiliser->thrust_controller).differentiator), 0.5, 0.95, 1.0); // 0.1 0.5 0.2
	initInt(&((stabiliser->thrust_controller).integrator), 1.5, 1.0, 1.0); // 1.0 1.0 0.5
	
	
}

void init_stabilisation_copter(Stabiliser_Stack_copter_t* stabiliser_stack)
{
	centralData = get_central_data();
	centralData->controls.run_mode = MOTORS_OFF;
	centralData->controls.control_mode = ATTITUDE_COMMAND_MODE;
	centralData->controls.yaw_mode = YAW_RELATIVE;

	stabiliser_stack->yaw_coordination_velocity = 1.5;

	init_rate_stabilisation(&stabiliser_stack->rate_stabiliser);
	init_angle_stabilisation(&stabiliser_stack->attitude_stabiliser);
	init_velocity_stabilisation(&stabiliser_stack->velocity_stabiliser);
}


void cascade_stabilise_copter(Imu_Data_t *imu, position_estimator_t *pos_est, Control_Command_t *control_input) {
	float rpyt_errors[4];
	Control_Command_t input;
	int i;
	
	// set the controller input
	input=*control_input;

	switch (control_input->control_mode) {
	case VELOCITY_COMMAND_MODE:
		rpyt_errors[ROLL]  =   input.tvel[Y] - pos_est->vel_bf[Y];     // map y-axis error to roll axis
		rpyt_errors[PITCH] = -(input.tvel[X] - pos_est->vel_bf[X]);   // map x axis error to pitch axis
		rpyt_errors[3]     = -(input.tvel[Z] - pos_est->vel[Z]);      // attention - input z-axis maps to thrust input!
		

		if (control_input->yaw_mode == YAW_COORDINATED)  {
			float rel_heading = atan2(pos_est->vel_bf[Y], pos_est->vel_bf[X]);
			float current_velocity_sqr=SQR(pos_est->vel_bf[X])+SQR(pos_est->vel_bf[Y]);
			//float blend_func=0.5*(sigmoid(4.0*(current_velocity_sqr - yaw_coordination_velocity))+1.0);
			//blend_func=1.0;
			if (current_velocity_sqr > SQR(centralData->stabiliser_stack.yaw_coordination_velocity)) {
				input.rpy[YAW]+=sigmoid(3.0*rel_heading);
			} else {
				//input.rpy[YAW]=input.theading;
			}
		}

		rpyt_errors[YAW]= input.rpy[YAW];
		
		// run PID update on all velocity controllers
		stabilise(&centralData->stabiliser_stack.velocity_stabiliser, centralData->imu1.dt, rpyt_errors);
		
		//velocity_stabiliser.output.thrust = f_min(velocity_stabiliser.output.thrust,control_input->thrust);
		
		centralData->stabiliser_stack.velocity_stabiliser.output.thrust += THRUST_HOVER_POINT;
		centralData->stabiliser_stack.velocity_stabiliser.output.theading = input.theading;
		input = centralData->stabiliser_stack.velocity_stabiliser.output;
	
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case ATTITUDE_COMMAND_MODE:
		// run absolute attitude controller
		rpyt_errors[0]= input.rpy[0] - (-imu->attitude.up_vec.v[1] ); 
		rpyt_errors[1]= input.rpy[1] - imu->attitude.up_vec.v[0];
		
		rpyt_errors[2]= input.rpy[2];
		
		if ((control_input->yaw_mode == YAW_ABSOLUTE) ) {
			rpyt_errors[2] +=calc_smaller_angle(input.theading- pos_est->localPosition.heading);
		}
		rpyt_errors[3]= input.thrust;       // no feedback for thrust at this level
		
		// run PID update on all attitude controllers
		stabilise(&centralData->stabiliser_stack.attitude_stabiliser, centralData->imu1.dt, &rpyt_errors);
		
		// use output of attitude controller to set rate setpoints for rate controller 
		input = centralData->stabiliser_stack.attitude_stabiliser.output;
	
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case RATE_COMMAND_MODE: // this level is always run
		// get rate measurements from IMU (filtered angular rates)
		for (i=0; i<3; i++) {
			rpyt_errors[i]= input.rpy[i]- imu->attitude.om[i];
		}
		rpyt_errors[3] = input.thrust ;  // no feedback for thrust at this level
		
		// run PID update on all rate controllers
		stabilise(&centralData->stabiliser_stack.rate_stabiliser, centralData->imu1.dt, &rpyt_errors );
	}
	
	// mix to servo outputs depending on configuration
	#ifdef CONF_DIAG
	mix_to_servos_diag_quad(&centralData->stabiliser_stack.rate_stabiliser.output);
	#else
	#ifdef CONF_CROSS
	mix_to_servos_cross_quad(&centralData->stabiliser_stack.rate_stabiliser.output);
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