/*
 * stabilisation_copter.c
 *
 * Created: 07/06/2012 21:08:01
 *  Author: Felix Schill
 */ 

#include "stabilisation_copter.h"
#include "conf_stabilisation_copter.h"
#include "central_data.h"

central_data_t *centralData;


void init_stabilisation_copter(Stabiliser_Stack_copter_t* stabiliser_stack)
{
	centralData = get_central_data();
	centralData->controls.run_mode = MOTORS_OFF;
	centralData->controls.control_mode = ATTITUDE_COMMAND_MODE;
	centralData->controls.yaw_mode = YAW_RELATIVE;

	*stabiliser_stack = stabiliser_defaults_copter;
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