/*
 * stabilisation_hybrid.c
 *
 * Created: 13/11/2013 15:46:00
 *  Author: Julien
 */ 

#include "stabilisation_hybrid.h"
#include "central_data.h"
#include "conf_stabilisation_hybrid.h"
#include "quick_trig.h"

central_data_t *centralData;

void stabilisation_hybrid_init(Stabiliser_Stack_hybrid_t* stabiliser_stack)
{
	centralData = central_data_get_pointer_to_struct();
	centralData->run_mode = MOTORS_OFF;
	centralData->controls.control_mode = RATE_COMMAND_MODE;

	*stabiliser_stack = stabiliser_defaults_hybrid;
}

void stabilisation_hybrid_cascade_stabilise_hybrid(Imu_Data_t *imu, position_estimator_t *pos_est, Control_Command_t *control_input)
{
	float rpyt_errors[4];
	Control_Command_t input;
	int i;
	
	// set the controller input
	input = *control_input;

	float target_global[3];
	float target_loc[3];
	float reference_loc[3];

	switch (control_input->control_mode) {
	case VELOCITY_COMMAND_MODE:
		/*
		 * Disabled for now
		 */
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case ATTITUDE_COMMAND_MODE:
		// reference vector	in local frame
		reference_loc[0] = 1.0f;	// front vector
		reference_loc[1] = 0.0f;
		reference_loc[2] = 0.0f;	// norm = 1

		// get target vector in global frame
		target_global[0] = input.rpy[1];
		target_global[1] = input.rpy[2];
		target_global[2] = -1;

		// target vector in local frame
		UQuat_t qtarget = maths_quat_from_vector(&target_global);
		qtarget = maths_quat_global_to_local(imu->attitude.qe, qtarget);
		target_loc[0] = qtarget.v[0];
		target_loc[1] = qtarget.v[1];
		target_loc[2] = qtarget.v[2];
		maths_vector_normalize(target_loc, target_loc);

		// get rotation axis
		float axis[3];
		maths_cross_product(reference_loc, target_loc, axis);
		maths_vector_normalize(axis, axis);

		// get angle
		float angle = acosf(maths_scalar_product(reference_loc, target_loc));
		// float angle = quick_trig_acos(maths_scalar_product(reference_loc, target_loc));
		
		// get errors
		rpyt_errors[0]= input.rpy[0];
		rpyt_errors[1]= axis[1] * angle;
		rpyt_errors[2]= axis[2] * angle;
		
		rpyt_errors[3]= input.thrust;       // no feedback for thrust at this level
		
		// run PID update on all attitude controllers
		stabilisation_run(&centralData->stabiliser_stack.attitude_stabiliser, centralData->imu1.dt, &rpyt_errors);
		
		// use output of attitude controller to set rate setpoints for rate controller 
		input = centralData->stabiliser_stack.attitude_stabiliser.output;
		
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case RATE_COMMAND_MODE: // this level is always run
		// get rate measurements from IMU (filtered angular rates)
		for (i=0; i<3; i++) {
			rpyt_errors[i]= input.rpy[i] - imu->attitude.om[i];
		}
		rpyt_errors[3] = input.thrust ;  // no feedback for thrust at this level
		
		// run PID update on all rate controllers
		stabilisation_run(&centralData->stabiliser_stack.rate_stabiliser, centralData->imu1.dt, &rpyt_errors );
	}

	// mix to servos 
	stabilisation_hybrid_mix_to_servos_xwing(&centralData->stabiliser_stack.rate_stabiliser.output);
}

void stabilisation_hybrid_mix_to_servos_xwing(Control_Command_t *control)
{
	int i;
	float motor_command;
	float servo_command[4];

	// mix
	motor_command = control->thrust;
	servo_command[FLAP_FRONT] = FLAP_FRONT_DIR * ( + control->rpy[ROLL] 
												   + control->rpy[YAW] );
	servo_command[FLAP_RIGHT] = FLAP_RIGHT_DIR * ( + control->rpy[ROLL] 
												   + control->rpy[PITCH] ); 
	servo_command[FLAP_REAR] = FLAP_REAR_DIR * ( + control->rpy[ROLL] 
												 - control->rpy[YAW] );
	servo_command[FLAP_LEFT] = FLAP_LEFT_DIR * ( + control->rpy[ROLL] 
												 - control->rpy[PITCH] ); 

	// maths_clip
	if (motor_command < MIN_THRUST) motor_command = MIN_THRUST;
	if (motor_command > MAX_THRUST) motor_command = MAX_THRUST;
	for (i=0; i<4; i++) 
	{
		if (servo_command[i] < MIN_DEFLECTION) servo_command[i] = MIN_DEFLECTION;
		if (servo_command[i] > MAX_DEFLECTION) servo_command[i] = MAX_DEFLECTION;
	}

	// scale and write values
	centralData->servos[FLAP_FRONT].value = SERVO_NEUTRAL + SERVO_AMPLITUDE * servo_command[FLAP_FRONT];
	centralData->servos[FLAP_RIGHT].value = SERVO_NEUTRAL + SERVO_AMPLITUDE * servo_command[FLAP_RIGHT];
	centralData->servos[FLAP_REAR].value = SERVO_NEUTRAL + SERVO_AMPLITUDE * servo_command[FLAP_REAR];
	centralData->servos[FLAP_LEFT].value = SERVO_NEUTRAL + SERVO_AMPLITUDE * servo_command[FLAP_LEFT];
	centralData->servos[MAIN_ENGINE].value = SERVO_SCALE * motor_command;
}