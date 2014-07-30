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
 * \file attitude_controller_p2.c
 *
 * A simple quaternion based proportionnal squared controller for attitude control. It takes as input the desired attitude, the estimated attitude and the gyro rates.
 * The control scheme is the following:
 * - first the controller computes local roll pitch and yaw angular errors ( \f$ qerror \f$ ) using quaternion formulation
 * - angular errors are multiplied by a proportional gain \f$ P_{q} \f$
 * - feedback from the gyro is added to the output with gain \f$ P_{g} \f$
 * 
 * in short:
 * \f$  
 * 			output = - (P_{q} . qerror) - (P_{g} . gyro)  
 * \f$
 */


#include "attitude_controller_p2.h"

void attitude_controller_p2_init(attitude_controller_p2_t* controller, const attitude_controller_p2_conf_t* config, const attitude_command_t* attitude_command, const ahrs_t* ahrs)
{
	// Init dependencies
	controller->attitude_command = attitude_command;
	controller->ahrs 			 = ahrs;
	
	// Init attitude error estimator
	attitude_error_estimator_init(&controller->attitude_error_estimator, ahrs);

	// Init gains
	controller->p_gain_angle[0] = config->p_gain_angle[0]; 
	controller->p_gain_angle[1] = config->p_gain_angle[1];
	controller->p_gain_angle[2] = config->p_gain_angle[2];
	controller->p_gain_rate[0]  = config->p_gain_rate[0];
	controller->p_gain_rate[1]  = config->p_gain_rate[1];
	controller->p_gain_rate[2]  = config->p_gain_rate[2];

	// Init ouput
	controller->output[0] = 0.0f;
	controller->output[1] = 0.0f;
	controller->output[2] = 0.0f;
}


void attitude_controller_p2_update(attitude_controller_p2_t* controller)
{
	float errors[3];
	float rates[3];

	// Get attitude command
	switch ( controller->attitude_command->mode )
	{
		case ATTITUDE_COMMAND_MODE_QUATERNION:
			attitude_error_estimator_set_quat_ref(	&controller->attitude_error_estimator,
													controller->attitude_command->quat );
			break;

		case ATTITUDE_COMMAND_MODE_RPY:
			attitude_error_estimator_set_quat_ref_from_rpy( &controller->attitude_error_estimator,
															controller->attitude_command->rpy );
			break;
	}

	// Get local angular errors
	attitude_error_estimator_update( &controller->attitude_error_estimator );
	errors[0] = controller->attitude_error_estimator.rpy_errors[0];
	errors[1] = controller->attitude_error_estimator.rpy_errors[1];
	errors[2] = controller->attitude_error_estimator.rpy_errors[2];

	// Get gyro rates
	rates[0] = controller->ahrs->angular_speed[0];
	rates[1] = controller->ahrs->angular_speed[1];
	rates[2] = controller->ahrs->angular_speed[2];

	// Compute outputs
	controller->output[0] = - controller->p_gain_angle[0] * errors[0] - controller->p_gain_rate[0] * rates[0];
	controller->output[1] = - controller->p_gain_angle[1] * errors[1] - controller->p_gain_rate[1] * rates[1];
	controller->output[2] = - controller->p_gain_angle[2] * errors[2] - controller->p_gain_rate[2] * rates[2];
}
