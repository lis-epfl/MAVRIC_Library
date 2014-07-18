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
 * \file stabilisation.c
 *
 * Executing the PID controllers for stabilization
 */


#include "stabilisation.h"

void stabilisation_run(Stabiliser_t *stabiliser, float dt, float errors[]) 
{
	int32_t i;
	for (i = 0; i < 3; i++) 
	{
		stabiliser->output.rpy[i]=	pid_control_update_dt(&(stabiliser->rpy_controller[i]),  errors[i], dt);
	}		
	stabiliser->output.thrust= pid_control_update_dt(&(stabiliser->thrust_controller),  errors[3], dt);
}

task_return_t  stabilisation_send_rpy_speed_thrust_setpoint(Stabiliser_t* rate_stabiliser)
{
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_send(	MAVLINK_COMM_0,
	time_keeper_get_millis(),
	rate_stabiliser->rpy_controller[0].output,
	rate_stabiliser->rpy_controller[1].output,
	rate_stabiliser->rpy_controller[2].output,
	0 );
	return TASK_RUN_SUCCESS;
}

task_return_t  stabilisation_send_rpy_rates_error(Stabiliser_t* rate_stabiliser)
{
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_send(	MAVLINK_COMM_0,
	time_keeper_get_millis(),
	rate_stabiliser->rpy_controller[0].error,
	rate_stabiliser->rpy_controller[1].error,
	rate_stabiliser->rpy_controller[2].error,
	0 );
	return TASK_RUN_SUCCESS;
}