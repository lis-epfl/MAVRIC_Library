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
#include "time_keeper.h"
#include "mavlink_communication.h"
#include "print_util.h"

void stabilisation_init(Stabiliser_t * stabiliser, Control_Command_t *command, const mavlink_stream_t* mavlink_stream)
{
	stabiliser->mavlink_stream = mavlink_stream;
	command->mavlink_stream = mavlink_stream;
	
	print_util_dbg_print("Stabilisation init.\n");
}

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
	mavlink_message_t msg;
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack(	rate_stabiliser->mavlink_stream->sysid,
															rate_stabiliser->mavlink_stream->compid,
															&msg,
															time_keeper_get_millis(),
															rate_stabiliser->rpy_controller[0].output,
															rate_stabiliser->rpy_controller[1].output,
															rate_stabiliser->rpy_controller[2].output,
															0 );
	
	mavlink_stream_send(rate_stabiliser->mavlink_stream,&msg);
	
	return TASK_RUN_SUCCESS;
}

task_return_t  stabilisation_send_rpy_rates_error(Stabiliser_t* rate_stabiliser)
{
	mavlink_message_t msg;
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_pack(	rate_stabiliser->mavlink_stream->sysid,
															rate_stabiliser->mavlink_stream->compid,
															&msg,
															time_keeper_get_millis(),
															rate_stabiliser->rpy_controller[0].error,
															rate_stabiliser->rpy_controller[1].error,
															rate_stabiliser->rpy_controller[2].error,
															0 );
	
	mavlink_stream_send(rate_stabiliser->mavlink_stream,&msg);
	
	return TASK_RUN_SUCCESS;
}

task_return_t stabilisation_send_rpy_thrust_setpoint(Control_Command_t* controls)
{
	// Controls output
	mavlink_message_t msg;
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack(	controls->mavlink_stream->sysid,
														controls->mavlink_stream->compid,
														&msg,
														time_keeper_get_millis(),
														controls->rpy[ROLL],
														controls->rpy[PITCH],
														controls->rpy[YAW],
														controls->thrust);
	
	mavlink_stream_send(controls->mavlink_stream,&msg);
	
	return TASK_RUN_SUCCESS;
}