/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file stabilisation.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief Executing the PID controllers for stabilization
 *
 ******************************************************************************/


#include "stabilisation.h"
#include "time_keeper.h"
#include "mavlink_communication.h"
#include "print_util.h"

void stabilisation_init(control_command_t *controls, const mavlink_stream_t* mavlink_stream)
{
	controls->mavlink_stream = mavlink_stream;
	
	controls->control_mode = ATTITUDE_COMMAND_MODE;
	controls->yaw_mode = YAW_RELATIVE;
	
	controls->rpy[ROLL] = 0.0f;
	controls->rpy[PITCH] = 0.0f;
	controls->rpy[YAW] = 0.0f;
	controls->tvel[X] = 0.0f;
	controls->tvel[Y] = 0.0f;
	controls->tvel[Z] = 0.0f;
	controls->theading = 0.0f;
	controls->thrust = -1.0f;
	
	print_util_dbg_print("Stabilisation init.\r\n");
}

void stabilisation_run(stabiliser_t *stabiliser, float dt, float errors[]) 
{
	int32_t i;
	for (i = 0; i < 3; i++) 
	{
		stabiliser->output.rpy[i]=	pid_control_update_dt(&(stabiliser->rpy_controller[i]),  errors[i], dt);
	}		
	stabiliser->output.thrust= pid_control_update_dt(&(stabiliser->thrust_controller),  errors[3], dt);
}

task_return_t  stabilisation_send_rpy_speed_thrust_setpoint(stabiliser_t* stabiliser)
{
	mavlink_message_t msg;
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack(	stabiliser->mavlink_stream->sysid,
															stabiliser->mavlink_stream->compid,
															&msg,
															time_keeper_get_millis(),
															stabiliser->rpy_controller[0].output,
															stabiliser->rpy_controller[1].output,
															stabiliser->rpy_controller[2].output,
															stabiliser->thrust_controller.output );
	
	mavlink_stream_send(stabiliser->mavlink_stream,&msg);
	
	return TASK_RUN_SUCCESS;
}

task_return_t  stabilisation_send_rpy_rates_error(stabiliser_t* stabiliser)
{
	mavlink_message_t msg;
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_pack(	stabiliser->mavlink_stream->sysid,
															stabiliser->mavlink_stream->compid,
															&msg,
															time_keeper_get_millis(),
															stabiliser->rpy_controller[0].error,
															stabiliser->rpy_controller[1].error,
															stabiliser->rpy_controller[2].error,
															stabiliser->thrust_controller.error );
	
	mavlink_stream_send(stabiliser->mavlink_stream,&msg);
	
	return TASK_RUN_SUCCESS;
}

task_return_t stabilisation_send_rpy_thrust_setpoint(control_command_t* controls)
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