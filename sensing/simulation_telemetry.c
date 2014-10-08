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
 * \file simulation_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the simulation module
 *
 ******************************************************************************/


#include "simulation_telemetry.h"
#include "time_keeper.h"
#include "print_util.h"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Changes between simulation to and from reality
 *
 * \param	sim				The pointer to the simulation model structure
 * \param	packet			The pointer to the decoded MAVLink command long message
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t simulation_telemetry_set_new_home_position(simulation_model_t *sim, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t simulation_telemetry_set_new_home_position(simulation_model_t *sim, mavlink_command_long_t* packet)
{
	mav_result_t result;
	
	if (packet->param1 == 1)
	{
		// Set new home position to actual position
		print_util_dbg_print("Set new home location to actual position.\r\n");
		sim->local_position.origin = coord_conventions_local_to_global_position(sim->local_position);

		print_util_dbg_print("New Home location: (");
		print_util_dbg_print_num(sim->local_position.origin.latitude * 10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(sim->local_position.origin.longitude * 10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(sim->local_position.origin.altitude * 1000.0f,10);
		print_util_dbg_print(")\r\n");
	}
	else
	{
		// Set new home position from msg
		print_util_dbg_print("Set new home location.\r\n");

		sim->local_position.origin.latitude = packet->param5;
		sim->local_position.origin.longitude = packet->param6;
		sim->local_position.origin.altitude = packet->param7;

		print_util_dbg_print("New Home location: (");
		print_util_dbg_print_num(sim->local_position.origin.latitude * 10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(sim->local_position.origin.longitude * 10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(sim->local_position.origin.altitude * 1000.0f,10);
		print_util_dbg_print(")\r\n");
	}

	*sim->nav_plan_active = false;
	
	result = MAV_RESULT_ACCEPTED;
	
	return result;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void simulation_telemetry_init(simulation_model_t* sim, mavlink_message_handler_t* message_handler)
{
	mavlink_message_handler_cmd_callback_t callbackcmd;
		
	callbackcmd.command_id    = MAV_CMD_DO_SET_HOME; // 179
	callbackcmd.sysid_filter  = MAV_SYS_ID_ALL;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER;
	callbackcmd.function      = (mavlink_cmd_callback_function_t)	&simulation_telemetry_set_new_home_position;
	callbackcmd.module_struct =										sim;
	mavlink_message_handler_add_cmd_callback(message_handler, &callbackcmd);
}


void simulation_telemetry_send_state(const simulation_model_t* sim_model, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	aero_attitude_t aero_attitude;
	aero_attitude = coord_conventions_quat_to_aero(sim_model->ahrs.qe);

	global_position_t gpos = coord_conventions_local_to_global_position(sim_model->local_position);
	
	mavlink_msg_hil_state_pack(	mavlink_stream->sysid,
								mavlink_stream->compid,
								msg,
								time_keeper_get_micros(),
								aero_attitude.rpy[0],
								aero_attitude.rpy[1],
								aero_attitude.rpy[2],
								sim_model->rates_bf[ROLL],
								sim_model->rates_bf[PITCH],
								sim_model->rates_bf[YAW],
								gpos.latitude * 10000000,
								gpos.longitude * 10000000,
								gpos.altitude * 1000.0f,
								100 * sim_model->vel[X],
								100 * sim_model->vel[Y],
								100 * sim_model->vel[Z],
								1000 * sim_model->lin_forces_bf[0],
								1000 * sim_model->lin_forces_bf[1],
								1000 * sim_model->lin_forces_bf[2] 	);
}

void simulation_telemetry_send_quaternions(const simulation_model_t *sim_model, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	aero_attitude_t aero_attitude;
	aero_attitude = coord_conventions_quat_to_aero(sim_model->ahrs.qe);

	global_position_t gpos = coord_conventions_local_to_global_position(sim_model->local_position);

	mavlink_msg_hil_state_quaternion_pack(	mavlink_stream->sysid,
											mavlink_stream->compid,
											msg,
											time_keeper_get_micros(),
											(float*) &sim_model->ahrs.qe,
											aero_attitude.rpy[ROLL],
											aero_attitude.rpy[PITCH],
											aero_attitude.rpy[YAW],
											gpos.latitude * 10000000,
											gpos.longitude * 10000000,
											gpos.altitude * 1000.0f,
											100 * sim_model->vel[X],
											100 * sim_model->vel[Y],
											100 * sim_model->vel[Z],
											100 * vectors_norm(sim_model->vel),
											0.0f,
											sim_model->ahrs.linear_acc[X],
											sim_model->ahrs.linear_acc[Y],
											sim_model->ahrs.linear_acc[Z]	);
}