


#include "remote_communication.h"
#include "time_keeper.h"
#include "spektrum.h"


void remote_communication_init(remote_communication_t* remote_communication, remote_t* remote, mavlink_stream_t* mavlink_stream, mavlink_message_handler_t *mavlink_handler)
{
	remote_communication->remote = remote;
	remote_communication->mavlink_stream = mavlink_stream;
	
	
	mavlink_message_handler_cmd_callback_t callbackcmd;
		
	callbackcmd.command_id    = MAV_CMD_DO_JUMP;//MAV_CMD_START_RX_PAIR; // 500
	callbackcmd.sysid_filter  = MAV_SYS_ID_ALL;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL;
	callbackcmd.function      = (mavlink_cmd_callback_function_t)	&spektrum_satellite_bind;
	callbackcmd.module_struct =										remote->sat;
	mavlink_message_handler_add_cmd_callback(mavlink_handler, &callbackcmd);
}

task_return_t remote_send_raw(const remote_communication_t* remote_communication)
{
	mavlink_message_t msg;
	mavlink_msg_rc_channels_raw_pack(	remote_communication->mavlink_stream->sysid,
										remote_communication->mavlink_stream->compid,
										&msg,
										time_keeper_get_millis(),
										0,
										remote_communication->remote->sat->channels[0] + 1024,
										remote_communication->remote->sat->channels[1] + 1024,
										remote_communication->remote->sat->channels[2] + 1024,
										remote_communication->remote->sat->channels[3] + 1024,
										remote_communication->remote->sat->channels[4] + 1024,
										remote_communication->remote->sat->channels[5] + 1024,
										remote_communication->remote->sat->channels[6] + 1024,
										remote_communication->remote->sat->channels[7] + 1024,
										// remote_communication->remote->mode.current_desired_mode.byte);
										remote_communication->remote->signal_quality	);
	
	mavlink_stream_send(remote_communication->mavlink_stream, &msg);
	
	return TASK_RUN_SUCCESS;
}

task_return_t remote_send_scaled(const remote_communication_t* remote_communication)
{
	mavlink_message_t msg;
	mavlink_msg_rc_channels_scaled_pack(	remote_communication->mavlink_stream->sysid,
											remote_communication->mavlink_stream->compid,
											&msg,
											time_keeper_get_millis(),
											0,
											remote_communication->remote->channels[0] * 10000.0f,
											remote_communication->remote->channels[1] * 10000.0f,
											remote_communication->remote->channels[2] * 10000.0f,
											remote_communication->remote->channels[3] * 10000.0f,
											remote_communication->remote->channels[4] * 10000.0f,
											remote_communication->remote->channels[5] * 10000.0f,
											remote_communication->remote->channels[6] * 10000.0f,
											remote_communication->remote->channels[7] * 10000.0f,
											remote_communication->remote->mode.current_desired_mode.byte );
											// remote_communication->remote->signal_quality	);
	
	mavlink_stream_send(remote_communication->mavlink_stream, &msg);
	
	return TASK_RUN_SUCCESS;
}