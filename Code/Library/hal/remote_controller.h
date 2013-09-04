/*
 * remote_controller.h
 *
 *  Created on: Aug 27, 2013
 *      Author: ndousse
 */


#ifndef REMOTE_CONTROLLER_H_
#define REMOTE_CONTROLLER_H_

#include "conf_platform.h"

#ifdef SPEKTUM_REMOTE
	#include "spektrum.h"
	#define remote_ctrl_init			spektrum_init
	#define getChannel_remote			getChannel_spektrum
	#define centerChannel_remote		centerChannel_spektrum
	#define getChannelNeutral_remote	getChannelNeutral_spektrum
	#define checkReceiver1_remote		checkReceiver1_spektrum
	#define checkReceiver2_remote		checkReceiver2_spektrum
	#define checkReceivers_remote		checkReceivers_spektrum

	#define get_command_from_remote		get_command_from_spektrum
	#define get_roll_from_remote		get_roll_from_spektrum
	#define get_pitch_from_remote		get_pitch_from_spektrum
	#define get_yaw_from_remote			get_yaw_from_spektrum
	#define get_thrust_from_remote		get_thrust_from_spektrum

	#define get_channel_mode			get_channel_mode_spektrum

	#define REM_THROTTLE				S_THROTTLE
	#define REM_ROLL					S_ROLL
	#define REM_PITCH					S_PITCH
	#define REM_YAW						S_YAW
	#define REM_4						4
	#define REM_5						5

	#define REM_SCALEFACTOR				S_SCALEFACTOR

#endif

#ifdef TURNIGY_REMOTE
	#include "turnigy.h"
	#define remote_ctrl_init			turnigy_init
	#define getChannel_remote			getChannel_turnigy
	#define centerChannel_remote		centerChannel_turnigy
	#define getChannelNeutral_remote	getChannelNeutral_turnigy
	#define checkReceiver1_remote		checkReceiver1_turnigy
	#define checkReceiver2_remote		checkReceiver2_turnigy
	#define checkReceivers_remote		checkReceivers_turnigy

	#define get_command_from_remote		get_command_from_turnigy
	#define get_roll_from_remote		get_roll_from_turnigy
	#define get_pitch_from_remote		get_pitch_from_turnigy
	#define get_yaw_from_remote			get_yaw_from_turnigy
	#define get_thrust_from_remote		get_thrust_from_turnigy

	#define get_channel_mode			get_channel_mode_turnigy

	#define REM_THROTTLE				T_THROTTLE
	#define REM_ROLL					T_ROLL
	#define REM_PITCH					T_PITCH
	#define REM_YAW						T_YAW
	#define REM_4						T_GEAR
	#define REM_5						T_ID_MODE
	#define REM_6						T_TRIM_P3
	#define REM_7						T_TRIM_P1
	#define REM_8						T_TRIM_P2

	#define REM_SCALEFACTOR				T_SCALEFACTOR

#endif

#endif //REMOTE_CONTROLLER_H_