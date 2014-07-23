/*
 * remote_controller.h
 *
 *  Created on: Aug 27, 2013
 *      Author: ndousse
 */


#ifndef REMOTE_CONTROLLER_H_
#define REMOTE_CONTROLLER_H_

#include "conf_platform.h"

#ifdef SPEKTRUM_REMOTE
	#include "spektrum.h"
	#include "remote_dsm2.h"
#endif

#ifdef TURNIGY_REMOTE
	#include "turnigy.h"
	#include "remote_dsm2.h"
#endif

#ifdef JOYSTICK_REMOTE
	#include "joystick.h"
	#include "joystick_rc.h"
#endif



static float inline remote_controller_get_roll_from_remote()	{return remote_dsm2_rc_get_channel(RC_ROLL)*RC_ROLL_DIR * RC_SCALEFACTOR; };
static float inline remote_controller_get_pitch_from_remote()	{return remote_dsm2_rc_get_channel(RC_PITCH)*RC_PITCH_DIR * RC_SCALEFACTOR; };
static float inline remote_controller_get_yaw_from_remote()	{return remote_dsm2_rc_get_channel(RC_YAW)*RC_YAW_DIR * RC_SCALEFACTOR; };
static float inline remote_controller_get_thrust_from_remote(){return remote_dsm2_rc_get_channel(RC_THROTTLE)*RC_THROTTLE_DIR*RC_SCALEFACTOR; };


static inline control_command_t remote_controller_get_command_from_remote()
{
	control_command_t controls;
	controls.rpy[ROLL]= remote_controller_get_roll_from_remote()*RC_INPUT_SCALE;
	controls.rpy[PITCH]= remote_controller_get_pitch_from_remote()*RC_INPUT_SCALE;
	controls.rpy[YAW]= remote_controller_get_yaw_from_remote()*RC_INPUT_SCALE;
	controls.thrust = remote_controller_get_thrust_from_remote();
	
	return controls;
}

static inline void remote_controller_get_channel_mode(uint8_t* chan_switch)
{
	if (remote_dsm2_rc_get_channel(RC_SAFETY)<0)
	{
		*chan_switch |= 0x00;
		}else if(remote_dsm2_rc_get_channel(RC_SAFETY)>0 && remote_dsm2_rc_get_channel(RC_ID_MODE)<0){
		*chan_switch |= 0x01;
		}else if (remote_dsm2_rc_get_channel(RC_SAFETY)>0 && remote_dsm2_rc_get_channel(RC_ID_MODE)>20){
		*chan_switch |= 0x03;
		}else{
		*chan_switch |= 0x02;
	}
}


#endif //REMOTE_CONTROLLER_H_