/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
* \file remote_controller.h
*
* This file is the driver for the remote control
*/


#ifndef REMOTE_CONTROLLER_H_
#define REMOTE_CONTROLLER_H_

#include "conf_platform.h"

#ifdef SPEKTRUM_REMOTE				///< If you use the SPEKTRUM remote
	#include "spektrum.h"
	#include "remote_dsm2.h"
#endif

#ifdef TURNIGY_REMOTE				///< If you use the TURNIGY remote
	#include "turnigy.h"
	#include "remote_dsm2.h"
#endif

#ifdef JOYSTICK_REMOTE				///< If you use the JOYSTICK as a remote
	#include "joystick.h"
	#include "joystick_rc.h"
#endif

/**
 * \brief Return the roll angle from the remote
 *
 * \return the roll angle from the remote
 */
static float inline get_roll_from_remote(void)	
{
	return rc_get_channel_neutral(RC_ROLL)*RC_ROLL_DIR * RC_SCALEFACTOR; 
}

/**
 * \brief Return the pitch angle from the remote
 *
 * \return the pitch angle from the remote
 */
static float inline get_pitch_from_remote(void)	
{
	return rc_get_channel_neutral(RC_PITCH)*RC_PITCH_DIR * RC_SCALEFACTOR; 
}

/**
 * \brief Return the yaw angle from the remote
 *
 * \return the yaw angle from the remote
 */
static float inline get_yaw_from_remote(void)	
{
	return rc_get_channel_neutral(RC_YAW)*RC_YAW_DIR * RC_SCALEFACTOR; 
}

/**
 * \brief Return the thrust command from the remote
 *
 * \return the thrust command from the remote
 */
static float inline get_thrust_from_remote(void)	
{
	return rc_get_channel(RC_THROTTLE)*RC_THROTTLE_DIR*RC_SCALEFACTOR; 
}

/**
 * \brief return an object containing the stick position of the remote (roll, pitch, yaw and thrust)
 *
 * \return an object containing the stick position of the remote (roll, pitch, yaw and thrust)
 */
static inline Control_Command_t get_command_from_remote(void)
{
	Control_Command_t controls;
	controls.rpy[ROLL]= get_roll_from_remote()*RC_INPUT_SCALE;
	controls.rpy[PITCH]= get_pitch_from_remote()*RC_INPUT_SCALE;
	controls.rpy[YAW]= get_yaw_from_remote()*RC_INPUT_SCALE;
	controls.thrust = get_thrust_from_remote();
	
	return controls;
}

#ifdef SPEKTRUM_REMOTE
	/**
	 * \brief return a switch state of the remote
	 *
	 * \param chanSwitch pointer to a channel switch
	 */
	static inline void get_channel_mode(uint8_t* chanSwitch)
	{
		//TODO: remap with remote!
		*chanSwitch |= 0x00;
	
		if (rc_get_channel(RC_SAFETY)<0)
		{
			*chanSwitch |= 0x00;
		}
		else if(rc_get_channel(RC_SAFETY)>0 && rc_get_channel(RC_ID_MODE)<0)
		{
			*chanSwitch |= 0x01;
		}
		// else if (rc_get_channel(RC_SAFETY)>0 && rc_get_channel(RC_ID_MODE)>20)
		// {
		// 	*chanSwitch |= 0x03;
		// }
		else
		{
			*chanSwitch |= 0x02;
		}
	}
#endif

#ifdef TURNIGY_REMOTE
	/**
	 * \brief return a switch state of the remote
	 *
	 * \param chanSwitch pointer to a channel switch
	 */
	static inline void get_channel_mode(uint8_t* chanSwitch)
	{
		if (rc_get_channel(RC_SAFETY)<0)
		{
			*chanSwitch |= 0x00;
			}else if(rc_get_channel(RC_SAFETY)>0 && rc_get_channel(RC_ID_MODE)<0){
			*chanSwitch |= 0x01;
			}else if (rc_get_channel(RC_SAFETY)>0 && rc_get_channel(RC_ID_MODE)>20){
			*chanSwitch |= 0x03;
			}else{
			*chanSwitch |= 0x02;
		}
	}
#endif

#ifdef JOYSTICK_REMOTE
	/**
	 * \brief return a switch state of the joystick
	 *
	 * \param chanSwitch pointer to a channel switch
	 */
	static inline void get_channel_mode(uint8_t* chanSwitch)
	{
		if (rc_get_channel(RC_SAFETY)<0)
		{
			*chanSwitch |= 0x00;
			}else if(rc_get_channel(RC_SAFETY)>0 && rc_get_channel(RC_ID_MODE)<0){
			*chanSwitch |= 0x01;
			}else if (rc_get_channel(RC_SAFETY)>0 && rc_get_channel(RC_ID_MODE)>20){
			*chanSwitch |= 0x03;
			}else{
			*chanSwitch |= 0x02;
		}
	}
#endif

#endif //REMOTE_CONTROLLER_H_