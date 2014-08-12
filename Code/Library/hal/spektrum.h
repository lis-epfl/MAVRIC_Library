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
* \file spektrum.h
*
* This file is the driver for the remote control
*/


#ifndef SPEKTRUM_H_
#define SPEKTRUM_H_

#ifdef __cplusplus
	extern "C" {
#endif


#define RC_THROTTLE		0			///< Define the remote channel number for the throttle
#define RC_THROTTLE_DIR 1			///< Define the remote channel direction for the throttle

#define RC_ROLL			1			///< Define the remote channel number for the roll
#define RC_ROLL_DIR     -1			///< Define the remote channel direction for the roll

#define RC_PITCH		2			///< Define the remote channel number for the pitch
#define RC_PITCH_DIR    1			///< Define the remote channel direction for the pitch

#define RC_YAW			3			///< Define the remote channel number for the yaw
#define RC_YAW_DIR		1			///< Define the remote channel direction for the yaw

#define RC_SAFETY   4				///< Define the remote channel number for the safety switch
#define RC_ID_MODE  5				///< Define the remote channel number for the mode selection
#define RC_TRIM_P3  6				///< Define the remote channel number for the trim of (?)

#define DEADZONE 30.0f				///< Define the dead zone of a remote channel

#define RC_SCALEFACTOR 1.0f / 700.0f	///< Define the scale factor to apply on a channel value


#ifdef __cplusplus
	}
#endif

#endif //SPEKTRUM_H_