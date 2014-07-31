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
 * \file turnigy.h
 *
 * This file configures the channels and the direction of the Turnigy remote
 */


#ifndef TURNIGY_H_
#define TURNIGY_H_

#ifdef __cplusplus
extern "C" {
#endif

#define BAUD_REMOTE  115200						///< The baud rate of the remote

#define RC_THROTTLE		0						///< The throttle channel
#define RC_THROTTLE_DIR 1						///< The direction of the throttle channel

#define RC_ROLL			1						///< The roll axis channel
#define RC_ROLL_DIR     1						///< The direction of the roll axis channel

#define RC_PITCH		2						///< The pitch axis channel
#define RC_PITCH_DIR    -1						///< The direction of the pitch axis channel

#define RC_YAW			3						///< The yaw axis channel
#define RC_YAW_DIR		1						///< The direction of the yaw axis channel

#define RC_SAFETY   4							///< The safety switch channel
#define RC_ID_MODE  5							///< The ID mode switch channel
#define RC_TRIM_P3  6							///< The P3 trim channel
#define RC_TRIM_P1  7							///< The P1 trim channel
#define RC_TRIM_P2  8							///< The P2 trim channel

#define DEADZONE 30.0f							///< The deadzone of the remote

#define RC_SCALEFACTOR 1.0f / 880.0f				///< The scale factor of remote inputs


#ifdef __cplusplus
}
#endif

#endif //TURNIGY_H_