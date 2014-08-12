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
* \file remote.h
*
* This file is the driver for the remote control
*/

#ifndef REMOTE_H__
#define REMOTE_H__

#ifdef __cplusplus
	extern "C" {
#endif

#include "spektrum_satellite.h"
#include "scheduler.h"
#include "mavlink_stream.h" 

#include "mav_modes.h"

#define REMOTE_CHANNEL_COUNT 8


typedef enum
{
	SIGNAL_GOOD = 100,
	SIGNAL_BAD 	= 50,
	SIGNAL_LOST = 0,
} signal_quality_t;


typedef enum
{
	NORMAL 		= 1,
	INVERTED 	= -1,
} channel_inv_t;


typedef enum
{
	CHANNEL_THROTTLE = 0,
	CHANNEL_ROLL     = 1,
	CHANNEL_PITCH    = 2,
	CHANNEL_YAW      = 3,
	CHANNEL_GEAR     = 4,
	CHANNEL_FLAPS    = 5,
	CHANNEL_AUX1     = 6,
	CHANNEL_AUX2     = 7,
} remote_channel_t;


typedef enum
{
	REMOTE_TURNIGY	= 0,
	REMOTE_SPEKTRUM = 1,
} remote_type_t;


typedef struct
{
	remote_type_t type;
	// remote_mode_conf_t mode_config;
} remote_conf_t;


typedef struct
{
	spektrum_satellite_t* sat;
	float channels[REMOTE_CHANNEL_COUNT];
	channel_inv_t channel_inv[REMOTE_CHANNEL_COUNT];
	float trims[REMOTE_CHANNEL_COUNT];
	float scale;
	int16_t deadzone;
	signal_quality_t signal_quality;
	remote_type_t type;
	// remote_mode_t mode;
	const mavlink_stream_t* mavlink_stream;
} remote_t;


void remote_init(remote_t* remote, const remote_conf_t* config, const mavlink_stream_t* mavlink_stream);


task_return_t remote_update(remote_t* remote);


signal_quality_t remote_check(const remote_t* remote);


/**
 * \brief Update the remote channel central position array stored in .c file
 * Warning: you should ensure first that the remote has the stick in their neutral position first
 *
 * \param index Specify which channel we are interested in
 */
void remote_calibrate(remote_t* remote, remote_channel_t channel);


float remote_get_throttle(const remote_t* remote);


float remote_get_roll(const remote_t* remote);


float remote_get_pitch(const remote_t* remote);


float remote_get_yaw(const remote_t* remote);


task_return_t remote_send_raw(const remote_t* remote);


task_return_t remote_send_scaled(const remote_t* remote);


#ifdef __cplusplus
	}
#endif

#endif