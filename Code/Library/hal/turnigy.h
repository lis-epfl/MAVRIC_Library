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

#include "compiler.h"
#include "buffer.h"
#include "stabilisation.h"

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

#define RC_SCALEFACTOR 1.0f/880.0f				///< The scale factor of remote inputs

/*
typedef struct Turnigy_Receiver {
	Buffer_t receiver;
	uint16_t channels[16];
	uint32_t last_update;
	uint8_t valid;
	uint32_t last_time;
	uint32_t duration;
} Turnigy_Receiver_t;

void turnigy_init (void);
int16_t getChannel_turnigy(uint8_t index);
void centerChannel_turnigy(uint8_t index);
int16_t getChannelNeutral_turnigy(uint8_t index);
int8_t checkReceiver1_turnigy(void);
int8_t checkReceiver2_turnigy(void);
int8_t checkReceivers_turnigy(void);

Control_Command_t get_command_from_turnigy();
float get_roll_from_turnigy();
float get_pitch_from_turnigy();
float get_yaw_from_turnigy();
float get_thrust_from_turnigy();

void get_channel_mode_turnigy(uint8_t* chanSwitch);
*/

#ifdef __cplusplus
}
#endif

#endif //TURNIGY_H_