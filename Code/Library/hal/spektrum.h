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
	
#include "compiler.h"
#include "buffer.h"
#include "stabilisation.h"

#define BAUD_REMOTE  115200			///< Define the remote baudrate

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

#define RC_SCALEFACTOR 1.0f/700.0f	///< Define the scale factor to apply on a channel value

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

#endif //SPEKTRUM_H_