/*
 * spektrum.h
 *
 *  Created on: Nov 01, 2013
 *      Author: ndousse
 */

#ifndef SPEKTRUM_H_
#define SPEKTRUM_H_

#include "compiler.h"
#include "buffer.h"
#include "stabilisation.h"

#define BAUD_REMOTE  115200

#define RC_THROTTLE		0
#define RC_THROTTLE_DIR 1

#define RC_ROLL			1
#define RC_ROLL_DIR     -1

#define RC_PITCH		2
#define RC_PITCH_DIR    1

#define RC_YAW			3
#define RC_YAW_DIR		1

#define RC_SAFETY   4
#define RC_ID_MODE  5
#define RC_TRIM_P3  6


#define DEADZONE 30.0

#define RC_SCALEFACTOR 1.0/700.0

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
#endif //SPEKTRUM_H_