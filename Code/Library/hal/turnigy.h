/*
 * turnigy.h
 *
 *  Created on: Aug 27, 2013
 *      Author: ndousse
 */

#ifndef TURNIGY_H_
#define TURNIGY_H_

#include "compiler.h"
#include "buffer.h"
#include "stabilisation.h"

#define BAUD_TURNIGY  115200
#define TURNIGY_UART AVR32_USART1

#define T_THROTTLE 0
#define T_ROLL     1
#define T_PITCH    2
#define T_YAW      3
#define T_GEAR	   4
#define T_ID_MODE  5
#define T_TRIM_P3  6
#define T_TRIM_P1  7
#define T_TRIM_P2  8

#define DEADZONE 0.5

#define T_SCALEFACTOR 1.0/438.0

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

#endif //TURNIGY_H_