/*
 * spektrum.h
 *
 *  Created on: Mar 2, 2010
 *      Author: felix
 */

#ifndef SPEKTRUM_H_
#define SPEKTRUM_H_
#include "compiler.h"
#include "buffer.h"
#include "stabilisation.h"

#define BAUD_SPEKTRUM  115200
#define SPECTRUM_UART AVR32_USART1

#define S_THROTTLE 0
#define S_ROLL     1
#define S_PITCH    2
#define S_YAW      3

#define DEADZONE 7

typedef struct Spektrum_Receiver {
	Buffer_t receiver;
	uint16_t channels[16];
	uint32_t last_update;
	uint8_t valid;
	uint32_t last_time;
	uint32_t duration;
} Spektrum_Receiver_t;


void spektrum_init (void);
int16_t getChannel(uint8_t index);
void centerChannel(uint8_t index);
int16_t getChannelNeutral(uint8_t index);
int8_t checkReceiver1(void);
int8_t checkReceiver2(void);
int8_t checkReceivers(void);

Control_Command_t get_command_from_spektrum();
float get_roll_from_spektrum();
float get_pitch_from_spektrum();
float get_yaw_from_spektrum();
float get_thrust_from_spektrum();

#endif /* SPEKTRUM_H_ */
