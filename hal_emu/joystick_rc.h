/*
 * spektrum.h
 *
 * ----- This is the emulated version using linux joystick instead of spektrum receiver, for offline mode -----
 * 
 *  Created on: Mar 2, 2010
 *      Author: felix
 */

#ifndef JOYSTICK_RC_H_
#define JOYSTICK_RC_H_
#include <stdint.h>
#include "buffer.h"

#define JOYSTICK_DEVICE "/dev/input/js0"
#define KEYBOARD_ACTIVE


//Gamepad
#define J_GAIN 700
#define JOY_THROTTLE 1
#define JOY_ROLL     3
#define JOY_PITCH    4
#define JOY_YAW      0
#define JOY_SAFETY   5
#define JOY_ID_MODE  6

#define JOY_SAFETY_OFF_BUTTON 7
#define JOY_SAFETY_ON_BUTTON 5
#define JOY_MODE_1_BUTTON 0
#define JOY_MODE_2_BUTTON 1
#define JOY_MODE_3_BUTTON 2
#define DEADZONE 5

//Saitek
/*
#define JOY_THROTTLE 2
#define JOY_ROLL     0
#define JOY_PITCH    1
#define JOY_YAW      5
/**/
/*
// logitech
#define JOY_THROTTLE 3
#define JOY_ROLL     0
#define JOY_PITCH    1
#define JOY_YAW      2
/**/


typedef struct Spektrum_Receiver {
	buffer_t receiver;
	uint32_t channels[16];
	uint32_t last_update;
	uint8_t valid;
	uint32_t last_time;
	uint32_t duration;
} spektrum_satellite_t;


void spektrum_satellite_init (void);
int16_t spektrum_satellite_get_channel(uint8_t index);
void spektrum_satellite_calibrate_center(uint8_t index);
int16_t spektrum_satellite_get_neutral(uint8_t index);
int8_t spektrum_satellite_check(void);

#endif 
