/*
 * spektrum.c
 *
 * ----- This is the emulated version using linux joystick instead of spektrum receiver, for offline mode -----
 * *
 *  Created on: August, 2013
 *      Author: felix
 */
#include "spektrum.h"
#include "time_keeper.h"
#include "print_util.h"
#include "joystick.h"

int joystick_filedescriptor;

Spektrum_Receiver_t spRec1;


int joystick_axes[16];
int joystick_buttons[16];

int16_t channelCenter[16];
int joyMax[16], joyMin[16];
uint32_t last_update;

void spektrum_init (void) {
	
	int i;
	for (i=0; i<16; i++) {
		spRec1.channels[i]=500;
		channelCenter[i]=0;
		joystick_axes[i]=0;
		joystick_buttons[i]=0;
		joyMax[i]=33000;
		joyMin[i]=-33000;
	}
	spRec1.channels[S_THROTTLE]=0;
	channelCenter[S_YAW]=8;
	joystick_filedescriptor=open_joystick(JOYSTICK_DEVICE);
	last_update=get_millis();
}
/**/


int16_t getChannel(uint8_t index) {
	int i;
	if (get_millis()-last_update>100) 
	{
		
		get_joystick_status(joystick_filedescriptor, &joystick_axes, &joystick_buttons, 16, 16);
		for (i=0; i<16; i++) {
			if (joystick_axes[i]>joyMax[i]) joyMax[i]=joystick_axes[i];
			if (joystick_axes[i]<joyMin[i]) joyMin[i]=joystick_axes[i];
			
		}
		
		spRec1.channels[S_ROLL] = -joystick_axes[JOY_ROLL]*700/ (joyMax[JOY_ROLL]-joyMin[JOY_ROLL]);
		spRec1.channels[S_PITCH] = -joystick_axes[JOY_PITCH]*700/ (joyMax[JOY_PITCH]-joyMin[JOY_PITCH]);
		spRec1.channels[S_YAW] = -joystick_axes[JOY_YAW]*700/ (joyMax[JOY_YAW]-joyMin[JOY_YAW]);
		spRec1.channels[S_THROTTLE] = -joystick_axes[JOY_THROTTLE]*700/ (joyMax[JOY_THROTTLE]-joyMin[JOY_THROTTLE]);
		spRec1.channels[4] = 400;
		spRec1.channels[5] = 400;
		last_update=get_millis();
	}	
	return spRec1.channels[index];
}

int16_t getChannelNeutral(uint8_t index) {
	int16_t value=getChannel(index)-channelCenter[index];
	// clamp to dead zone
	if ((value>-DEADZONE)&&(value<DEADZONE)) value=0;
	return value;
}

void centerChannel(uint8_t index){
	channelCenter[index]=getChannel(index);
}

int8_t checkReceiver1() {
	return (joystick_filedescriptor>0);

}

int8_t checkReceiver2(){
	return checkReceiver1();
}

int8_t checkReceivers() {
	return checkReceiver1();// + checkReceiver2();
}


