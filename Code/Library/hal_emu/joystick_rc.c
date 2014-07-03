/*
 * spektrum.c
 *
 * ----- This is the emulated version using linux joystick instead of spektrum receiver, for offline mode -----
 * *
 *  Created on: August, 2013
 *      Author: felix
 */
#include "joystick_rc.h"
#include "time_keeper.h"
#include "print_util.h"
#include "joystick.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
//#include <stdlib.h>
#include <string.h>
#include <termios.h>

struct joystick_event {
  unsigned int timestamp;
  short value;
  unsigned char type;
  unsigned char number;
};

#define JS_BUTTON 0x01 // button event
#define JS_AXIS 0x02   // axis event
#define JS_INIT 0x80

int joystick_filedescriptor;

Spektrum_Receiver_t spRec1;


int joystick_axes[16];
int joystick_buttons[16];

int16_t channelCenter[16];
int joyMax[16], joyMin[16];
uint32_t last_update;

int open_joystick(char* device_name)
{
  int joystick_file_descriptor;
  joystick_file_descriptor = open(device_name, O_RDONLY | O_NONBLOCK);
  fcntl(joystick_file_descriptor, F_SETFL, O_NONBLOCK);
  return joystick_file_descriptor;
}

int read_joystick_event(int joystick_file_descriptor, struct joystick_event *event)
{
  int bytes;
  //struct timeval tv;
  //fd_set fds;
  //tv.tv_sec = 0;
  //tv.tv_usec = 0;
  //FD_ZERO(&fds);
  //FD_SET(joystick_file_descriptor, &fds);
  //if (select(joystick_file_descriptor+1, &fds, NULL, NULL, &tv)) 
  {
    
	bytes = read(joystick_file_descriptor, event, sizeof(*event));

	if (bytes == -1)    return 0;
	if (bytes == sizeof(*event))    return 1;
	
  }

  return -1;
}

void close_joystick(int joystick_file_descriptor)
{
	close(joystick_file_descriptor);
}

struct termios orig_termios;

void reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode()
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}

int kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

int getch()
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}


int get_joystick_status(int joystick_file_descriptor, int *axes, int *buttons, int axes_size, int buttons_size)
{
  int rc;
  struct joystick_event event;


  if (joystick_file_descriptor < 0)
    return -1;


  while ((rc = read_joystick_event(joystick_file_descriptor, &event) == 1)) {
    event.type &= ~JS_INIT;
    if ((event.type == JS_AXIS) && (event.number>=0) &&(event.number<axes_size)) {
      axes[event.number] = event.value;
	  
    } else if (event.type == JS_BUTTON) {
      if ((event.number < buttons_size)&& (event.number>=0)) {
		  
		switch (event.value) {
		  case 0:buttons[event.number] = event.value; break;
		  
		  case 1: 
		    print_util_dbg_print_num(event.number, 10);print_util_dbg_print(": ");print_util_dbg_print_num(event.value, 10);print_util_dbg_print("\n");
			buttons[event.number] = event.value;
			switch (event.number) {
			 case JOY_SAFETY_OFF_BUTTON: axes[RC_SAFETY]=-32000; break;
			 case JOY_SAFETY_ON_BUTTON: axes[RC_SAFETY]=  32000; break;
			 case JOY_MODE_1_BUTTON: axes[RC_ID_MODE]=-32000; break;
			 case JOY_MODE_2_BUTTON: axes[RC_ID_MODE]= 0; break;
			 case JOY_MODE_3_BUTTON: axes[RC_ID_MODE]= 32000; break;
			}
			break;
		  default:
			break;
		}
      }
    }
  }
  return 0;
}


void remote_dsm2_rc_init (void) {
	
	int i;
	for (i=0; i<16; i++) {
		spRec1.channels[i]=-500;
		channelCenter[i]=0;
		joystick_axes[i]=0;
		joystick_buttons[i]=0;
		joyMax[i]=32700;
		joyMin[i]=-32700;
	}
	spRec1.channels[RC_THROTTLE]=0;
	channelCenter[RC_YAW]=0;
	joystick_filedescriptor=open_joystick(JOYSTICK_DEVICE);
	last_update=time_keeper_get_millis();
	
	#ifdef KEYBOARD_ACTIVE
	set_conio_terminal_mode();
	#endif
}
/**/
uint32_t last_keypress;
void get_keyboard_input(int *joystick_axes) {
	if (!kbhit()) {
		if (time_keeper_get_millis()-last_keypress<1000) {
			
			if (time_keeper_get_millis()-last_keypress<500) {
				joystick_axes[JOY_PITCH]/=2;
				joystick_axes[JOY_ROLL]/=2;
				joystick_axes[JOY_YAW]/=2;
				joystick_axes[JOY_THROTTLE]/=2;
			} else {
				joystick_axes[JOY_PITCH]=0;
				joystick_axes[JOY_ROLL]=0;
				joystick_axes[JOY_YAW]=0;
				joystick_axes[JOY_THROTTLE]=0;
			}
		}
	} else
	while (kbhit()) {
		last_keypress=time_keeper_get_millis();
		char c=getch();
		if (c==3) exit(0);
		//wdbg_print(c);
		switch (c) {
			case 'i': joystick_axes[JOY_PITCH]=-20000; break;
			case 'k': joystick_axes[JOY_PITCH]= 20000; break;
			case 'j': joystick_axes[JOY_ROLL]=-20000; break;
			case 'l': joystick_axes[JOY_ROLL]= 20000; break;
			case 'a': joystick_axes[JOY_YAW]=-20000; break;
			case 'd': joystick_axes[JOY_YAW]= 20000; break;
			case 'w': joystick_axes[JOY_THROTTLE]=-10000; break;
			case 's': joystick_axes[JOY_THROTTLE]= 10000; break;
			
			case 'x': joystick_axes[JOY_SAFETY]=-20000; break;
			case 'c': joystick_axes[JOY_SAFETY]= 20000; break;

			case 'm': joystick_axes[JOY_THROTTLE]= 32000; 
			          joystick_axes[JOY_YAW]= 32000; 
			          break;

			case 'o': joystick_axes[JOY_THROTTLE]= 32000; 
			          joystick_axes[JOY_YAW]= -32000; 
			          break;

			case '1': joystick_axes[JOY_ID_MODE]=-20000; break;
			case '2': joystick_axes[JOY_ID_MODE]= 0; break;
			case '3': joystick_axes[JOY_ID_MODE]= 20000; break;
		}
	}
}


int16_t remote_dsm2_rc_get_channel(uint8_t index) {
	int i;
	if (time_keeper_get_millis()-last_update>20) 
	{

		#ifdef KEYBOARD_ACTIVE
		get_keyboard_input(&joystick_axes);
		#endif
		
		get_joystick_status(joystick_filedescriptor, &joystick_axes, &joystick_buttons, 16, 16);
		for (i=0; i<16; i++) {
			if (joystick_axes[i]>joyMax[i]) joyMax[i]=joystick_axes[i];
			if (joystick_axes[i]<joyMin[i]) joyMin[i]=joystick_axes[i];
			
		}
		
		spRec1.channels[RC_ROLL] = joystick_axes[JOY_ROLL]*J_GAIN/ (joyMax[JOY_ROLL]-joyMin[JOY_ROLL]);
		spRec1.channels[RC_PITCH] = joystick_axes[JOY_PITCH]*J_GAIN/ (joyMax[JOY_PITCH]-joyMin[JOY_PITCH]);
		spRec1.channels[RC_YAW] = joystick_axes[JOY_YAW]*J_GAIN/ (joyMax[JOY_YAW]-joyMin[JOY_YAW]);
		spRec1.channels[RC_THROTTLE] = -joystick_axes[JOY_THROTTLE]*J_GAIN/ (joyMax[JOY_THROTTLE]-joyMin[JOY_THROTTLE]);
		spRec1.channels[RC_SAFETY] = joystick_axes[JOY_SAFETY] *J_GAIN/ (joyMax[JOY_SAFETY]-joyMin[JOY_SAFETY]);
		spRec1.channels[RC_ID_MODE] =joystick_axes[JOY_ID_MODE]*J_GAIN/ (joyMax[JOY_ID_MODE]-joyMin[JOY_ID_MODE]);
		last_update=time_keeper_get_millis();
	}	
	return spRec1.channels[index];
}

int16_t remote_dsm2_rc_get_channel_neutral(uint8_t index) {
	int16_t value=remote_dsm2_rc_get_channel(index)-channelCenter[index];
	// clamp to dead zone
	if ((value>-DEADZONE)&&(value<DEADZONE)) value=0;
	return value;
}

void remote_dsm2_rc_center_channel(uint8_t index){
	channelCenter[index]=remote_dsm2_rc_get_channel(index);
}

int8_t checkReceiver1() {
	return (joystick_filedescriptor>0);

}


int8_t remote_dsm2_rc_check_receivers() {
	return checkReceiver1();// + checkReceiver2();
}


