/*
 * spektrum.c
 *
 * ----- This is the emulated version using linux joystick instead of spektrum receiver, for offline mode -----
 * *
 *  Created on: August, 2013
 *      Author: felix
 */
#include "spektrum_satellite.h"
#include "time_keeper.h"
#include "print_util.h"
// #include "joystick.h"

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
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
#define JS_BUTTON 0x01 // button event
#define JS_AXIS 0x02   // axis event
#define JS_INIT 0x80

#define RC_SCALEFACTOR 1/350.0
#define RC_THROTTLE		0
#define RC_THROTTLE_DIR 1

#define RC_ROLL			1
#define RC_ROLL_DIR     1

#define RC_PITCH		2
#define RC_PITCH_DIR    1

#define RC_YAW			3
#define RC_YAW_DIR		1

#define RC_SAFETY 5
#define RC_ID_MODE 6

#define RC_TRIM_P3 7


struct joystick_event 
{
  uint32_t timestamp;
  int16_t value;
  unsigned char type;
  unsigned char number;
};


int32_t joystick_filedescriptor;

satellite_t *spek_sat;									///< Declare a pointer to satellite struct containing the receiver structure for receiver 1

int32_t joystick_axes[16];
int32_t joystick_buttons[16];

int16_t channel_center[16];
int32_t joy_max[16], joy_min[16];
uint32_t last_update;


int32_t open_joystick(char* device_name)
{
  int32_t joystick_file_descriptor;
  joystick_file_descriptor = open(device_name, O_RDONLY | O_NONBLOCK);
  fcntl(joystick_file_descriptor, F_SETFL, O_NONBLOCK);
  return joystick_file_descriptor;
}


int32_t read_joystick_event(int32_t joystick_file_descriptor, struct joystick_event *event)
{
  int32_t bytes;
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


void close_joystick(int32_t joystick_file_descriptor)
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


int32_t kbhit()
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


int32_t getch()
{
    int32_t r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}


int32_t get_joystick_status(int32_t joystick_file_descriptor, int32_t *axes, int32_t *buttons, int32_t axes_size, int32_t buttons_size)
{
	int32_t rc;
	struct joystick_event event;


	if(joystick_file_descriptor < 0 )
	{
		return -1;
	}


	while( (rc = read_joystick_event(joystick_file_descriptor, &event) == 1) ) 
	{
		event.type &= ~JS_INIT;
		if ((event.type == JS_AXIS) && (event.number>=0) &&(event.number<axes_size)) 
		{
			axes[event.number] = event.value;
		} 
		else if (event.type == JS_BUTTON) 
		{
			if ((event.number < buttons_size)&& (event.number>=0)) 
			{  
				switch (event.value) 
				{
					case 0:
						buttons[event.number] = event.value; 
					break;

					case 1: 
						print_util_dbg_print_num(event.number, 10);
						print_util_dbg_print(": ");
						print_util_dbg_print_num(event.value, 10);
						print_util_dbg_print("\n");
						buttons[event.number] = event.value;
						switch (event.number) 
						{
							case JOY_SAFETY_OFF_BUTTON: 
								axes[RC_SAFETY]=-32000; 
							break;
							
							case JOY_SAFETY_ON_BUTTON: 
								axes[RC_SAFETY]=  32000; 
							break;
							
							case JOY_MODE_1_BUTTON: 
								axes[RC_ID_MODE]=-32000; 
							break;
							
							case JOY_MODE_2_BUTTON: 
								axes[RC_ID_MODE]= 0; 
							break;
							
							case JOY_MODE_3_BUTTON: 
								axes[RC_ID_MODE]= 32000; 
							break;
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

void spektrum_satellite_init(satellite_t* satellite, usart_config_t usart_conf_spektrum) 
{
	//init dependencies
	spek_sat = satellite;
	satellite->usart_conf_sat = usart_conf_spektrum;
	
	satellite_init = &spektrum_satellite_init;
	satellite_bind = &spektrum_satellite_bind;
	
	for( int32_t i = 0; i < 16; i++ ) 
	{
		satellite->channels[i] = 0;
		channel_center[i] = 0;
	}
	
	satellite->new_data_available = false;

	int32_t i;
	for( i=0; i<16; i++ ) 
	{
		channel_center[i] 	= 0;
		joystick_axes[i] 	= 0;
		joystick_buttons[i] = 0;
		joy_max[i] 			= 32700;
		joy_min[i] 			= -32700;
	}

	channel_center[RC_YAW] = 0;
	joystick_filedescriptor = open_joystick(JOYSTICK_DEVICE);
	last_update = time_keeper_get_millis();
	
	#ifdef KEYBOARD_ACTIVE
	set_conio_terminal_mode();
	#endif
	 
}


/**/
uint32_t last_keypress;

void get_keyboard_input(int32_t *joystick_axes) 
{
	if (!kbhit()) 
	{
		if( time_keeper_get_millis() - last_keypress < 1000 ) 
		{	
			if( time_keeper_get_millis() - last_keypress < 500 ) 
			{
				joystick_axes[JOY_PITCH] 	/= 2;
				joystick_axes[JOY_ROLL] 	/= 2;
				joystick_axes[JOY_YAW] 		/= 2;
				joystick_axes[JOY_THROTTLE] /= 2;
			} 
			else 
			{
				joystick_axes[JOY_PITCH] 	= 0;
				joystick_axes[JOY_ROLL] 	= 0;
				joystick_axes[JOY_YAW] 		= 0;
				joystick_axes[JOY_THROTTLE] = 0;
			}
		}
	} 
	else
	{
		while (kbhit()) 
		{
			last_keypress=time_keeper_get_millis();
			char c=getch();
			if (c==3) 
			{
				exit(0);
			}

			//wdbg_print(c);
			switch (c) 
			{
				case 'i':
					joystick_axes[JOY_PITCH] = -20000; 
				break;

				case 'k':
					joystick_axes[JOY_PITCH] = 20000; 
				break;

				case 'j':
					joystick_axes[JOY_ROLL] = -20000; 
				break;

				case 'l':
					joystick_axes[JOY_ROLL] = 20000; 
				break;

				case 'a':
					joystick_axes[JOY_YAW] = -20000; 
				break;

				case 'd':
					joystick_axes[JOY_YAW] = 20000; 
				break;

				case 'w':
					joystick_axes[JOY_THROTTLE] = -10000; 
				break;

				case 's':
					joystick_axes[JOY_THROTTLE] = 10000; 
				break;

				case 'x':
					joystick_axes[JOY_SAFETY] = -20000; 
				break;

				case 'c':
					joystick_axes[JOY_SAFETY] = 20000; 
				break;

				case 'm':
					joystick_axes[JOY_THROTTLE] = 32000; 
					joystick_axes[JOY_YAW] = 32000; 
				break;

				case 'o':
					joystick_axes[JOY_THROTTLE] = 32000; 
					joystick_axes[JOY_YAW] = -32000; 
				break;

				case '1':
					joystick_axes[JOY_ID_MODE] = -20000; 
				break;

				case '2':
					joystick_axes[JOY_ID_MODE] = 0; 
				break;

				case '3':
					joystick_axes[JOY_ID_MODE] = 20000; 
				break;		
			}
		}
	}
}


int16_t spektrum_satellite_get_channel(uint8_t index) {
	int32_t i;
	int16_t channels[16];

	if (time_keeper_get_millis() - last_update>20) 
	{

		#ifdef KEYBOARD_ACTIVE
		get_keyboard_input( joystick_axes );
		#endif
		
		get_joystick_status(joystick_filedescriptor, joystick_axes, joystick_buttons, 16, 16);
		
		for( i=0; i<16; i++ ) 
		{
			if( joystick_axes[i] > joy_max[i] ) 
			{
				joy_max[i] = joystick_axes[i];
			}
			if( joystick_axes[i] < joy_min[i] )
			{
				joy_min[i] = joystick_axes[i];
			}
		}
		
		channels[RC_ROLL] 		= joystick_axes[JOY_ROLL] 		* J_GAIN / ( joy_max[JOY_ROLL]    - joy_min[JOY_ROLL] 	  );
		channels[RC_PITCH] 		= joystick_axes[JOY_PITCH] 		* J_GAIN / ( joy_max[JOY_PITCH]   - joy_min[JOY_PITCH] 	  );
		channels[RC_YAW] 		= joystick_axes[JOY_YAW] 		* J_GAIN / ( joy_max[JOY_YAW]     - joy_min[JOY_YAW] 	  );
		channels[RC_THROTTLE] 	= -joystick_axes[JOY_THROTTLE] 	* J_GAIN / ( joy_max[JOY_THROTTLE]- joy_min[JOY_THROTTLE] );
		channels[RC_SAFETY] 	= joystick_axes[JOY_SAFETY] 	* J_GAIN / ( joy_max[JOY_SAFETY]  - joy_min[JOY_SAFETY]   );
		channels[RC_ID_MODE] 	= joystick_axes[JOY_ID_MODE] 	* J_GAIN / ( joy_max[JOY_ID_MODE] - joy_min[JOY_ID_MODE]  );
		
		last_update = time_keeper_get_millis();
	}	

	return channels[index];
}


int16_t spektrum_satellite_get_neutral(uint8_t index) 
{
	int16_t value=spektrum_satellite_get_channel(index)-channel_center[index];
	// clamp to dead zone
	if ((value>-DEADZONE)&&(value<DEADZONE)) value=0;
	return value;
}


void spektrum_satellite_calibrate_center(uint8_t index)
{
	channel_center[index]=spektrum_satellite_get_channel(index);
}


int8_t check_receiver1() 
{
	return (joystick_filedescriptor>0);
}


int8_t spektrum_satellite_check() 
{
	return check_receiver1();// + check_receiver2();
}