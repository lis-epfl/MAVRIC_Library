#include "joystick.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>


#define JS_BUTTON 0x01 // button event
#define JS_AXIS 0x02   // axis event
#define JS_INIT 0x80


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
	  case 0:
	  case 1: buttons[event.number] = event.value;
	    break;
	  default:
	    break;
	}
      }
    }
  }
  return 0;
}


