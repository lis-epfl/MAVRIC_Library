#ifndef JOYSTICK_H
#define JOYSTICK_H

struct joystick_event {
  unsigned int timestamp;
  short value;
  unsigned char type;
  unsigned char number;
};


int open_joystick(char* device_name);
int read_joystick_event(int joystick_file_descriptor, struct joystick_event *event);
void close_joystick(int joystick_file_descriptor);
int get_joystick_status(int joystick_file_descriptor, int *axes, int *buttons, int axes_size, int buttons_size);




#endif
