/*
 * navigation.h
 *
 * Created: 15.05.2013 10:57:32
 *  Author: Philippe
 */ 

#include "control.h"

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#define PID_X	0	//For beta
#define PID_Y	1	//For phi
#define PID_Z	2	//For thrust

typedef struct {
	long latitude;
	long longitude;
	int on;
} Navigation_Data_t;

typedef struct { //in quadrotor reference frame
	float x_speed;
	float y_speed;
	float z_speed;
	float yaw;
	PID_Controller_t pid_data[3]; //x,y,z
} SpeedContr_Data_t;


void init_navigation(void);
void nav_function(int nav_type, int current_waypoint);
void run_navigation(void);
void speed_control_pid(void);
float acos_func(float x);
float Q_rsqrt(float number);
float get_angle_with_XNED(float v1_x,float v1_y,float v1_z);
float get_angle(float v1_x,float v1_y,float v1_z,float v2_x,float v2_y,float v2_z);
void circular_fly(void);

#endif /* NAVIGATION_H_ */