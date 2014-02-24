/*
 * stabilisation.h
 *
 * Created: 13/11/2013 15:46:00
 *  Author: Felix Schill, Julien
 */ 

#ifndef STABILISATION_H_
#define STABILISATION_H_

#include "compiler.h"
#include "imu.h"
#include "pid_control.h"

 
typedef enum control_mode_t {VELOCITY_COMMAND_MODE, ATTITUDE_COMMAND_MODE, RATE_COMMAND_MODE} control_mode_t;
typedef enum yaw_mode_t     {YAW_RELATIVE, YAW_ABSOLUTE, YAW_COORDINATED} yaw_mode_t;

typedef enum run_mode_t {MOTORS_OFF, MOTORS_ON, SIMULATE} run_mode_t;

typedef struct {
	float rpy[3];  // roll pitch yaw rates/angles
	float thrust;

	float tvel[3]; // target velocity in m/s
	float theading;// absolute target heading
	
	control_mode_t control_mode;
	yaw_mode_t     yaw_mode;
	
} Control_Command_t;

typedef struct {
	PID_Controller_t rpy_controller[3];	
	PID_Controller_t thrust_controller;
	Control_Command_t output;
} Stabiliser_t;

void stabilise(Stabiliser_t *stabiliser, float dt, float errors[]);

#endif /* STABILISATION_H_ */