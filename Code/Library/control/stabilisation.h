/*
 * stabilisation.h
 *
 * 3-axis attitude and rate stabilisation for VTOLs
 * 
 * Created: 07/06/2012 21:08:01
 *  Author: Felix Schill
 */ 


#ifndef STABILISATION_H_
#define STABILISATION_H_

#include "compiler.h"
#include "imu.h"
#include "control.h"

 
typedef enum control_mode_t {VELOCITY_COMMAND_MODE, ATTITUDE_COMMAND_MODE_REL_YAW, ATTITUDE_COMMAND_MODE_ABS_YAW, RATE_COMMAND_MODE} control_mode_t;
typedef enum run_mode_t {MOTORS_OFF, MOTORS_ON, SIMULATE} run_mode_t;

typedef struct {
	float rpy[3];  // roll pitch yaw rates/angles
	float thrust;

	float tvel[3]; // target velocity in m/s
	float theading;// absolute target heading
	control_mode_t control_mode;
	run_mode_t run_mode;
} Control_Command_t;

typedef struct {
	PID_Controller_t rpy_controller[3];	
	PID_Controller_t thrust_controller;
	Control_Command_t output;
} Stabiliser_t;



void init_stabilisation(void);

Stabiliser_t* get_rate_stabiliser(void);
Stabiliser_t* get_attitude_stabiliser(void);

void stabilise(Stabiliser_t *stabiliser, float *errors);
void quad_stabilise(Imu_Data_t *imu, Control_Command_t *control_input);

void mix_to_servos_diag_quad(Control_Command_t *control);
void mix_to_servos_cross_quad(Control_Command_t *control);

#endif /* STABILISATION_H_ */