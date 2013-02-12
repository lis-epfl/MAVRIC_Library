/*
 * stabilisation.h
 *
 * Created: 07/06/2012 21:07:26
 *  Author: sfx
 */ 


#ifndef STABILISATION_H_
#define STABILISATION_H_

#include "compiler.h"
#include "imu.h"
#include "control.h"

#define ROLL 0
#define PITCH 1
#define YAW 2
 
enum {ATTITUDE_COMMAND_MODE, RATE_COMMAND_MODE};

typedef struct {
	float rpy[3];
	float thrust;
} Control_Command_t;

typedef struct {
	PID_Controller_t rpy_controller[3];	
	Control_Command_t output;
} Stabiliser_t;

void init_stabilisation(void);

Stabiliser_t* get_rate_stabiliser();
Stabiliser_t* get_attitude_stabiliser();

void stabilise(Stabiliser_t *stabiliser, float *rpy_sensor_values, Control_Command_t *control_input);
void quad_stabilise(Imu_Data_t *imu, Control_Command_t *control_input, int mode);

void mix_to_servos_diag_quad(Control_Command_t *control);
void mix_to_servos_cross_quad(Control_Command_t *control);

#endif /* STABILISATION_H_ */