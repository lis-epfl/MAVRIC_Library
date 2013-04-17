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
 
typedef enum control_mode_t {ATTITUDE_COMMAND_MODE, RATE_COMMAND_MODE} control_mode_t;
typedef enum run_mode_t {MOTORS_OFF, MOTORS_ON} run_mode_t;

typedef struct {
	float rpy[3];
	float thrust;
	control_mode_t control_mode;
	run_mode_t run_mode;
} Control_Command_t;

typedef struct {
	PID_Controller_t rpy_controller[3];	
	Control_Command_t output;
} Stabiliser_t;

void init_stabilisation(void);

Stabiliser_t* get_rate_stabiliser();
Stabiliser_t* get_attitude_stabiliser();

void stabilise(Stabiliser_t *stabiliser, float *rpy_sensor_values, Control_Command_t *control_input);
void quad_stabilise(Imu_Data_t *imu, Control_Command_t *control_input);

void mix_to_servos_diag_quad(Control_Command_t *control);
void mix_to_servos_cross_quad(Control_Command_t *control);

#endif /* STABILISATION_H_ */