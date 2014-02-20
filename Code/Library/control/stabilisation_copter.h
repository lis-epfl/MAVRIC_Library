/*
 * stabilisation_copter.h
 *
 * 3-axis attitude and rate stabilisation for VTOLs
 * 
 * Created: 07/06/2012 21:08:01
 *  Author: Felix Schill
 */ 


#ifndef STABILISATION_COPTER_H_
#define STABILISATION_COPTER_H_

#include "stabilisation.h"


typedef struct {
	Stabiliser_t rate_stabiliser;
	Stabiliser_t attitude_stabiliser;
	Stabiliser_t velocity_stabiliser;
	float yaw_coordination_velocity;
} Stabiliser_Stack_copter_t;

void init_stabilisation_copter(Stabiliser_Stack_copter_t* stabiliser_stack);

void get_velocity_vector_from_remote(float tvel[]);

void cascade_stabilise_copter(Imu_Data_t *imu, position_estimator_t *pos_est, Control_Command_t *control_input);

void mix_to_servos_diag_quad(Control_Command_t *control);
void mix_to_servos_cross_quad(Control_Command_t *control);

#endif /* STABILISATION_COPTER_H_ */