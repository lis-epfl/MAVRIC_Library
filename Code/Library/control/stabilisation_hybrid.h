/*
 * stabilisation_hybrid.h
 *
 * Created: 13/11/2013 15:46:00
 *  Author: Julien
 */ 

#ifndef STABILISATION_HYBRID_H_
#define STABILISATION_HYBRID_H_

#include "stabilisation.h"

typedef struct {
	Stabiliser_t rate_stabiliser;
	Stabiliser_t attitude_stabiliser;
} Stabiliser_Stack_hybrid_t;

void init_stabilisation_hybrid(Stabiliser_Stack_hybrid_t* stabiliser_stack);

void cascade_stabilise_hybrid(Imu_Data_t *imu, position_estimator_t *pos_est, Control_Command_t *control_input);
void mix_to_servos_xwing(Control_Command_t *control);

#endif /* STABILISATION_HYBRID_H_ */