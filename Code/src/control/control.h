/*
 * control.h
 *
 *  Created on: Mar 7, 2010
 *      Author: felix
 */

#ifndef CONTROL_H_
#define CONTROL_H_
#include "compiler.h"
#include <math.h>

typedef struct {
	float pregain, postgain;
	float accumulator;
	float clip;
	float leakiness;
} Integrator_t;

typedef struct {
	float gain;
	float previous;
	float LPF;
	float clip;
} Differentiator_t;

typedef struct {
	float p_gain;
	float clip_min, clip_max;
	Integrator_t integrator;
	Differentiator_t differentiator;
	float output;
	float error;
	uint32_t last_update; // last update time in timer tick
	float dt;  // is updated from system time at each update.
}PID_Controller_t;

float integrate(Integrator_t *integrator, float input, float dt);

void initInt(Integrator_t *integrator, float pregain, float postgain, float clip);
void resetInt(Integrator_t *integrator);

void initDiff(Differentiator_t *diff, float gain, float LPF, float clip);

float differentiate(Differentiator_t *diff, float input,  float dt);


float pid_update(PID_Controller_t* controller, float input, float goal_state);


#endif /* CONTROL_H_ */
