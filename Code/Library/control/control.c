/*
 * control.c
 *
 *  Created on: Mar 7, 2010
 *      Author: felix
 */

#include "time_keeper.h"

#include "control.h"


PID_Controller_t passthroughController() {
	PID_Controller_t out;
	out.p_gain=1.0;
	out.last_update=get_time_ticks();	
	out.clip_min=-1.0;
	out.clip_max= 1.0;
	out.output=0.0;
	initDiff(&(out.differentiator), 0.0, 0.0, 0.0);
	initInt(&(out.integrator), 0.0, 0.0, 0.0);
	return out;
}


float clip(float input_value, float clip_value);

float clip(float input_value, float clip_value) {
	
	if (input_value>clip_value)  return clip_value;     
	if (input_value<-clip_value) return -clip_value; 
	return input_value;
}


float integrate(Integrator_t *integrator, float input, float dt) {

	integrator->accumulator=clip(integrator->accumulator+dt* integrator->pregain * input, integrator->clip);
	return integrator->postgain* integrator->accumulator;

}




void initInt(Integrator_t *integrator, float pregain, float postgain, float clip_val) {
	integrator->pregain=pregain;
	integrator->postgain=postgain;
	integrator->clip=clip_val;
	integrator->accumulator=0.0;

}
void resetInt(Integrator_t *integrator) {
	integrator->accumulator=0.0;
}

void initDiff(Differentiator_t *diff, float gain, float LPF, float clip_val) {
	diff->gain=gain;
	diff->LPF=LPF;
	diff->clip=clip_val;
}

float differentiate(Differentiator_t *diff, float input, float dt) {
	float output=0;
	if (dt<0.000001) {
		output=0; 
	} else {
		output=clip(diff->gain*(input - diff->previous)/dt, diff->clip);
	}	
	diff->previous=(1.0-(diff->LPF))*input + (diff->LPF) * (diff->previous);
	return output;
}

float pid_update(PID_Controller_t* controller, float error) {
	uint32_t t= get_time_ticks();
	controller->error=error;
	controller->dt=ticks_to_seconds(t - controller->last_update);
	controller->last_update=t;
	
	controller->output = controller->p_gain* (controller->error +integrate(&controller->integrator, controller->error, controller->dt) + differentiate(&controller->differentiator, controller->error, controller->dt));
	if (controller->output < controller->clip_min) controller->output=controller->clip_min;
	if (controller->output > controller->clip_max) controller->output=controller->clip_max;
	return controller->output;	
}