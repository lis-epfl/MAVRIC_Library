/*
 * stabilisation.c
 *
 * Created: 13/11/2013 15:46:00
 *  Author: Felix Schill, Julien
 */ 

#include "stabilisation.h"
#include "stabilisation_copter.h"
#include "stabilisation_hybrid.h"

void stabilise(Stabiliser_t *stabiliser, float dt, float errors[]) 
{
	int i;
	for (i = 0; i < 3; i++) 
	{
		stabiliser->output.rpy[i]=	pid_update_dt(&(stabiliser->rpy_controller[i]),  errors[i], dt);
	}		
	stabiliser->output.thrust= pid_update_dt(&(stabiliser->thrust_controller),  errors[3], dt);
}