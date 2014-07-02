/**
 * Executing the PID controllers for stabilization
 *
 * The MAV'RIC Framework
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the MAV'RIC Framework.
 */

#include "stabilisation.h"

void stabilise(Stabiliser_t *stabiliser, float dt, float errors[]) 
{
	int i;
	for (i = 0; i < 3; i++) 
	{
		stabiliser->output.rpy[i]=	pid_update_dt(&(stabiliser->rpy_controller[i]),  errors[i], dt);
	}		
	stabiliser->output.thrust= pid_update_dt(&(stabiliser->thrust_controller),  errors[3], dt);
}