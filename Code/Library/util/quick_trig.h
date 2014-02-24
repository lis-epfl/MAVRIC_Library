/*
 * quick_trig.h
 *
 * Created: 11/11/2013 18:31:00
 *  Author: julien
 */ 

#ifndef QUICK_TRIG_H_
#define QUICK_TRIG_H_

#define INTERP_POINTS 50
#include "maths.h"

float quick_sin(float x);
float quick_cos(float x);
float quick_acos(float x);
float quick_asin(float x);
float quick_tan(float x);
float quick_atan(float x);

static inline float quick_func(float x, const float func_x_min, const float func_x_max, float func_x_step, const float func_y[])
{
	float y;
	int i;
	if ( x <= func_x_min )
	{
		y = func_y[0];
	}
	else if ( x >= func_x_max )
	{
		y = func_y[INTERP_POINTS - 1];
	}
	else
	{
		i = (int) ((x - func_x_min) / func_x_step);
		y = interpolate(x, func_x_min + i * func_x_step,
		func_x_min + (i + 1) * func_x_step,
		func_y[i], func_y[i+1]);
	}
	return y;
}


#endif /* QUICK_TRIG_H_ */