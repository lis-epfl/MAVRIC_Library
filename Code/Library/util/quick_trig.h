/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file quick_trig.h
 * 
 * Quick implementation of trigonometric functions
 */ 


#ifndef QUICK_TRIG_H_
#define QUICK_TRIG_H_

#define INTERP_POINTS 50

#ifdef __cplusplus
extern "C" 
{
#endif

#include "maths.h"


/**
 * @brief             Quick implementation of the sinus function
 * 
 * @param x           Input value
 * 
 * @return            Estimated return value of the function
 */
float quick_sin(float x);


/**
 * @brief			  Quick implementation of the cosinus function
 * 
 * @param x           Input value
 * 
 * @return            Estimated return value of the function
 */
float quick_cos(float x);


/**
 * @brief             Quick implementation of the arccosinus function
 * 
 * @param x           Input value
 * 
 * @return            Estimated return value of the function
 */
float quick_acos(float x);


/**
 * @brief             Quick implementation of the arcsinus function
 * 
 * @param x           Input value
 * 
 * @return            Estimated return value of the function
 */
float quick_asin(float x);


/**
 * @brief             Quick implementation of the tangent function
 * 
 * @param x           Input value
 * 
 * @return            Estimated return value of the function
 */
float quick_tan(float x);


/**
 * @brief             Quick implementation of the arctangent function
 * 
 * @param x           Input value
 * 
 * @return            Estimated return value of the function
 */
float quick_atan(float x);


/**
 * @brief             Generic function used to approximate any function by interpolation
 * 
 * @param x           Input value
 * @param func_x_min  Minimum of input range
 * @param func_x_max  Maximum of input range
 * @param func_x_step Discretisation step used for interpolation  
 * @param func_y      Exact value of the fonction for each sampled input value
 * 
 * @return            Estimated return value of the function
 */
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
		y = interpolate(x, 
						func_x_min + i * func_x_step,
						func_x_min + (i + 1) * func_x_step,
						func_y[i], 
						func_y[i+1]);
	}
	return y;
}

#ifdef __cplusplus
}
#endif

#endif /* QUICK_TRIG_H_ */