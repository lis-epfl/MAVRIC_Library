/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file quick_trig.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Quick implementation of trigonometric functions
 *
 ******************************************************************************/


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
float quick_trig_sin(float x);


/**
 * @brief			  Quick implementation of the cosinus function
 * 
 * @param x           Input value
 * 
 * @return            Estimated return value of the function
 */
float quick_trig_cos(float x);


/**
 * @brief             Quick implementation of the arccosinus function
 * 
 * @param x           Input value
 * 
 * @return            Estimated return value of the function
 */
float quick_trig_acos(float x);


/**
 * @brief             Quick implementation of the arcsinus function
 * 
 * @param x           Input value
 * 
 * @return            Estimated return value of the function
 */
float quick_trig_asin(float x);


/**
 * @brief             Quick implementation of the tangent function
 * 
 * @param x           Input value
 * 
 * @return            Estimated return value of the function
 */
float quick_trig_tan(float x);


/**
 * @brief             Quick implementation of the arctangent function
 * 
 * @param x           Input value
 * 
 * @return            Estimated return value of the function
 */
float quick_trig_atan(float x);


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
static inline float quick_trig_func(float x, const float func_x_min, const float func_x_max, float func_x_step, const float func_y[])
{
	float y;
	int32_t i;
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
		i = (int32_t) ((x - func_x_min) / func_x_step);
		y = maths_interpolate(x, 
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