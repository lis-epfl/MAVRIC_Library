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
 * \file maths.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Geraud L'Eplattenier
 *   
 * \brief Useful math functions
 *
 ******************************************************************************/


#ifndef MATHS_H
#define MATHS_H

#ifdef __cplusplus
extern "C" 
{
#endif

#include <stdint.h>
#include <math.h>

#define PI 3.141592653589793f


/**
 * \brief Conversion from degrees to radians
 */
#define MATH_DEG_TO_RAD (PI/180.0f)


/**
 * \brief Conversion from radians to degrees
 */
#define MATH_RAD_TO_DEG (180.0f/PI)


/**
 * \brief 			Square function
 * 
 * \param  	in 		Input variable
 * \return 			Squared variable
 */
#define SQR(in) \
		((in)*(in))


/**
 * \brief 		Converts a float from degrees to radians
 * 
 * \param 	i 	Value in degrees
 * \return 		Value in radians
 */
float static inline maths_deg_to_rad(float i)
{
	return MATH_DEG_TO_RAD * i;
}


/**
 * \brief 		Converts a float from radians to degrees
 * 
 * \param 	i 	Value in radians
 * \return 		Value in degrees
 */
float static inline maths_rad_to_deg(float i)
{
	return MATH_RAD_TO_DEG * i;
}


/**
 * \brief 			For any given angle, computes an equivalent angle between -2pi and 2pi 
 * 
 * \param 	angle 	Input angle
 * 
 * \return 			Output angle
 */
float static inline maths_calc_smaller_angle(float angle) 
{
	float out=angle;
	while (out<-PI) out += 2.0f * PI;
	while (out>=PI) out -= 2.0f * PI;
	return out;
}


/**
 * \brief 			Fast newton iteration for approximate square root
 * 
 * \param 	number 	Input value
 * 
 * \return 			Output value
 */
float static inline maths_fast_sqrt(float number) 
{
	union
	{
		float	f;
		int32_t	l;
	}i;
	
	float x, y;
	const float f = 1.5f;

	x = number * 0.5f;
	i.f = number;
	i.l  = 0x5f3759df - ( i.l >> 1 );
	y = i.f;
	y  = y * ( f - ( x * y * y ) );
	y  = y * ( f - ( x * y * y ) ); // repeat newton iteration for more accuracy
	return number * y;
}


/**
 * \brief 			Fast newton iteration for approximate square root of numbers close to 1 (for re-normalisation)
 * 
 * \param 	input 	Input value
 * \return 			Output value
 */
float static inline maths_fast_sqrt_1(float input) {
	if (input<0) 
	{
		return 0.0f;
	}

	float result = 1.0f;

	result = 0.5f * (result + (input / result));
	result = 0.5f * (result + (input / result));

	return result;
}


/**
 * \brief 		Returns the absolute value of a floating point value
 * 
 * \param 	a 	Input value
 * 
 * \return 		Absolute value
 */
static inline float maths_f_abs(const float a)
{
	if (a >= 0.0f)
	{
		return a;
	}
	else
	{
		return -a;
	}
}


/**
 * \brief 		Returns the minimum value of two floating point values
 * 
 * \param 	a 	Input value
 * \param 	b 	Input value
 * 
 * \return 		Minimum value
 */
static inline float maths_f_min(const float a, const float b)
{
	if (a <= b)
	{
		return a;
	}
	else
	{
		return b;
	}
}


/**
 * \brief 		Returns the maximum value of two floating point values
 * 
 * \param 	a 	Input value
 * \param 	b 	Input value
 * 
 * \return 		Maximum value
 */
static inline float maths_f_max(const float a, const float b){
	if (a >= b)
	{
		return a;
	}
	else
	{
		return b;
	}
}


/**
 * \brief 					Clip a variable
 * 
 * \details 				Acts like a saturation between -clip_value and clip_value
 * 
 * \param 	input_value 	Input value
 * \param 	clip_value 		Clip value
 * 
 * \return 					Clipped value
 */
static float inline maths_clip(float input_value, float clip_value) {
	
	if (input_value>clip_value)  return clip_value;     
	if (input_value<-clip_value) return -clip_value; 
	return input_value;
}


/**
 * \brief 	 				A function to attenuate values bellow a certain threshold
 * 
 * \param 	x 				Input value
 * \param 	soft_zone_width Width of the soft zone
 * 
 * \return 					Output value
 */
static float inline maths_soft_zone(float x, float soft_zone_width) 
{
	if (soft_zone_width < 0.0000001f) 
	{	
		return x;
	} 
	else 
	{
		return x * x * x / ( SQR(soft_zone_width) + SQR(x) );
	}
};


/**
 * \brief 		Sigmoid function
 * 
 * \param 	x 	Input value
 * \return 		Output value
 */
static float inline maths_sigmoid(float x) 
{
	return (x / maths_fast_sqrt(1 + SQR(x)));
};


/**
 * \brief 		Center window 2 (?)
 * 
 * \param 	x 	Input value
 * \return 		Output value
 */
static float inline maths_center_window_2(float x) 
{
	return 1.0f / (1 + SQR(x));
}


/**
 * \brief 		Center window 4 (?)
 * 
 * \param 	x 	Input value
 * \return 		Output value
 */
static float inline maths_center_window_4(float x) 
{
	return 1.0f / (1 + SQR(SQR(x)));
}


/**
 * \brief 		Median filter
 * 
 * \details  	Returns the median of 3 floating point values
 * 
 * \param 	a 	Input value
 * \param 	b 	Input value
 * \param 	c 	Input value
 * 
 * \return 		Output value
 */
static float inline maths_median_filter_3x(float a, float b, float c) {
	float middle;
	
	if ((a <= b) && (a <= c)) 
	{
		middle = (b <= c) ? b : c;
	}
	else if ((b <= a) && (b <= c))
	{
		middle = (a <= c) ? a : c;
	} 
	else 
	{
	   middle = (a <= b) ? a : b;
	}

	return middle;

}


/**
 * \brief 		Interpolation
 * 
 * \details 	With known y1 = f(x1) and y2 = f(x2), this function will maths_interpolate f to compute f(x)
 * 
 * \param 	x 	Target abscissa
 * \param 	x1  First known abscissa
 * \param 	x2 	Second known abscissa
 * \param 	y1 	First known ordinate
 * \param 	y2 	Second known ordinate
 * 
 * \return 		Interpolated value
 */
static inline float maths_interpolate(float x, float x1, float x2, float y1, float y2)
{
	if (x1 == x2)
	{
		return y1;
	}
	else
	{
		float y = y1 + (y2 - y1) * (x - x1) / (x2 - x1); 
		return y;
	}
}


#ifdef __cplusplus
}
#endif


#endif	/*	MATHS_H	 */
