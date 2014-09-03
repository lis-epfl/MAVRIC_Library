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
 * \file quick_trig.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Quick implementation of trigonometric functions
 *
 ******************************************************************************/


#include "quick_trig.h"

#include <math.h>


#define PI 3.141592653589793f

const float acos_x_min = 0.0f;
const float acos_x_max = 1.0f;
const float acos_x_step = 0.02040816326530612f;
const float acos_y[INTERP_POINTS] = {  1.57079633f,  1.55038675f,  1.52996866f,  1.50953352f,  1.48907274f,
								        1.4685776f ,  1.44803927f,  1.42744876f,  1.40679686f,  1.38607412f,
								        1.36527082f,  1.34437689f,  1.32338188f,  1.30227492f,  1.28104463f,
								        1.25967907f,  1.23816568f,  1.21649116f,  1.19464142f,  1.17260143f,
								        1.15035513f,  1.12788528f,  1.1051733f ,  1.08219905f,  1.05894067f,
								        1.03537426f,  1.01147361f,  0.98720979f,  0.96255075f,  0.93746072f,
								        0.9118996f ,  0.88582209f,  0.85917664f,  0.83190417f,  0.80393635f,
								        0.77519337f,  0.745581f  ,  0.71498658f,  0.6832735f ,  0.65027331f,
								        0.61577418f,  0.5795034f ,  0.54109953f,  0.50006579f,  0.45568637f,
								        0.40686148f,  0.3517375f ,  0.28669514f,  0.20237569f,  0.0f };     

const float sin_x_min = 0.0f;
const float sin_x_max = 1.57079633f;
const float sin_x_step = 0.0320570678937734f;
const float sin_y[INTERP_POINTS] = {	0.        ,  0.03205158f,  0.06407022f,  0.09602303f,  0.12787716f,
								        0.1595999f ,  0.19115863f,  0.22252093f,  0.25365458f,  0.28452759f,
								        0.31510822f,  0.34536505f,  0.375267f  ,  0.40478334f,  0.43388374f,
								        0.46253829f,  0.49071755f,  0.51839257f,  0.5455349f ,  0.57211666f,
								        0.59811053f,  0.6234898f ,  0.6482284f ,  0.67230089f,  0.69568255f,
								        0.71834935f,  0.740278f  ,  0.76144596f,  0.78183148f,  0.80141362f,
								        0.82017225f,  0.8380881f ,  0.85514276f,  0.8713187f ,  0.88659931f,
								        0.90096887f,  0.91441262f,  0.92691676f,  0.93846842f,  0.94905575f,
								        0.95866785f,  0.96729486f,  0.97492791f,  0.98155916f,  0.98718178f,
								        0.99179001f,  0.99537911f,  0.99794539f,  0.99948622f,  1.0f };       

const float tan_x_min = 0.0f;
const float tan_x_max = 1.57079633f;
const float tan_x_step = 0.0320570678937734f;
const float tan_y[INTERP_POINTS] = { 0.00000000e+00f,   3.20680536e-02f,   6.42021301e-02f,
							         9.64687973e-02f,   1.28935722e-01f,   1.61672245e-01f,
							         1.94749983e-01f,   2.28243474e-01f,   2.62230881e-01f,
							         2.96794751e-01f,   3.32022875e-01f,   3.68009244e-01f,
							         4.04855131e-01f,   4.42670336e-01f,   4.81574619e-01f,
							         5.21699359e-01f,   5.63189508e-01f,   6.06205877e-01f,
							         6.50927865e-01f,   6.97556711e-01f,   7.46319396e-01f,
							         7.97473389e-01f,   8.51312412e-01f,   9.08173541e-01f,
							         9.68445994e-01f,   1.03258210e+00f,   1.10111114e+00f,
							         1.17465690e+00f,   1.25396034e+00f,   1.33990890e+00f,
							         1.43357520e+00f,   1.53626854e+00f,   1.64960460e+00f,
							         1.77560126e+00f,   1.91681278e+00f,   2.07652140e+00f,
							         2.25901742e+00f,   2.47001933e+00f,   2.71732305e+00f,
							         3.01184067e+00f,   3.36933183e+00f,   3.81343340e+00f,
							         4.38128627e+00f,   5.13478865e+00f,   6.18535359e+00f,
							         7.75580253e+00f,   1.03660461e+01f,   1.55758072e+01f,
							         3.11836824e+01f,   0.0f };

const float atan_x_min = 0.0f;
const float atan_x_max = 10.0f;
const float atan_x_step = 0.20408163265306123f;  
const float atan_y[INTERP_POINTS] = {	0.0f        ,  0.20131711f,  0.38752381f,  0.54937448f,  0.68461716f,
								        0.79549883f,  0.88597508f,  0.96007036f,  1.02123631f,  1.07222842f,
								        1.11518067f,  1.15172883f,  1.18312675f,  1.21034074f,  1.23412151f,
								        1.25505772f,  1.2736155f ,  1.29016748f,  1.30501442f,  1.31840124f,
								        1.33052905f,  1.34156439f,  1.35164615f,  1.36089099f,  1.36939759f,
								        1.37724986f,  1.38451966f,  1.39126877f,  1.39755066f,  1.40341177f,
								        1.40889264f,  1.4140288f ,  1.41885155f,  1.42338852f,  1.4276642f ,
								        1.43170039f,  1.43551654f,  1.43913006f,  1.4425566f ,  1.44581022f,
								        1.44890362f,  1.45184831f,  1.4546547f ,  1.45733227f,  1.45988967f,
								        1.46233476f,  1.46467476f,  1.46691629f,  1.4690654f ,  1.47112767f};




float quick_trig_sin(float x)
{
	float y;
	
	float xx = fmod(x, 2.0f * PI);

	if (xx < 0)
	{
		y = - quick_trig_sin(-xx);
	}
	else if (xx > PI/2.0f)
	{
		y = quick_trig_sin(PI - xx);
	}
	else
	{
		y = quick_trig_func(xx, sin_x_min, sin_x_max, sin_x_step, sin_y);
	}

	return y;
}


float quick_trig_cos(float x)
{
	return quick_trig_sin(PI/2.0f + x);	
}


float quick_trig_acos(float x)
{
	float y;

	if (x < 0)
	{
		y = PI - quick_trig_acos(-x);
	}
	else
	{
		y = quick_trig_func(x, acos_x_min, acos_x_max, acos_x_step, acos_y);
	}

	return y;
}


float quick_trig_asin(float x)
{
	return PI/2.0f - quick_trig_acos(x);
}


float quick_trig_tan(float x)
{
	float y;
	float xx = fmod(x, PI/2);

	if (xx < 0)
	{
		y = - quick_trig_tan(-xx);
	}
	else
	{
		y = quick_trig_func(xx, sin_x_min, sin_x_max, sin_x_step, tan_y);
	}

	return y;
}


float quick_trig_atan(float x)
{
	float y;

	if (x < 0)
	{
		y = - quick_trig_atan(-x);
	}
	else if (x > 1000.0f)
	{
		y = PI/2.0f;
	}
	else if (x > 10.0f)
	{
		y = maths_interpolate(x, 10, 1000, atan_y[INTERP_POINTS - 1], PI/2);
	}
	else
	{
		y = quick_trig_func(x, atan_x_min, atan_x_max, atan_x_step, atan_y);
	}

	return y;
}