/*
 * quick_trig.c
 *
 * Created: 11/11/2013 18:31:00
 *  Author: julien
 */ 

#include "quick_trig.h"

#include <math.h>
#include "maths.h"

#define INTERP_POINTS 50
#define PI 3.14159265

const float acos_x_min = 0.0;
const float acos_x_max = 1.0;
const float acos_x_step = 0.02040816326530612;
const float acos_y[INTERP_POINTS] = {  1.57079633,  1.55038675,  1.52996866,  1.50953352,  1.48907274,
								        1.4685776 ,  1.44803927,  1.42744876,  1.40679686,  1.38607412,
								        1.36527082,  1.34437689,  1.32338188,  1.30227492,  1.28104463,
								        1.25967907,  1.23816568,  1.21649116,  1.19464142,  1.17260143,
								        1.15035513,  1.12788528,  1.1051733 ,  1.08219905,  1.05894067,
								        1.03537426,  1.01147361,  0.98720979,  0.96255075,  0.93746072,
								        0.9118996 ,  0.88582209,  0.85917664,  0.83190417,  0.80393635,
								        0.77519337,  0.745581  ,  0.71498658,  0.6832735 ,  0.65027331,
								        0.61577418,  0.5795034 ,  0.54109953,  0.50006579,  0.45568637,
								        0.40686148,  0.3517375 ,  0.28669514,  0.20237569,  0.0 };     

const float sin_x_min = 0.0;
const float sin_x_max = 1.57079633;
const float sin_x_step = 0.0320570678937734;
const float sin_y[INTERP_POINTS] = {	0.        ,  0.03205158,  0.06407022,  0.09602303,  0.12787716,
								        0.1595999 ,  0.19115863,  0.22252093,  0.25365458,  0.28452759,
								        0.31510822,  0.34536505,  0.375267  ,  0.40478334,  0.43388374,
								        0.46253829,  0.49071755,  0.51839257,  0.5455349 ,  0.57211666,
								        0.59811053,  0.6234898 ,  0.6482284 ,  0.67230089,  0.69568255,
								        0.71834935,  0.740278  ,  0.76144596,  0.78183148,  0.80141362,
								        0.82017225,  0.8380881 ,  0.85514276,  0.8713187 ,  0.88659931,
								        0.90096887,  0.91441262,  0.92691676,  0.93846842,  0.94905575,
								        0.95866785,  0.96729486,  0.97492791,  0.98155916,  0.98718178,
								        0.99179001,  0.99537911,  0.99794539,  0.99948622,  1.0 };       

const float tan_x_min = 0.0;
const float tan_x_max = 1.57079633;
const float tan_x_step = 0.0320570678937734;
const float tan_y[INTERP_POINTS] = { 0.00000000e+00,   3.20680536e-02,   6.42021301e-02,
							         9.64687973e-02,   1.28935722e-01,   1.61672245e-01,
							         1.94749983e-01,   2.28243474e-01,   2.62230881e-01,
							         2.96794751e-01,   3.32022875e-01,   3.68009244e-01,
							         4.04855131e-01,   4.42670336e-01,   4.81574619e-01,
							         5.21699359e-01,   5.63189508e-01,   6.06205877e-01,
							         6.50927865e-01,   6.97556711e-01,   7.46319396e-01,
							         7.97473389e-01,   8.51312412e-01,   9.08173541e-01,
							         9.68445994e-01,   1.03258210e+00,   1.10111114e+00,
							         1.17465690e+00,   1.25396034e+00,   1.33990890e+00,
							         1.43357520e+00,   1.53626854e+00,   1.64960460e+00,
							         1.77560126e+00,   1.91681278e+00,   2.07652140e+00,
							         2.25901742e+00,   2.47001933e+00,   2.71732305e+00,
							         3.01184067e+00,   3.36933183e+00,   3.81343340e+00,
							         4.38128627e+00,   5.13478865e+00,   6.18535359e+00,
							         7.75580253e+00,   1.03660461e+01,   1.55758072e+01,
							         3.11836824e+01,   0.0 };

const float atan_x_min = 0.0;
const float atan_x_max = 10.0;
const float atan_x_step = 0.20408163265306123;  
const float atan_y[INTERP_POINTS] = {	0.        ,  0.20131711,  0.38752381,  0.54937448,  0.68461716,
								        0.79549883,  0.88597508,  0.96007036,  1.02123631,  1.07222842,
								        1.11518067,  1.15172883,  1.18312675,  1.21034074,  1.23412151,
								        1.25505772,  1.2736155 ,  1.29016748,  1.30501442,  1.31840124,
								        1.33052905,  1.34156439,  1.35164615,  1.36089099,  1.36939759,
								        1.37724986,  1.38451966,  1.39126877,  1.39755066,  1.40341177,
								        1.40889264,  1.4140288 ,  1.41885155,  1.42338852,  1.4276642 ,
								        1.43170039,  1.43551654,  1.43913006,  1.4425566 ,  1.44581022,
								        1.44890362,  1.45184831,  1.4546547 ,  1.45733227,  1.45988967,
								        1.46233476,  1.46467476,  1.46691629,  1.4690654 ,  1.47112767};

float quick_func(float x, const float func_x_min, const float func_x_max, float func_x_step, const float func_y[])
{
	float y, step;
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


float quick_sin(float x)
{
	float y;
	
	float xx = fmod(x, 2 * PI);

	if (xx < 0)
	{
		y = - quick_sin(-xx);
	}
	else if (xx > PI/2)
	{
		y = quick_sin(PI - xx);
	}
	else
	{
		y = quick_func(xx, sin_x_min, sin_x_max, sin_x_step, sin_y);
	}

	return y;
}


float quick_cos(float x)
{
	return quick_sin(PI/2 + x);	
}


float quick_acos(float x)
{
	float y;

	if (x < 0)
	{
		y = PI - quick_acos(-x);
	}
	else
	{
		y = quick_func(x, acos_x_min, acos_x_max, acos_x_step, acos_y);
	}

	return y;
}


float quick_asin(float x)
{
	return PI/2 - quick_acos(x);
}


float quick_tan(float x)
{
	float y;
	float xx = fmod(x, PI/2);

	if (xx < 0)
	{
		y = - quick_tan(-xx);
	}
	else
	{
		y = quick_func(xx, sin_x_min, sin_x_max, sin_x_step, tan_y);
	}

	return y;
}


float quick_atan(float x)
{
	float y;

	if (x < 0)
	{
		y = - quick_atan(-x);
	}
	else if (x > 1000)
	{
		y = PI/2;
	}
	else if (x > 10)
	{
		y = interpolate(x, 10, 1000, atan_y[INTERP_POINTS - 1], PI/2);
	}
	else
	{
		y = quick_func(x, atan_x_min, atan_x_max, atan_x_step, atan_y);
	}

	return y;
}

// #include <stdio.h>
// int main(int argc, char const *argv[])
// {
// 	printf("%f\n", quick_atan(1.2));
// 	return 0;
// }