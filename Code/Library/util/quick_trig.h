/*
 * quick_trig.h
 *
 * Created: 11/11/2013 18:31:00
 *  Author: julien
 */ 

#ifndef QUICK_TRIG_H_
#define QUICK_TRIG_H_

float quick_func(float x, const float func_x_min, const float func_x_max, const float func_x_step, const float func_y[]);
float quick_sin(float x);
float quick_cos(float x);
float quick_acos(float x);
float quick_asin(float x);
float quick_tan(float x);
float quick_atan(float x);

#endif /* QUICK_TRIG_H_ */