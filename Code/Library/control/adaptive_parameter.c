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
 * \file adaptive_parameter.c
 * 
 * This file automatically adapts the existing parameters based on given control variable
 */


#include "adaptive_parameter.h"
#include "maths.h"

Adaptive_Parameter_Set_t adapt_param_set;

Adaptive_Parameter_Set_t* get_param_set(void)
{
	return &adapt_param_set;
}


void init_adaptive_parameters(void)
{
	adapt_param_set.param_count = 0;
}


int add_adaptive_parameter(float* control_variable, float* parameter, 
							int nb_setpoints, float setpoints[], float setvalues[])
{
	if (adapt_param_set.param_count >= MAX_ADAPT_PARAM_COUNT)
	{
		return 0;
	}
	else
	{			
		Adaptive_Parameter_t new_param;
		new_param.control_variable = control_variable;
		new_param.parameter = parameter;
		new_param.nb_setpoints = nb_setpoints;
		int i;
		for (i = 0; i < nb_setpoints; ++i)
		{
			new_param.setpoints[i] = setpoints[i];
			new_param.setvalues[i] = setvalues[i];
		}

		adapt_param_set.parameters[adapt_param_set.param_count] = new_param;
		adapt_param_set.param_count += 1;
		return 1;
	}
}


void update_adaptive_parameter(Adaptive_Parameter_t param)
{
	if (*param.control_variable <= param.setpoints[0])
	{
		*param.parameter = param.setvalues[0];
	}
	else if (*param.control_variable >= param.setpoints[param.nb_setpoints - 1])
	{
		*param.parameter = param.setvalues[param.nb_setpoints - 1];
	}
	else
	{
		int i;
		for (i = 0; i < param.nb_setpoints; ++i)
		{
			if (*param.control_variable >= param.setpoints[i])
			{
				*param.parameter = interpolate(*param.control_variable, 
												param.setpoints[i], param.setpoints[i + 1],
												param.setvalues[i], param.setvalues[i + 1]);
				break;
			}
		}
	}
}


void update_all_adaptive_parameters(void)
{
	int i;
	for (i = 0; i < adapt_param_set.param_count; ++i)
	{
		update_adaptive_parameter(adapt_param_set.parameters[i]);
	}
}
