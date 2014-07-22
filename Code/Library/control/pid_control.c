/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 
 
/**
 * \file pid_control.c
 *
 * PID controller
 */


#include "time_keeper.h"

#include "pid_control.h"
#include "maths.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Integrating
 *
 * \param	integrator	Pointer to integrator parameters
 * \param	input
 * \param	dt			Timestep
 *
 * \return	Result
 */
static float pid_control_integrate(Integrator_t *integrator, float input, float dt);

/**
 * \brief	Initialize integrator parameters
 *
 * \param	integrator	Pointer to an integrator structure
 * \param	pregain		The gain of the integrator
 * \param	postgain	The gain of the returned value
 * \param	clip_val	Clipping value
 */
static void pid_control_init_integrator(Integrator_t *integrator, float pregain, float postgain, float clip_val);

/**
 * \brief				Initialize Differentiator parameters
 *
 * \param	diff		Pointer to differentiator structure
 * \param	gain		The differential gain
 * \param	LPF			Low pass filter
 * \param	clip_val	Clipping value
 */
static void pid_control_init_differenciator(Differentiator_t *diff, float gain, float LPF, float clip_val);

/**
 * \brief Differentiating
 *
 * \param	diff		Pointer to differentiator parameters
 * \param	input		The new value to differentiate
 * \param	dt			Timestep
 *
 * \return				Result
 */
static float pid_control_differentiate(Differentiator_t *diff, float input,  float dt);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static float pid_control_integrate(Integrator_t *integrator, float input, float dt)
{
	integrator->accumulator=maths_clip(integrator->accumulator + dt* integrator->pregain * input, integrator->maths_clip);
	return integrator->postgain* integrator->accumulator;
}

static void pid_control_init_integrator(Integrator_t *integrator, float pregain, float postgain, float clip_val)
{
	integrator->pregain=pregain;
	integrator->postgain=postgain;
	integrator->maths_clip=clip_val;
	integrator->accumulator=0.0f;
}

static void pid_control_init_differenciator(Differentiator_t *diff, float gain, float LPF, float clip_val)
{
	diff->gain=gain;
	diff->LPF=LPF;
	diff->maths_clip=clip_val;
}

static float pid_control_differentiate(Differentiator_t *diff, float input, float dt)
{
	float output=0.0f;
	if (dt<0.000001f) {
		output=0.0f;
		} else {
		output=maths_clip(diff->gain * (input - diff->previous) / dt, diff->maths_clip);
	}
	//diff->previous=(1.0f - (diff->LPF)) * input + (diff->LPF) * (diff->previous);
	diff->previous=input;
	return output;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

PID_Controller_t pid_control_passthroughController()
{
	PID_Controller_t out;
	uint32_t t= time_keeper_get_time_ticks();

	out.dt = 0.0f;
	out.last_update = t;
	
	out.p_gain = 1.0f;
	out.clip_min = -10000.0f;
	out.clip_max = 10000.0f;
	out.error = 0.0f;
	out.output = 0.0f;
	out.soft_zone_width = 0.0f;
	out.integrator.leakiness = 0.0f;
	out.differentiator.previous = 0.0f;
	
	pid_control_init_differenciator(&(out.differentiator), 0.0f, 0.0f, 0.0f);
	pid_control_init_integrator(&(out.integrator), 0.0f, 0.0f, 0.0f);
	
	return out;
}

void pid_control_reset_integrator(Integrator_t *integrator)
{
	integrator->accumulator=0.0f;
}

float pid_control_update(PID_Controller_t* controller, float error)
{
	uint32_t t= time_keeper_get_time_ticks();
	controller->error=maths_soft_zone(error, controller->soft_zone_width);
	controller->dt=time_keeper_ticks_to_seconds(t - controller->last_update);
	controller->last_update=t;
	controller->output = controller->p_gain* (controller->error +pid_control_integrate(&controller->integrator, controller->error, controller->dt) + pid_control_differentiate(&controller->differentiator, controller->error, controller->dt));
	if (controller->output < controller->clip_min) controller->output=controller->clip_min;
	if (controller->output > controller->clip_max) controller->output=controller->clip_max;
	return controller->output;	
}

float pid_control_update_dt(PID_Controller_t* controller, float error, float dt) 
{
	controller->error=error;
	controller->dt=dt;
	controller->last_update=time_keeper_get_time_ticks();
	controller->output = controller->p_gain* (controller->error +pid_control_integrate(&controller->integrator, controller->error, controller->dt) + pid_control_differentiate(&controller->differentiator, controller->error, controller->dt));
	if (controller->output < controller->clip_min) controller->output=controller->clip_min;
	if (controller->output > controller->clip_max) controller->output=controller->clip_max;
	return controller->output;	
}

