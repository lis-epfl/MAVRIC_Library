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
 * \file pid_control.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief PID controller
 *
 ******************************************************************************/


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
static float pid_control_integrate(integrator_t *integrator, float input, float dt);

/**
 * \brief	Initialize integrator parameters
 *
 * \param	integrator	Pointer to an integrator structure
 * \param	pregain		The gain of the integrator
 * \param	postgain	The gain of the returned value
 * \param	clip_val	Clipping value
 */
static void pid_control_init_integrator(integrator_t *integrator, float pregain, float postgain, float clip_val);

/**
 * \brief				Initialize Differentiator parameters
 *
 * \param	diff		Pointer to differentiator structure
 * \param	gain		The differential gain
 * \param	LPF			Low pass filter
 * \param	clip_val	Clipping value
 */
static void pid_control_init_differenciator(differentiator_t *diff, float gain, float LPF, float clip_val);

/**
 * \brief Differentiating
 *
 * \param	diff		Pointer to differentiator parameters
 * \param	input		The new value to differentiate
 * \param	dt			Timestep
 *
 * \return				Result
 */
static float pid_control_differentiate(differentiator_t *diff, float input,  float dt);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static float pid_control_integrate(integrator_t *integrator, float input, float dt)
{
	integrator->accumulator=maths_clip(integrator->accumulator + dt* integrator->pregain * input, integrator->maths_clip);
	return integrator->postgain* integrator->accumulator;
}

static void pid_control_init_integrator(integrator_t *integrator, float pregain, float postgain, float clip_val)
{
	integrator->pregain=pregain;
	integrator->postgain=postgain;
	integrator->maths_clip=clip_val;
	integrator->accumulator=0.0f;
}

static void pid_control_init_differenciator(differentiator_t *diff, float gain, float LPF, float clip_val)
{
	diff->gain=gain;
	diff->LPF=LPF;
	diff->maths_clip=clip_val;
}

static float pid_control_differentiate(differentiator_t *diff, float input, float dt)
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

pid_controller_t pid_control_pass_through_controller()
{
	pid_controller_t out;
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

void pid_control_reset_integrator(integrator_t *integrator)
{
	integrator->accumulator=0.0f;
}

float pid_control_update(pid_controller_t* controller, float error)
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

float pid_control_update_dt(pid_controller_t* controller, float error, float dt) 
{
	controller->error=error;
	controller->dt=dt;
	controller->last_update=time_keeper_get_time_ticks();
	controller->output = controller->p_gain* (controller->error +pid_control_integrate(&controller->integrator, controller->error, controller->dt) + pid_control_differentiate(&controller->differentiator, controller->error, controller->dt));
	if (controller->output < controller->clip_min) controller->output=controller->clip_min;
	if (controller->output > controller->clip_max) controller->output=controller->clip_max;
	return controller->output;	
}

