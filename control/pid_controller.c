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


#include "pid_controller.h"
#include "time_keeper.h"
#include "maths.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Integrating
 *
 * \param	integrator	Pointer to integrator parameters
 * \param	input		Imput of the PID controller
 * \param	dt			Timestep
 *
 * \return	Result
 */
static float pid_controller_integrate(integrator_t* integrator, float input, float dt);


/**
 * \brief	Initialize integrator parameters
 *
 * \param	integrator	Pointer to an integrator structure
 * \param	gain		The gain of the integrator
 * \param	clip_pre	Clipping value for charging rate
 * \param	clip		Clipping value
 */
static void pid_controller_init_integrator(integrator_t* integrator, float gain, float clip_pre, float clip);


/**
 * \brief				Initialize Differentiator parameters
 *
 * \param	diff		Pointer to differentiator structure
 * \param	gain		The differential gain
 * \param	clip	Clipping value
 */
static void pid_controller_init_differenciator(differentiator_t* diff, float gain, float clip);


/**
 * \brief Differentiating
 *
 * \param	diff		Pointer to differentiator parameters
 * \param	input		The new value to differentiate
 * \param	dt			Timestep
 *
 * \return				Result
 */
static float pid_controller_differentiate(differentiator_t* diff, float input,  float dt);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static float pid_controller_integrate(integrator_t* integrator, float input, float dt)
{
	integrator->accumulator = integrator->accumulator + maths_clip(dt* integrator->gain * input, integrator->clip_pre);
	integrator->accumulator = maths_clip(integrator->accumulator, integrator->clip);
	return integrator->accumulator;
}


static void pid_controller_init_integrator(integrator_t* integrator, float gain, float clip_pre, float clip)
{
	integrator->gain 		= gain;
	integrator->clip_pre 	= clip_pre;
	integrator->clip 		= clip;
	integrator->accumulator = 0.0f;
}


static void pid_controller_init_differenciator(differentiator_t* diff, float gain, float clip_val)
{
	diff->gain 	= gain;
	diff->clip 	= clip_val;
}


static float pid_controller_differentiate(differentiator_t* diff, float input, float dt)
{
	float output = 0.0f;

	if( dt<0.000001f ) 
	{
		output=0.0f;
	} 
	else 
	{
		output = maths_clip(diff->gain * (input - diff->previous) / dt, diff->clip);
	}

	diff->previous = input;
	return output;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void pid_controller_init(pid_controller_t* controller, const pid_controller_conf_t* config)
{
	uint32_t t = time_keeper_get_time_ticks();

	controller->p_gain 			= config->p_gain;
	controller->clip_min 	 	= config->clip_min; 					
	controller->clip_max	 	= config->clip_max;		
	controller->integrator	 	= config->integrator;			
	controller->differentiator 	= config->differentiator;	
	controller->soft_zone_width = config->soft_zone_width;

	controller->output 		= 0.0f;
	controller->error 		= 0.0f;
	controller->last_update = t;
	controller->dt			= 0.0f;
}



void pid_controller_init_pass_through(pid_controller_t* controller)
{
	uint32_t t = time_keeper_get_time_ticks();

	controller->dt = 0.0f;
	controller->last_update = t;
	
	controller->p_gain 		= 1.0f;
	controller->clip_min 	= -10000.0f;
	controller->clip_max 	= 10000.0f;
	controller->error 		= 0.0f;
	controller->output 		= 0.0f;
	controller->soft_zone_width 		= 0.0f;
	controller->differentiator.previous = 0.0f;
	
	pid_controller_init_differenciator(	&(controller->differentiator), 
										0.0f, 0.0f);
	pid_controller_init_integrator(	&(controller->integrator), 
									0.0f, 0.0f, 0.0f);
}


void pid_controller_reset_integrator(pid_controller_t* controller)
{
	controller->integrator.accumulator = 0.0f;
}


float pid_controller_update(pid_controller_t* controller, float error)
{
	uint32_t t = time_keeper_get_time_ticks();
	controller->error 		= maths_soft_zone(error, controller->soft_zone_width);
	controller->dt 			= time_keeper_ticks_to_seconds(t - controller->last_update);
	controller->last_update = t;
	controller->output 		= controller->p_gain * controller->error  
								+ pid_controller_integrate( &controller->integrator, controller->error, controller->dt)
								+ pid_controller_differentiate(&controller->differentiator, controller->error, controller->dt);
	
	if( controller->output < controller->clip_min ) 
	{
		controller->output = controller->clip_min;
	}

	if( controller->output > controller->clip_max ) 
	{
		controller->output=controller->clip_max;
	}

	return controller->output;	
}


float pid_controller_update_dt(pid_controller_t* controller, float error, float dt) 
{
	controller->error 		= error;
	controller->dt 			= dt;
	controller->last_update = time_keeper_get_time_ticks();
	controller->output 		= controller->p_gain * controller->error  
								+ pid_controller_integrate( &controller->integrator, controller->error, controller->dt)
								+ pid_controller_differentiate(&controller->differentiator, controller->error, controller->dt);
	
	if( controller->output < controller->clip_min ) 
	{
		controller->output=controller->clip_min;
	}

	if( controller->output > controller->clip_max )
	{
		controller->output=controller->clip_max;
	}
	
	return controller->output;	
}
