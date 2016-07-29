/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file pid_control.cpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Basil Huber
 *
 * \brief PID controller
 *
 ******************************************************************************/


#include "control/pid_controller.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/maths.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Integrating
 *
 * \param   integrator  Pointer to integrator parameters
 * \param   input       Imput of the PID controller
 * \param   dt          Timestep
 *
 * \return  Result
 */
static float pid_controller_integrate(integrator_t* integrator, float input, float dt);


/**
 * \brief Differentiating
 *
 * \param   diff        Pointer to differentiator parameters
 * \param   input       The new value to differentiate
 * \param   dt          Timestep
 *
 * \return              Result
 */
static float pid_controller_differentiate(differentiator_t* diff, float input,  float dt);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static float pid_controller_integrate(integrator_t* integrator, float input, float dt)
{
    integrator->accumulator = integrator->accumulator + maths_clip(dt * integrator->gain * input, integrator->clip_pre);
    integrator->accumulator = maths_clip(integrator->accumulator, integrator->clip);
    return integrator->accumulator;
}

static float pid_controller_differentiate(differentiator_t* diff, float input, float dt)
{
    float output = 0.0f;

    if (dt < 0.000001f)
    {
        output = 0.0f;
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

bool pid_controller_init(pid_controller_t* controller, const pid_controller_conf_t* config)
{
    float t = time_keeper_get_s();

    pid_controller_apply_config(controller, config);

    controller->output        = 0.0f;
    controller->error         = 0.0f;
    controller->last_update_s = t;
    controller->dt_s          = 1.0f;
    controller->integrator.accumulator = 0;
    controller->differentiator.previous = 0;
    return true;
}



void pid_controller_init_pass_through(pid_controller_t* controller)
{
    pid_controller_conf_t config    = {};
    config.p_gain                   = 1.0f;
    config.clip_min                 = -10000.0f;
    config.clip_max                 = 10000.0f;
    config.soft_zone_width          = 0.0f;
    config.integrator               = {};
    config.differentiator           = {};
    pid_controller_init(controller, &config);
}


void pid_controller_reset_integrator(pid_controller_t* controller)
{
    controller->integrator.accumulator = 0.0f;
}


float pid_controller_update(pid_controller_t* controller, float error)
{
    float t                   = time_keeper_get_s();
    controller->error         = maths_soft_zone(error, controller->soft_zone_width);
    controller->dt_s          = t - controller->last_update_s;
    controller->last_update_s = t;
    controller->output      = controller->p_gain * controller->error
                              + pid_controller_integrate(&controller->integrator, controller->error, controller->dt_s)
                              + pid_controller_differentiate(&controller->differentiator, controller->error, controller->dt_s);

    if (controller->output < controller->clip_min)
    {
        controller->output = controller->clip_min;
    }

    if (controller->output > controller->clip_max)
    {
        controller->output = controller->clip_max;
    }

    return controller->output;
}


float pid_controller_update_dt(pid_controller_t* controller, float error, float dt)
{
    controller->error         = error;
    controller->dt_s          = dt;
    controller->last_update_s = time_keeper_get_s();
    controller->output      = controller->p_gain * controller->error
                              + pid_controller_integrate(&controller->integrator, controller->error, controller->dt_s)
                              + pid_controller_differentiate(&controller->differentiator, controller->error, controller->dt_s);

    if (controller->output < controller->clip_min)
    {
        controller->output = controller->clip_min;
    }

    if (controller->output > controller->clip_max)
    {
        controller->output = controller->clip_max;
    }

    return controller->output;
}

float pid_controller_update_feedforward_dt(pid_controller_t* controller, float error, float feedforward, float dt)
{
    controller->error         = error;
    controller->dt_s          = dt;
    controller->last_update_s = time_keeper_get_s();
    controller->output        = controller->p_gain * controller->error
                                + pid_controller_integrate( &controller->integrator, controller->error, controller->dt_s)
                                + pid_controller_differentiate(&controller->differentiator, controller->error, controller->dt_s)
                                + feedforward;

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


void pid_controller_apply_config(pid_controller_t* controller, const pid_controller_conf_t* config)
{
    /* configure proportional part of PID */
    controller->p_gain          = config->p_gain;
    controller->clip_min        = config->clip_min;
    controller->clip_max        = config->clip_max;
    controller->soft_zone_width = config->soft_zone_width;
    
    /* configure integrator part of PID */
    controller->integrator.gain = config->integrator.gain;
    controller->integrator.clip_pre = config->integrator.clip_pre;
    controller->integrator.clip = config->integrator.clip;

    /* configure differentiator part of PID */
    controller->differentiator.gain = config->differentiator.gain;
    controller->differentiator.clip = config->differentiator.clip;
}
