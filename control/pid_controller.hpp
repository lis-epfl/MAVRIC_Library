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
 * \file pid_control.hpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Basil Huber
 *
 * \brief PID controller
 *
 ******************************************************************************/


#ifndef PID_CONTROL_HPP_
#define PID_CONTROL_HPP_

#include <cstdint>
#include <cstdbool>
#include <math.h>


/**
 * \brief Integrator part of PID
 */
typedef struct
{
    float gain;         ///< Pregain
    float accumulator;  ///< Accumulator
    float clip_pre;     ///< Clipping value for the charging rate
    float clip;         ///< Clipping value for the accumulator
} integrator_t;


/**
 * \brief Derivative part of PID
 */
typedef struct
{
    float gain;         ///< Gain
    float previous;     ///< Previous input to the differentiator
    float clip;         ///< Clipping value
} differentiator_t;


/**
 * \brief Configuration for PID controller
 */
typedef struct
{
    float p_gain;
    float clip_min;                     ///< Min clipping values
    float clip_max;                     ///< Max clipping values
    integrator_t integrator;            ///< Integrator parameters
    differentiator_t differentiator;    ///< Differentiator parameters
    float soft_zone_width;              ///< Approximate width of a "soft zone" on the error input, i.e. a region of low gain around the target point. Value 0 -> switched off
} pid_controller_conf_t;


/**
 * \brief PID controller
 */
typedef struct
{
    float p_gain;                       ///< Proportional gain
    float clip_min;                     ///< Min clipping values
    float clip_max;                     ///< Max clipping values
    integrator_t integrator;            ///< Integrator parameters
    differentiator_t differentiator;    ///< Differentiator parameters
    float output;                       ///< Output
    float error;                        ///< Error
    float last_update_s;                ///< Last update time in seconds
    float dt_s;                         ///< Time step
    float soft_zone_width;              ///< Approximate width of a "soft zone" on the error input, i.e. a region of low gain around the target point. Value 0 -> switched off
} pid_controller_t;


/**
 * \brief   Init PID controller
 *
 * \param   controller      A PID controller structure
 * \param   config          Configuration
 *
 * \return  success
 */
bool pid_controller_init(pid_controller_t* controller, const pid_controller_conf_t* config);


/**
 * \brief   Init as passing through controller
 *
 * \param   controller      A PID controller structure
 */
void pid_controller_init_pass_through(pid_controller_t* controller);


/**
 * \brief   Reset integrator
 *
 * \param   controller      A PID controller structure
 */
void pid_controller_reset_integrator(pid_controller_t* controller);


/**
 * \brief               Update the PID controller
 *
 * \param   controller  Pointer to the PID controller structure
 * \param   error       Error in the controlled variable
 *
 * \return              The controller output
 */
float pid_controller_update(pid_controller_t* controller, float error);


/**
 * \brief               Update the PID controller for a given time step
 *
 * \param   controller  Pointer to the PID controller structure
 * \param   error       Error in the controlled variable
 * \param   dt          Timestep
 *
 * \return              The controller output
 */
float pid_controller_update_dt(pid_controller_t* controller, float error, float dt);


/**
 * \brief               Update the PID controller for a given time step
 *
 * \param   controller  Pointer to the PID controller structure
 * \param   error       Error in the controlled variable
 * \param   feedforward Feed-forward
 * \param   dt          Timestep
 *
 * \return              The controller output
 */
float pid_controller_update_feedforward_dt(pid_controller_t* controller, float error, float feedforward, float dt);


/**
 * \brief               Apply configuration to pid_controller without reseting the controller
 *
 * \details             config.integrator.accumulator and config.differentiator.previous are ignored
 *
 * \param   controller  Pointer to the PID controller structure
 * \param   config      Pointer to the configuration
 */
void pid_controller_apply_config(pid_controller_t* controller, const pid_controller_conf_t* config);

#endif /* PID_CONTROL_HPP_ */
