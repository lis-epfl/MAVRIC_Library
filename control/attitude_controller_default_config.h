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
 * \file attitude_controller_default_config.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Default configuration for the module attitude_controller
 *
 ******************************************************************************/


#ifndef ATTITUDE_CONTROLLER_DEFAULT_CONFIG_H_
#define ATTITUDE_CONTROLLER_DEFAULT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "control/attitude_controller.h"

static inline attitude_controller_conf_t attitude_controller_default_config()
{
	attitude_controller_conf_t conf = {};
	// #########################################################################
	// ######  RATE CONTROL  ###################################################
	// #########################################################################
	// -----------------------------------------------------------------
	// ------ ROLL RATE PID --------------------------------------------
	// -----------------------------------------------------------------
	conf.rate_pid_config[ROLL] 							 = {};
	conf.rate_pid_config[ROLL].p_gain                    = 0.03f;
	conf.rate_pid_config[ROLL].clip_min                  = -0.9f;
	conf.rate_pid_config[ROLL].clip_max                  = 0.9f;
	conf.rate_pid_config[ROLL].integrator                = {};
	conf.rate_pid_config[ROLL].integrator.gain           = 0.015f;
	conf.rate_pid_config[ROLL].integrator.clip_pre       = 1.0f;
	conf.rate_pid_config[ROLL].integrator.accumulator    = 0.0f;
	conf.rate_pid_config[ROLL].integrator.clip           = 0.02f;
	conf.rate_pid_config[ROLL].differentiator            = {};
	conf.rate_pid_config[ROLL].differentiator.gain       = 0.0045f;
	conf.rate_pid_config[ROLL].differentiator.previous   = 0.0f;
	conf.rate_pid_config[ROLL].differentiator.clip       = 0.02f;
	conf.rate_pid_config[ROLL].soft_zone_width           = 0.0f;
	// -----------------------------------------------------------------
	// ------ PITCH RATE PID -------------------------------------------
	// -----------------------------------------------------------------
	conf.rate_pid_config[PITCH] 						 = {};
	conf.rate_pid_config[PITCH].p_gain                   = 0.03f;
	conf.rate_pid_config[PITCH].clip_min                 = -0.9f;
	conf.rate_pid_config[PITCH].clip_max                 = 0.9f;
	conf.rate_pid_config[PITCH].integrator               = {};
	conf.rate_pid_config[PITCH].integrator.gain          = 0.015f,
	conf.rate_pid_config[PITCH].integrator.clip_pre      = 1.0f;
	conf.rate_pid_config[PITCH].integrator.accumulator   = 0.0f;
	conf.rate_pid_config[PITCH].integrator.clip          = 0.02f;
	conf.rate_pid_config[PITCH].differentiator           = {};
	conf.rate_pid_config[PITCH].differentiator.gain      = 0.0045f;
	conf.rate_pid_config[PITCH].differentiator.previous  = 0.0f;
	conf.rate_pid_config[PITCH].differentiator.clip      = 0.02f;
	conf.rate_pid_config[PITCH].soft_zone_width          = 0.0f;
	// -----------------------------------------------------------------
	// ------ YAW RATE PID ---------------------------------------------
	// -----------------------------------------------------------------
	conf.rate_pid_config[YAW] 							 = {};
	conf.rate_pid_config[YAW].p_gain                     = 0.3f;
	conf.rate_pid_config[YAW].clip_min                   = -0.3f;
	conf.rate_pid_config[YAW].clip_max                   = 0.3f;
	conf.rate_pid_config[YAW].integrator                 = {};
	conf.rate_pid_config[YAW].integrator.gain            = 0.15f;
	conf.rate_pid_config[YAW].integrator.clip_pre        = 1.0f;
	conf.rate_pid_config[YAW].integrator.accumulator     = 0.0f;
	conf.rate_pid_config[YAW].integrator.clip            = 0.09f;
	conf.rate_pid_config[YAW].differentiator             = {};
	conf.rate_pid_config[YAW].differentiator.gain        = 0.0f;
	conf.rate_pid_config[YAW].differentiator.previous    = 0.0f;
	conf.rate_pid_config[YAW].differentiator.clip        = 0.0f;
	conf.rate_pid_config[YAW].soft_zone_width            = 0.0;

	// #########################################################################
	// ######  ANGLE CONTROL  ##################################################
	// #########################################################################
	// -----------------------------------------------------------------
	// ------ ROLL ANGLE PID -------------------------------------------
	// -----------------------------------------------------------------
	conf.angle_pid_config[ROLL] 						 = {};
	conf.angle_pid_config[ROLL].p_gain                   = 4.0f;
	conf.angle_pid_config[ROLL].clip_min                 = -1.2f;
	conf.angle_pid_config[ROLL].clip_max                 = 1.2f;
	conf.angle_pid_config[ROLL].integrator               = {};
	conf.angle_pid_config[ROLL].integrator.gain          = 0.0f;
	conf.angle_pid_config[ROLL].integrator.clip_pre      = 0.0f;
	conf.angle_pid_config[ROLL].integrator.accumulator   = 0.0f;
	conf.angle_pid_config[ROLL].integrator.clip          = 0.0f;
	conf.angle_pid_config[ROLL].differentiator           = {};
	conf.angle_pid_config[ROLL].differentiator.gain      = 0.0f;
	conf.angle_pid_config[ROLL].differentiator.previous  = 0.0f;
	conf.angle_pid_config[ROLL].differentiator.clip      = 0.0f;
	conf.angle_pid_config[ROLL].soft_zone_width          = 0.0f;
	// -----------------------------------------------------------------
	// ------ PITCH ANGLE PID ------------------------------------------
	// -----------------------------------------------------------------
	conf.angle_pid_config[PITCH] 						 = {};
	conf.angle_pid_config[PITCH].p_gain                  = 4.0f;
	conf.angle_pid_config[PITCH].clip_min                = -1.2f;
	conf.angle_pid_config[PITCH].clip_max                = 1.2f;
	conf.angle_pid_config[PITCH].integrator              = {};
	conf.angle_pid_config[PITCH].integrator.gain         = 0.0f;
	conf.angle_pid_config[PITCH].integrator.clip_pre     = 0.0f;
	conf.angle_pid_config[PITCH].integrator.accumulator  = 0.0f;
	conf.angle_pid_config[PITCH].integrator.clip         = 0.0f;
	conf.angle_pid_config[PITCH].differentiator          = {};
	conf.angle_pid_config[PITCH].differentiator.gain     = 0.0f;
	conf.angle_pid_config[PITCH].differentiator.previous = 0.0f;
	conf.angle_pid_config[PITCH].differentiator.clip     = 0.0f;
	conf.angle_pid_config[PITCH].soft_zone_width         = 0.0f;
	// -----------------------------------------------------------------
	// ------ YAW ANGLE PID --------------------------------------------
	// -----------------------------------------------------------------
	conf.angle_pid_config[YAW] 							 = {};
	conf.angle_pid_config[YAW].p_gain                    = 3.0f;
	conf.angle_pid_config[YAW].clip_min                  = -1.5f;
	conf.angle_pid_config[YAW].clip_max                  = 1.5f;
	conf.angle_pid_config[YAW].integrator                = {};
	conf.angle_pid_config[YAW].integrator.gain           = 0.0f;
	conf.angle_pid_config[YAW].integrator.clip_pre       = 0.0f;
	conf.angle_pid_config[YAW].integrator.accumulator    = 0.0f;
	conf.angle_pid_config[YAW].integrator.clip           = 0.0f;
	conf.angle_pid_config[YAW].differentiator            = {};
	conf.angle_pid_config[YAW].differentiator.gain       = 0.0f;
	conf.angle_pid_config[YAW].differentiator.previous   = 0.0f;
	conf.angle_pid_config[YAW].differentiator.clip       = 0.0f;
	conf.angle_pid_config[YAW].soft_zone_width           = 0.0f;

	return conf;
};

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_CONTROLLER_DEFAULT_CONFIG_H_