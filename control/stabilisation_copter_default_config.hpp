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
 * \file stabilisation_copter_default_config.h
 *
 * \author MAV'RIC Team
 *
 * \brief Default values for cascade PID controller
 *
 ******************************************************************************/


#ifndef STABILISATION_COPTER_DEFAULT_CONFIG_H_
#define STABILISATION_COPTER_DEFAULT_CONFIG_H_

#include "control/stabilisation_copter.hpp"


static inline stabilisation_copter_conf_t stabilisation_copter_default_config()
{
    stabilisation_copter_conf_t conf    = {};
    conf.thrust_hover_point         = -0.26f;
    conf.motor_layout           = QUADCOPTER_MOTOR_LAYOUT_DIAG;

    conf.stabiliser_stack           = {};

    // #############################################################################
    // ######  RATE CONTROL  #######################################################
    // #############################################################################
    // -----------------------------------------------------------------
    // ------ ROLL PID -------------------------------------------------
    // -----------------------------------------------------------------
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].p_gain           = 0.07f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].clip_min             = -0.9f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].clip_max             = 0.9f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].integrator           = {};
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].integrator.gain      = 0.125f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].integrator.clip_pre      = 6.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].integrator.accumulator   = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].integrator.clip      = 0.3f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].differentiator       = {};
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].differentiator.gain      = 0.008f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].differentiator.previous  = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].differentiator.clip      = 0.14f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].output           = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].error            = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].last_update_s          = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].dt_s               = 1;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].soft_zone_width      = 0.0f;
    // -----------------------------------------------------------------
    // ------ PITCH PID ------------------------------------------------
    // -----------------------------------------------------------------
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].p_gain          = 0.07f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].clip_min            = -0.9f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].clip_max            = 0.9f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].integrator          = {};
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].integrator.gain         = 0.125f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].integrator.clip_pre     = 6.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].integrator.accumulator  = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].integrator.clip         = 0.3f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].differentiator      = {};
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].differentiator.gain     = 0.008f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].differentiator.previous     = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].differentiator.clip     = 0.14f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].output          = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].error           = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].last_update_s         = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].dt_s              = 1;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].soft_zone_width         = 0.0f;
    // -----------------------------------------------------------------
    // ------ YAW PID --------------------------------------------------
    // -----------------------------------------------------------------
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].p_gain            = 0.3f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].clip_min          = -0.3f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].clip_max          = 0.3f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].integrator            = {};
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].integrator.gain       = 0.075f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].integrator.clip_pre       = 1.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].integrator.accumulator    = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].integrator.clip       = 0.045f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].differentiator        = {};
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].differentiator.gain       = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].differentiator.previous   = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].differentiator.clip       = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].output            = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].error             = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].last_update_s           = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].dt_s                = 1;
    conf.stabiliser_stack.rate_stabiliser.rpy_controller[YAW].soft_zone_width       = 0.0f;
    // ---------------------------------------------------------------------
    // ------ THRUST PID ---------------------------------------------------
    // ---------------------------------------------------------------------
    conf.stabiliser_stack.rate_stabiliser.thrust_controller                 = {};
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.p_gain                      = 1.0f;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.clip_min                    = -1000;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.clip_max                    = 1000;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.integrator                  = {};
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.integrator.gain             = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.integrator.clip_pre         = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.integrator.accumulator      = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.integrator.clip             = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.differentiator              = {};
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.differentiator.gain         = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.differentiator.previous     = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.differentiator.clip         = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.output                      = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.error                       = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.last_update_s                 = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.thrust_controller.dt_s                          = 1,
                                                            conf.stabiliser_stack.rate_stabiliser.thrust_controller.soft_zone_width              = 0.0f;
    // ---------------------------------------------------------------------
    // ------ OUTPUT -------------------------------------------------------
    // ---------------------------------------------------------------------
    conf.stabiliser_stack.rate_stabiliser.output                        = {};
    conf.stabiliser_stack.rate_stabiliser.output.rpy[0]                 = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.output.rpy[1]                 = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.output.rpy[2]                 = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.output.thrust                     = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.output.tvel[0]                    = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.output.tvel[1]                    = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.output.tvel[2]                    = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.output.theading                   = 0.0f;
    conf.stabiliser_stack.rate_stabiliser.output.control_mode               =  RATE_COMMAND_MODE;
    conf.stabiliser_stack.rate_stabiliser.output.yaw_mode                   = YAW_RELATIVE;


    // #############################################################################
    // ######  ATTITUDE CONTROL  ###################################################
    // #############################################################################
    // -----------------------------------------------------------------
    // ------ ROLL PID -------------------------------------------------
    // -----------------------------------------------------------------
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].p_gain           = 4.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].clip_min         = -12.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].clip_max         = 12.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].integrator       = {};
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].integrator.gain      = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].integrator.clip_pre  = 0.00f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].integrator.accumulator   = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].integrator.clip      = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].differentiator       = {};
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].differentiator.gain  = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].differentiator.previous  = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].differentiator.clip  = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].output           = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].error            = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].last_update_s      = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].dt_s           = 1;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].soft_zone_width      = 0.0f;
    // -----------------------------------------------------------------
    // ------ PITCH PID ------------------------------------------------
    // -----------------------------------------------------------------
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].p_gain          = 4.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].clip_min        = -12.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].clip_max        = 12.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].integrator      = {};
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].integrator.gain     = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].integrator.clip_pre     = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].integrator.accumulator  = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].integrator.clip     = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].differentiator      = {};
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].differentiator.gain     = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].differentiator.previous = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].differentiator.clip     = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].output          = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].error           = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].last_update_s         = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].dt_s          = 1;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].soft_zone_width     = 0.0f;
    // -----------------------------------------------------------------
    // ------ YAW PID --------------------------------------------------
    // -----------------------------------------------------------------
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].p_gain            = 3.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].clip_min          = -1.5f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].clip_max          = 1.5f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].integrator        = {};
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].integrator.gain       = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].integrator.clip_pre   = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].integrator.accumulator    = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].integrator.clip       = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].differentiator        = {};
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].differentiator.gain   = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].differentiator.previous   = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].differentiator.clip   = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].output            = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].error             = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].last_update_s       = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].dt_s            = 1;
    conf.stabiliser_stack.attitude_stabiliser.rpy_controller[YAW].soft_zone_width       = 0.0f;
    // ---------------------------------------------------------------------
    // ------ THRUST PID ---------------------------------------------------
    // ---------------------------------------------------------------------
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller                             = {};
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.p_gain                  = 1.0f;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.clip_min                = -1;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.clip_max                = 1;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.integrator              = {};
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.integrator.gain         = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.integrator.clip_pre     = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.integrator.accumulator  = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.integrator.clip         = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.differentiator          = {};
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.differentiator.gain         = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.differentiator.previous     = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.differentiator.clip         = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.output                  = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.error                   = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.last_update_s             = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.dt_s                      = 1;
    conf.stabiliser_stack.attitude_stabiliser.thrust_controller.soft_zone_width         = 0.0f;
    // ---------------------------------------------------------------------
    // ------ OUTPUT -------------------------------------------------------
    // ---------------------------------------------------------------------
    conf.stabiliser_stack.attitude_stabiliser.output                                        = {};
    conf.stabiliser_stack.attitude_stabiliser.output.rpy[0]                 = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.output.rpy[1]                 = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.output.rpy[2]                 = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.output.thrust                 = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.output.tvel[0]                = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.output.tvel[1]                = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.output.tvel[2]                = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.output.theading               = 0.0f;
    conf.stabiliser_stack.attitude_stabiliser.output.control_mode               = RATE_COMMAND_MODE;
    conf.stabiliser_stack.attitude_stabiliser.output.yaw_mode               = YAW_RELATIVE;



// #############################################################################
// ######  VELOCITY CONTROL  ###################################################
// #############################################################################
    // -----------------------------------------------------------------
    // ------ ROLL PID -------------------------------------------------
    // -----------------------------------------------------------------
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].p_gain           = 0.15f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].clip_min         = -0.5f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].clip_max         = 0.5f; //if 0.5 => 45° because it acts on the norm of upvector
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].integrator       = {};
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].integrator.gain      = 0.0125f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].integrator.clip_pre  = 1.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].integrator.accumulator   = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].integrator.clip      = 0.5f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].differentiator       = {};
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].differentiator.gain  = 0.001f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].differentiator.previous  = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].differentiator.clip  = 1.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].output           = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].error            = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].last_update_s      = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].dt_s           = 1;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[ROLL].soft_zone_width      = 0.2f;
    // -----------------------------------------------------------------
    // ------ PITCH PID ------------------------------------------------
    // -----------------------------------------------------------------
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].p_gain          = 0.3f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].clip_min        = -0.5f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].clip_max        = 0.5f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].integrator      = {};
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].integrator.gain     = 0.0125f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].integrator.clip_pre     = 1.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].integrator.accumulator  = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].integrator.clip     = 0.5f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].differentiator      = {};
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].differentiator.gain     = 0.001f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].differentiator.previous = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].differentiator.clip     = 1.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].output          = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].error           = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].last_update_s         = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].dt_s          = 1;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[PITCH].soft_zone_width     = 0.2f;
    // -----------------------------------------------------------------
    // ------ YAW PID --------------------------------------------------
    // -----------------------------------------------------------------
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].p_gain            = 1.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].clip_min          = -1;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].clip_max          = 1;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].integrator        = {};
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].integrator.gain       = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].integrator.clip_pre   = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].integrator.accumulator    = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].integrator.clip       = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].differentiator        = {};
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].differentiator.gain   = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].differentiator.previous   = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].differentiator.clip   = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].output            = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].error             = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].last_update_s       = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].dt_s            = 1;
    conf.stabiliser_stack.velocity_stabiliser.rpy_controller[YAW].soft_zone_width       = 0.0f;
    // ---------------------------------------------------------------------
    // ------ THRUST PID ---------------------------------------------------
    // ---------------------------------------------------------------------
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller             = {};
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.p_gain                  = 0.20f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.clip_min                = -0.9f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.clip_max                = 0.65f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.integrator              = {};
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.integrator.gain         = 0.01f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.integrator.clip_pre     = 2.0f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.integrator.accumulator  = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.integrator.clip         = 0.3f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.differentiator          = {};
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.differentiator.gain         = 0.08f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.differentiator.previous     = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.differentiator.clip         = 0.04f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.output                  = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.error                   = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.last_update_s             = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.dt_s                      = 1;
    conf.stabiliser_stack.velocity_stabiliser.thrust_controller.soft_zone_width         = 0.2f;
    // ---------------------------------------------------------------------
    // ------ OUTPUT -------------------------------------------------------
    // ---------------------------------------------------------------------
    conf.stabiliser_stack.velocity_stabiliser.output                    = {};
    conf.stabiliser_stack.velocity_stabiliser.output.rpy[0]                 = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.output.rpy[1]                 = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.output.rpy[2]                 = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.output.thrust                 = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.output.tvel[0]                = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.output.tvel[1]                = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.output.tvel[2]                = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.output.theading               = 0.0f;
    conf.stabiliser_stack.velocity_stabiliser.output.control_mode               = RATE_COMMAND_MODE;
    conf.stabiliser_stack.velocity_stabiliser.output.yaw_mode               = YAW_RELATIVE;




// #############################################################################
// ######  POSITION CONTROL  ###################################################
// #############################################################################
    // -----------------------------------------------------------------
    // ------ ROLL PID -------------------------------------------------
    // -----------------------------------------------------------------
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].p_gain           = 0.01f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].clip_min         = -0.5f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].clip_max         = 0.5f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].integrator       = {};
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].integrator.gain      = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].integrator.clip_pre  = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].integrator.accumulator   = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].integrator.clip      = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].differentiator       = {};
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].differentiator.gain  = 0.00005f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].differentiator.previous  = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].differentiator.clip  = 0.005f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].output           = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].error            = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].last_update_s      = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].dt_s           = 1;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[ROLL].soft_zone_width      = 0.2f;
    // -----------------------------------------------------------------
    // ------ PITCH PID ------------------------------------------------
    // -----------------------------------------------------------------
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].p_gain          = 0.01f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].clip_min        = -0.5f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].clip_max        = 0.5f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].integrator      = {};
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].integrator.gain     = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].integrator.clip_pre     = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].integrator.accumulator  = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].integrator.clip     = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].differentiator      = {};
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].differentiator.gain     = 0.00005f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].differentiator.previous = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].differentiator.clip     = 0.005f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].output          = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].error           = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].last_update_s         = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].dt_s          = 1;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[PITCH].soft_zone_width     = 0.2f;
    // -----------------------------------------------------------------
    // ------ YAW PID --------------------------------------------------
    // -----------------------------------------------------------------
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].p_gain            = 1.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].clip_min          = -1;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].clip_max          = 1;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].integrator        = {};
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].integrator.gain       = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].integrator.clip_pre   = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].integrator.accumulator    = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].integrator.clip       = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].differentiator        = {};
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].differentiator.gain   = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].differentiator.previous   = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].differentiator.clip   = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].output            = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].error             = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].last_update_s       = 0.0f;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].dt_s            = 1;
    conf.stabiliser_stack.position_stabiliser.rpy_controller[YAW].soft_zone_width       = 0.0f;
    // ---------------------------------------------------------------------
    // ------ THRUST PID ---------------------------------------------------
    // ---------------------------------------------------------------------
    conf.stabiliser_stack.position_stabiliser.thrust_controller             = {};
    conf.stabiliser_stack.position_stabiliser.thrust_controller.p_gain                  = 0.05f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.clip_min                = -0.9f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.clip_max                = 0.65f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.integrator              = {};
    conf.stabiliser_stack.position_stabiliser.thrust_controller.integrator.gain         = 0.00005f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.integrator.clip_pre     = 1.0f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.integrator.accumulator  = 0.0f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.integrator.clip         = 0.025f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.differentiator          = {};
    conf.stabiliser_stack.position_stabiliser.thrust_controller.differentiator.gain         = 0.005f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.differentiator.previous     = 0.0f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.differentiator.clip         = 0.01f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.output                  = 0.0f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.error                   = 0.0f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.last_update_s             = 0.0f;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.dt_s                      = 1;
    conf.stabiliser_stack.position_stabiliser.thrust_controller.soft_zone_width         = 0.2f;
    // ---------------------------------------------------------------------
    // ------ OUTPUT -------------------------------------------------------
    // ---------------------------------------------------------------------
    conf.stabiliser_stack.position_stabiliser.output                    = {};
    conf.stabiliser_stack.position_stabiliser.output.rpy[0]                 = 0.0f;
    conf.stabiliser_stack.position_stabiliser.output.rpy[1]                 = 0.0f;
    conf.stabiliser_stack.position_stabiliser.output.rpy[2]                 = 0.0f;
    conf.stabiliser_stack.position_stabiliser.output.thrust                 = 0.0f;
    conf.stabiliser_stack.position_stabiliser.output.tvel[0]                = 0.0f;
    conf.stabiliser_stack.position_stabiliser.output.tvel[1]                = 0.0f;
    conf.stabiliser_stack.position_stabiliser.output.tvel[2]                = 0.0f;
    conf.stabiliser_stack.position_stabiliser.output.theading               = 0.0f;
    conf.stabiliser_stack.position_stabiliser.output.control_mode               = RATE_COMMAND_MODE;
    conf.stabiliser_stack.position_stabiliser.output.yaw_mode               = YAW_RELATIVE;


    // #############################################################################
    // ######  MISC  ###############################################################
    // #############################################################################
    conf.stabiliser_stack.yaw_coordination_velocity                     = 1.5f;


    return conf;
};


#endif /* STABILISATION_COPTER_DEFAULT_CONFIG_H_ */
