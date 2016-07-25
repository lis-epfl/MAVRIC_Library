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
 * \file stabilisation_copter.hpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Nicolas Dousse
 *
 * \brief This file handles the stabilization of the platform
 *
 ******************************************************************************/


#ifndef STABILISATION_COPTER_HPP_
#define STABILISATION_COPTER_HPP_

#include "communication/mavlink_waypoint_handler.hpp"
#include "control/stabilisation.hpp"
#include "sensing/ins.hpp"

extern "C"
{
#include "control/control_command.h"
}

/**
 * \brief Motor Layout (cross or diag)
 */
typedef enum
{
    QUADCOPTER_MOTOR_LAYOUT_DIAG    = 0,                    ///< Motors in diagonal layout (rear_left / front-left / front-right / rear-right)
    QUADCOPTER_MOTOR_LAYOUT_CROSS   = 1,                    ///< Motors in cross layout (left / front / right / rear)
} quadcopter_motor_layout_t;


/**
 * \brief Structure containing the stacked controller
 */
typedef struct
{
    stabiliser_t rate_stabiliser;                               ///< The rate controller structure
    stabiliser_t attitude_stabiliser;                           ///< The attitude controller structure
    stabiliser_t velocity_stabiliser;                           ///< The velocity controller structure
    stabiliser_t position_stabiliser;                           ///< The position controller structure
    float yaw_coordination_velocity;                            ///< the yaw coordination value in velocity control mode
} stabiliser_stack_copter_t;

/**
 * \brief Structure containing the pointers to the data needed in this module
 */
typedef struct
{
    float thrust_hover_point;                                   ///< The hover point of the thrust
    quadcopter_motor_layout_t motor_layout;                     ///< Motor layout (cross or diag)
    stabiliser_stack_copter_t stabiliser_stack;                 ///< The pointer to the PID parameters values for the stacked controller
    float dt_s;                                                 ///< Time interval between to updates
    float last_update_s;                                        ///< Last update in seconds
    control_command_t* controls;                                ///< The pointer to the control structure
    const ahrs_t* ahrs;                                         ///< The pointer to the attitude estimation structure
    const INS* ins;                                             ///< The pointer to the Inertial Navigation System
    torque_command_t* torque_command;                           ///< The pointer to the torque command structure
    thrust_command_t* thrust_command;                           ///< The pointer to the thrust command structure
} stabilisation_copter_t;

/**
 * \brief Structure containing the configuration data
 */
typedef struct
{
    float thrust_hover_point;                                   ///< The hover point of the thrust
    quadcopter_motor_layout_t motor_layout;                     ///< Motor layout (cross or diag)
    stabiliser_stack_copter_t stabiliser_stack;                 ///< The pointer to the PID parameters values and output for the stacked controller
} stabilisation_copter_conf_t;

/**
 * \brief                           Initialize module stabilization
 *
 * \param   stabilisation_copter    The pointer to the stabilisation copter structure
 * \param   stabiliser_conf         The pointer to structure with all PID controllers
 * \param   controls                The pointer to the controlling inputs
 * \param   ahrs                    The pointer to the attitude estimation structure
 * \param   ins                     The pointer to the INS structure
 * \param   torque                  The pointer to the torque command values structure
 * \param   thrust                  The pointer to the thrust command values structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool stabilisation_copter_init(stabilisation_copter_t* stabilisation_copter, const stabilisation_copter_conf_t stabiliser_conf, control_command_t* controls, const ahrs_t* ahrs, const INS* ins, torque_command_t* torque, thrust_command_t* thrust);

/**
 * \brief                           Main Controller for controlling and stabilizing the quad
 *
 * \param   stabilisation_copter    The stabilisation structure
 */
void stabilisation_copter_cascade_stabilise(stabilisation_copter_t* stabilisation_copter);


#endif /* STABILISATION_COPTER_HPP_ */
