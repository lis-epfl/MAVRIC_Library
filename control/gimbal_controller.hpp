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
 * \file gimbal_controller.hpp
 *
 * \author MAV'RIC Team
 * \author Alexandre Cherpillod
 *
 * \brief   Gimbal control management
 *
 ******************************************************************************/


#ifndef GIMBAL_CONTROLLER_H_
#define GIMBAL_CONTROLLER_H_

#include "control/control_command.h"
#include "control/pid_controller.h"


/**
 * \brief Gimbal controller structure
 * So far, the command are only sent to an external gimbal board which handles the closed-loop itself
 */
typedef struct
{
    attitude_command_t			attitude_command;	///< Attitude command (input)
} gimbal_controller_t;


/**
 * \brief Gimbal controller configuration
 */
typedef struct
{
    attitude_command_t   		attitude_command_config;  ///< Initial command sent to the external gimbal board
} gimbal_controller_conf_t;


/**
 * \brief                       Initializes the gimbal controller structure
 *
 * \param   controller          Pointer to data structure
 * \param   config              Pointer to configuration
 */
void gimbal_controller_init(gimbal_controller_t* controller,
								const gimbal_controller_conf_t config);


/**
 * \brief                   Main update function - so far useless
 *
 * \param   controller      Pointer to data structure
 */
void gimbal_controller_update(gimbal_controller_t* controller);

#endif /* GIMBAL_CONTROLLER_H_ */
