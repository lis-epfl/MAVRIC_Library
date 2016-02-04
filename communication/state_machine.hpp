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
 * \file state_machine.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *
 * \brief Handles transitions between states and modes
 *
 ******************************************************************************/


#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include "communication/mavlink_waypoint_handler.hpp"
#include "communication/state.hpp"
#include "control/manual_control.hpp"
#include "communication/remote.hpp"
#include "drivers/gps.hpp"
#include "sensing/imu.hpp"

/**
 * \brief Defines the state machine structure
 */
typedef struct
{
    mavlink_waypoint_handler_t* waypoint_handler;       ///< Pointer to the mavlink waypoint handler structure
    State* state;                                       ///< Pointer to the state structure
    remote_t* remote;                                   ///< Pointer to the remote structure
    const Gps* gps;                                     ///< Pointer to the gps structure
    const Imu* imu;                                     ///< Pointer to the imu structure
    manual_control_t* manual_control;                   ///< Pointer to the manual_control structure
} state_machine_t;


/**
 * \brief Initialize the state machine
 *
 * \param state_machine             Pointer to the state machine structure
 * \param state                     Pointer to the state structure
 * \param gps                       Pointer to the gps structure
 * \param imu                       Pointer to the imu structure
 * \param manual_control            Pointer to the manual_control structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool state_machine_init(state_machine_t* state_machine,
                        State* state,
                        const Gps* gps,
                        const Imu* imu,
                        manual_control_t* manual_control);

/**
 * \brief   Updates the state and mode of the UAV (not implemented yet)
 *
 * \param   state_machine           Pointer to the state machine structure
 *
 * \return Returns the result of the task
 */
bool state_machine_set_mav_mode_n_state(state_machine_t* state_machine);

/**
 * \brief   Updates the state machine
 *
 * \param   state_machine           Pointer to the state machine structure
 *
 * \return Returns the result of the task
 */
bool state_machine_update(state_machine_t* state_machine);


#endif // STATE_MACHINE_H_