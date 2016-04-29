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

#include "communication/state.hpp"
#include "control/manual_control.hpp"
#include "communication/remote.hpp"
#include "sensing/position_estimation.hpp"
#include "sensing/imu.hpp"

/**
 * \brief Defines the state machine structure
 */
class State_machine
{
public:
    /**
     * \brief Initialize the state machine
     *
     * \param state                     Pointer to the state structure
     * \param gps                       Pointer to the gps structure
     * \param imu                       Pointer to the imu structure
     * \param manual_control            Pointer to the manual_control structure
     *
     * \return  True if the init succeed, false otherwise
     */
    State_machine(  State& state,
                    const Position_estimation& position_estimation,
                    const Imu& imu,
                    Manual_control& manual_control);

    /**
     * \brief   Updates the state machine
     *
     * \param   state_machine           Pointer to the state machine structure
     *
     * \return Returns the result of the task
     */
    static bool update(State_machine* state_machine);

    /**
     * \brief   tries to change state.mav_mode to guided/unguided
     *
     * \details tries to set/clear the MAV_MODE_FLAG_GUIDED_ENABLED
     *          The following checks are performed:
     *          - position_estimation.healthy to pass to guided
     *
     * \param   guided      true to pass to guided, false to pass to unguided
     *
     * \return true if desired state was accepted; false if refused
     */
    bool set_mode_guided(bool guided);

    /**
     * \brief   tries to change state.mav_mode to stabilize/not stabilize
     *
     * \details tries to set/clear the MAV_MODE_FLAG_GUIDED_ENABLED
     *          The following checks are performed:
     *          - position_estimation.healthy to pass to stabilize
     *
     * \param   stabilize      true to pass to stabilize, false to pass to unguided
     *
     * \return true if desired state was accepted; false if refused
     */
    bool set_mode_stabilize(bool stabilize);

    State& state_;                                       ///< Pointer to the state structure
    const Position_estimation& position_estimation_;      ///< Pointer to the gps structure
    const Imu& imu_;                                     ///< Pointer to the imu structure
    Manual_control& manual_control_;                     ///< Pointer to the manual_control structure
private:
    /**
     * \brief Updates the custom flag and switch to critical state if needed
     *
     * \param current_custom_mode       Pointer to the current MAV custom mode
     * \param current_state             Pointer to the current MAV state
     */
    void set_custom_mode(mav_mode_custom_t *current_custom_mode, mav_state_t *current_state);
};










#endif // STATE_MACHINE_H_