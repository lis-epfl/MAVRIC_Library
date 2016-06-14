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
#include "sensing/ins.hpp"
#include "sensing/imu.hpp"

extern "C"
{
#include "sensing/ahrs.hpp"
}

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
                    const INS& ins,
                    const Imu& imu,
                    const ahrs_t& ahrs,
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
     * \brief
     */
    bool set_ctrl_mode(Mav_mode mode);

    State& state_;                                       ///< State structure
    const INS& ins_;                                     ///< Position and velocity estimation
    const Imu& imu_;                                     ///< Inertial measurement unit
    const ahrs_t& ahrs_;                                 ///< Attitude estimation
    Manual_control& manual_control_;                     ///< Manual_control

private:
    /**
     * \brief Updates the custom flag and switch to critical state if needed
     *
     * \param current_custom_mode       Pointer to the current MAV custom mode
     * \param current_state             Pointer to the current MAV state
     */
    void set_custom_mode(Mav_mode::custom_mode_t *current_custom_mode, mav_state_t *current_state);

    /**
     * \brief   checks if it is possible to set/clear guided flag
     *
     * \details checks if it is possible to set/clear the MAV_MODE_FLAG_GUIDED_ENABLED of state_.mav_mode_
     *          The following checks are performed:
     *          - position_estimation.healthy to set guided
     *
     * \param   guided      true to set guided flag, false to clear unguided flag
     *
     * \return true if desired state is allowed, false otherwise
     */
    bool is_set_guided_allowed(bool guided);

    /**
     * \brief   checks if it is possible to set/clear stabilize flag
     *
     * \details checks if it is possible to set/clear the MAV_MODE_FLAG_STABILIZE of state_.mav_mode_
     *          The following checks are performed:
     *          - imu is ready to set stabilize
     *          - ahrs is ready to set stabilize
     *
     * \param   stabilize      true to set stabilize flag, false to clear flag
     *
     * \return true if desired state is allowed, false otherwise
     */
    bool is_set_stabilize_allowed(bool stabilize);
};










#endif // STATE_MACHINE_H_
