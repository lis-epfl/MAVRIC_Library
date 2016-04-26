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
 * \file state_machine.c
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *
 * \brief Handles transitions between states and modes
 *
 ******************************************************************************/


#include "communication/state_machine.hpp"
#include "communication/state.hpp"
#include "drivers/spektrum_satellite.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void State_machine::set_custom_mode(mav_mode_custom_t *current_custom_mode, mav_state_t *current_state)
{
    mav_mode_custom_t mode_custom_new = *current_custom_mode;
    mav_state_t state_new = *current_state;

    //check battery level
    if (state_.battery_.is_low())
    {
        if (state_.mav_state_ == MAV_STATE_ACTIVE)
        {
            print_util_dbg_print("Battery low! Performing critical landing.\r\n");
            state_new = MAV_STATE_CRITICAL;
        }
        mode_custom_new |= CUST_BATTERY_LOW;
    }
    else
    {
        mode_custom_new &= ~CUST_BATTERY_LOW;
    }

    // check connection with GND station
    if (state_.connection_lost)
    {
        if (state_.mav_state_ == MAV_STATE_ACTIVE)
        {
            print_util_dbg_print("Connection with GND station lost! Performing critical landing.\r\n");
            state_new = MAV_STATE_CRITICAL;
        }

        mode_custom_new |= CUST_HEARTBEAT_LOST;
    }
    else
    {
        mode_custom_new &= ~CUST_HEARTBEAT_LOST;
    }

    // check whether out_of_fence_1
    if (state_.out_of_fence_1)
    {
        if (state_.mav_state_ == MAV_STATE_ACTIVE)
        {
            print_util_dbg_print("Out of fence 1!\r\n");
            state_new = MAV_STATE_CRITICAL;
        }
        mode_custom_new |= CUST_FENCE_1;
    }
    else
    {
        mode_custom_new &= ~CUST_FENCE_1;
    }

    // check whether out_of_fence_2
    if (state_.out_of_fence_2)
    {
        if (state_.mav_state_ == MAV_STATE_ACTIVE)
        {
            print_util_dbg_print("Out of fence 2!\r\n");
            state_new = MAV_STATE_CRITICAL;
        }
        mode_custom_new |= CUST_FENCE_2;
    }
    else
    {
        mode_custom_new &= ~CUST_FENCE_2;
    }

    // check GPS status
    if (!position_estimation_.healthy())
    {
        if (state_.mav_state_ == MAV_STATE_ACTIVE)
        {
            print_util_dbg_print("GPS bad!\r\n");
            state_new = MAV_STATE_CRITICAL;
        }
        mode_custom_new |= CUST_GPS_BAD;
    }
    else
    {
        mode_custom_new &= ~CUST_GPS_BAD;
    }

    *current_custom_mode = mode_custom_new;
    *current_state = state_new;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

State_machine::State_machine(State& state,
                            const Position_estimation& position_estimation,
                            const Imu& imu,
                            const Ahrs_ekf& ahrs_ekf,
                            Manual_control& manual_control) :
    state_(state),
    position_estimation_(position_estimation),
    imu_(imu),
    ahrs_ekf_(ahrs_ekf),
    manual_control_(manual_control)
{}

bool State_machine::update(State_machine* state_machine)
{
    mav_mode_t mode_new;
    signal_quality_t rc_check;

    // Get current mode and state
    const mav_mode_t mode_current = state_machine->state_.mav_mode();
    const mav_state_t state_current = state_machine->state_.mav_state_;

    // By default, set new state equal to current state
    mav_state_t state_new = state_current;
    mav_mode_custom_t mode_custom_new = state_machine->state_.mav_mode_custom;


    // Get remote signal strength
    rc_check = state_machine->manual_control_.get_signal_strength();

    mode_new = state_machine->manual_control_.get_mode_from_source(mode_current);

    state_machine->state_.battery_.update();

    state_machine->state_.connection_status();

    // try arming/disarming, if not allowed, reset flag in mode_new
    if(!state_machine->state_.set_armed(mav_modes_is_armed(mode_new)))
    {
        mode_new ^= MAV_MODE_FLAG_SAFETY_ARMED; // toggle armed flag
    }

    // Change state according to signal strength
    switch (state_current)
    {
        case MAV_STATE_UNINIT:
        case MAV_STATE_BOOT:
        case MAV_STATE_POWEROFF:
        case MAV_STATE_ENUM_END:
            break;

        case MAV_STATE_CALIBRATING:
            if (state_machine->imu_.is_ready() && state_machine->ahrs_ekf_.is_ready())
            {
                state_new = MAV_STATE_STANDBY;
            }
            break;

        case MAV_STATE_STANDBY:
            //disable out of fence checks
            state_machine->state_.out_of_fence_1 = false;
            state_machine->state_.out_of_fence_2 = false;

            if (mav_modes_is_armed(mode_new))
            {
                print_util_dbg_print("Switching from state_machine.\r\n");
                state_machine->state_.switch_to_active_mode(&state_new);

                mode_custom_new = CUSTOM_BASE_MODE;
            }

            if (!state_machine->imu_.is_ready() || !state_machine->ahrs_ekf_.is_ready())
            {
                state_new = MAV_STATE_CALIBRATING;
            }

            break;

        case MAV_STATE_ACTIVE:
            if ((state_machine->manual_control_.mode_source == Manual_control::MODE_SOURCE_REMOTE) || (state_machine->manual_control_.mode_source == Manual_control::MODE_SOURCE_JOYSTICK))
            {
                // check connection with remote
                if ((state_machine->manual_control_.mode_source == Manual_control::MODE_SOURCE_REMOTE) && (rc_check != SIGNAL_GOOD))
                {
                    print_util_dbg_print("Remote control signal lost! Returning to home and land.\r\n");
                    state_new = MAV_STATE_CRITICAL;
                    mode_custom_new |= CUST_REMOTE_LOST;
                }
                else
                {
                    mode_custom_new &= ~CUST_REMOTE_LOST;

                    if (!mav_modes_is_armed(mode_new))
                    {
                        state_new = MAV_STATE_STANDBY;
                        print_util_dbg_print("Switching off motors from state_machine!\r\n");
                    }
                }
            }

            state_machine->set_custom_mode(&mode_custom_new, &state_new);

            break;

        case MAV_STATE_CRITICAL:
            switch (rc_check)
            {
                case SIGNAL_GOOD:
                    if (!state_machine->state_.battery_.is_low() &&
                            !state_machine->state_.connection_lost &&
                            !state_machine->state_.out_of_fence_1 &&
                            !state_machine->state_.out_of_fence_2 &&
                            state_machine->position_estimation_.healthy())
                    {
                        state_new = MAV_STATE_ACTIVE;
                        // Reset all custom flags except collision avoidance flag
                        mode_custom_new &= static_cast<mav_mode_custom_t>(0xFFFFF820);
                    }
                    break;

                case SIGNAL_BAD:
                    // Stay in critical mode
                    break;

                case SIGNAL_LOST:
                    // If in manual mode, do emergency landing (cut off motors)
                    if (mav_modes_is_manual(mode_current) && (!mav_modes_is_stabilise(mode_current)))
                    {
                        print_util_dbg_print("Switch to Emergency mode!\r\n");
                        state_new = MAV_STATE_EMERGENCY;
                    }
                    // If in another mode, stay in critical mode
                    // higher level navigation module will take care of coming back home
                    break;
            }

            state_machine->set_custom_mode(&mode_custom_new, &state_new);

            if (!mav_modes_is_armed(mode_new))
            {
                state_new = MAV_STATE_STANDBY;
            }
            break;

        case MAV_STATE_EMERGENCY:
            // Recovery is not possible -> switch off motors
            mode_new &= ~MAV_MODE_FLAG_SAFETY_ARMED;

            if (!state_machine->state_.battery_.is_low())
            {
                // To get out of this state, if we are in the wrong use_mode_from_remote
                if (state_machine->manual_control_.mode_source != Manual_control::MODE_SOURCE_REMOTE)
                {
                    state_new = MAV_STATE_STANDBY;
                }

                switch (rc_check)
                {
                    case SIGNAL_GOOD:
                        state_new = MAV_STATE_STANDBY;
                        break;

                    case SIGNAL_BAD:
                        // Stay in emergency mode
                        break;

                    case SIGNAL_LOST:
                        // Stay in emergency mode
                        break;
                }
            }
            break;
    }

    // Check simulation mode
    if ( mav_modes_is_hil(state_machine->state_.mav_mode()) == true )
    {
        mode_new |= MAV_MODE_FLAG_HIL_ENABLED;
    }
    else
    {
        mode_new &= ~MAV_MODE_FLAG_HIL_ENABLED;
    }

    // Finally, write new modes and states
    state_machine->state_.mav_mode_       = mode_new;
    state_machine->state_.mav_state_       = state_new;
    state_machine->state_.mav_mode_custom = mode_custom_new;

    // overwrite internal state of joystick
    state_machine->manual_control_.set_mode_of_source(mode_new);

    return true;
}





bool State_machine::set_mode_guided(bool guided)
{
    // if already in desired state, return true
    if(state_.guided() == guided)
    {
        return true;
    }

    if(guided)
    {
        // if position_estimation is not healthy, abort
        if(!position_estimation_.healthy())
        {
            print_util_dbg_print("[STATE_MACHINE]: prevented passing to guided because position estimation is not healthy\r\n");
            return false;
        }

        // set guided flag
        state_.mav_mode_ |= MAV_MODE_FLAG_GUIDED_ENABLED;
        print_util_dbg_print("[STATE_MACHINE]: passing to guided mode\r\n");
    }
    else
    {
        // clear guided flag
        state_.mav_mode_ &= ~MAV_MODE_FLAG_GUIDED_ENABLED;
        print_util_dbg_print("[STATE_MACHINE]: leave guided mode\r\n");
    }

    return true;
}

