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

void State_machine::set_custom_mode(Mav_mode::custom_mode_t *current_custom_mode, mav_state_t *current_state)
{
    Mav_mode::custom_mode_t mode_custom_new = *current_custom_mode;
    mav_state_t state_new = *current_state;

    //check battery level
    if (state_.battery_.is_low())
    {
        if (state_.mav_state_ == MAV_STATE_ACTIVE)
        {
            print_util_dbg_print("Battery low! Performing critical landing.\r\n");
            state_new = MAV_STATE_CRITICAL;
        }
        mode_custom_new |= Mav_mode::CUST_BATTERY_LOW;
    }
    else
    {
        mode_custom_new &= ~Mav_mode::CUST_BATTERY_LOW;
    }

    // check connection with GND station
    if (state_.connection_lost)
    {
        if (state_.mav_state_ == MAV_STATE_ACTIVE)
        {
            print_util_dbg_print("Connection with GND station lost! Performing critical landing.\r\n");
            state_new = MAV_STATE_CRITICAL;
        }

        mode_custom_new |= Mav_mode::CUST_HEARTBEAT_LOST;
    }
    else
    {
        mode_custom_new &= ~Mav_mode::CUST_HEARTBEAT_LOST;
    }

    // check whether out_of_fence_1
    if (state_.out_of_fence_1)
    {
        if (state_.mav_state_ == MAV_STATE_ACTIVE)
        {
            print_util_dbg_print("Out of fence 1!\r\n");
            state_new = MAV_STATE_CRITICAL;
        }
        mode_custom_new |= Mav_mode::CUST_FENCE_1;
    }
    else
    {
        mode_custom_new &= ~Mav_mode::CUST_FENCE_1;
    }

    // check whether out_of_fence_2
    if (state_.out_of_fence_2)
    {
        if (state_.mav_state_ == MAV_STATE_ACTIVE)
        {
            print_util_dbg_print("Out of fence 2!\r\n");
            state_new = MAV_STATE_CRITICAL;
        }
        mode_custom_new |= Mav_mode::CUST_FENCE_2;
    }
    else
    {
        mode_custom_new &= ~Mav_mode::CUST_FENCE_2;
    }

    // check GPS status
    if (!position_estimation_.healthy())
    {
        if (state_.mav_state_ == MAV_STATE_ACTIVE)
        {
            print_util_dbg_print("GPS bad!\r\n");
            state_new = MAV_STATE_CRITICAL;
        }
        mode_custom_new |= Mav_mode::CUST_GPS_BAD;
    }
    else
    {
        mode_custom_new &= ~Mav_mode::CUST_GPS_BAD;
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
                            const ahrs_t& ahrs,
                            Manual_control& manual_control) :
    state_(state),
    position_estimation_(position_estimation),
    imu_(imu),
    ahrs_(ahrs),
    manual_control_(manual_control)
{}

bool State_machine::update(State_machine* state_machine)
{
    Mav_mode mode_new;
    signal_quality_t rc_check;

    // Get current mode and state
    const Mav_mode mode_current = state_machine->state_.mav_mode();
    const mav_state_t state_current = state_machine->state_.mav_state_;

    // By default, set new state equal to current state
    mav_state_t state_new = state_current;
    Mav_mode::custom_mode_t mode_custom_new = state_machine->state_.mav_mode_custom;


    // Get remote signal strength
    rc_check = state_machine->manual_control_.get_signal_strength();

    mode_new = state_machine->manual_control_.get_mode_from_source(mode_current);

    state_machine->state_.battery_.update();

    state_machine->state_.connection_status();

    // try passing to/leaving stabilize mode if requested
    if(!state_machine->set_mode_stabilize(mode_new.is_stabilize()))
    {
        mode_new.set_stabilize_flag(!mode_new.is_stabilize()); // toggle stabilize flag
    }

    // try passing to/leaving guided mode if requested
    if(!state_machine->set_mode_guided(mode_new.is_guided()))
    {
        mode_new.set_guided_flag(!mode_new.is_guided()); // toggle guided flag
    }

    // try arming/disarming, if not allowed, reset flag in mode_new
    if(!state_machine->state_.set_armed(mode_new.is_armed()))
    {
        mode_new.set_armed_flag(!mode_new.is_armed()); // toggle armed flag
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
            if (state_machine->imu_.is_ready() && (state_machine->ahrs_.internal_state == AHRS_READY))
            {
                state_new = MAV_STATE_STANDBY;
            }
            break;

        case MAV_STATE_STANDBY:
            //disable out of fence checks
            state_machine->state_.out_of_fence_1 = false;
            state_machine->state_.out_of_fence_2 = false;

            if (mode_new.is_armed())
            {
                print_util_dbg_print("Switching from state_machine.\r\n");
                state_machine->state_.switch_to_active_mode(&state_new);

                mode_custom_new = Mav_mode::CUSTOM_BASE_MODE;
            }

            if (!state_machine->imu_.is_ready() || !(state_machine->ahrs_.internal_state == AHRS_READY))
            {
                state_new = MAV_STATE_CALIBRATING;
            }

            break;

        case MAV_STATE_ACTIVE:
            if ((state_machine->manual_control_.mode_source() == Manual_control::MODE_SOURCE_REMOTE) || (state_machine->manual_control_.mode_source() == Manual_control::MODE_SOURCE_JOYSTICK))
            {
                // check connection with remote
                if ((state_machine->manual_control_.mode_source() == Manual_control::MODE_SOURCE_REMOTE) && (rc_check != SIGNAL_GOOD))
                {
                    print_util_dbg_print("Remote control signal lost! Returning to home and land.\r\n");
                    state_new = MAV_STATE_CRITICAL;
                    mode_custom_new |= Mav_mode::CUST_REMOTE_LOST;
                }
                else
                {
                    mode_custom_new &= ~Mav_mode::CUST_REMOTE_LOST;

                    if (!mode_new.is_armed())
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
                        mode_custom_new &= static_cast<Mav_mode::custom_mode_t>(0xFFFFF820);
                    }
                    break;

                case SIGNAL_BAD:
                    // Stay in critical mode
                    break;

                case SIGNAL_LOST:
                    // If in manual mode, do emergency landing (cut off motors)
                    if (mode_current.is_manual())
                    {
                        print_util_dbg_print("Switch to Emergency mode!\r\n");
                        state_new = MAV_STATE_EMERGENCY;
                    }
                    // If in another mode, stay in critical mode
                    // higher level navigation module will take care of coming back home
                    break;
            }

            state_machine->set_custom_mode(&mode_custom_new, &state_new);

            if (!mode_new.is_armed())
            {
                state_new = MAV_STATE_STANDBY;
            }
            break;

        case MAV_STATE_EMERGENCY:
            // Recovery is not possible -> switch off motors
            mode_new.set_armed_flag(false);

            if (!state_machine->state_.battery_.is_low())
            {
                // To get out of this state, if we are in the wrong use_mode_from_remote
                if (state_machine->manual_control_.mode_source() != Manual_control::MODE_SOURCE_REMOTE)
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
    mode_new.set_hil_flag(state_machine->state_.mav_mode().is_hil());

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
    if(state_.is_guided() == guided)
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
        state_.mav_mode_.set_guided_flag(true);
        print_util_dbg_print("[STATE_MACHINE]: passing to guided mode\r\n");
    }
    else
    {
        // clear guided flag
        state_.mav_mode_.set_guided_flag(false);
        print_util_dbg_print("[STATE_MACHINE]: leave guided mode\r\n");
    }

    return true;
}

bool State_machine::set_mode_stabilize(bool stabilize)
{
    // if already in desired state, return true
    if(state_.is_stabilize() == stabilize)
    {
        return true;
    }

    if(stabilize)
    {
        // if position_estimation is not healthy, abort
        if(!position_estimation_.healthy())
        {
            print_util_dbg_print("[STATE_MACHINE]: prevented passing to stabilize because position estimation is not healthy\r\n");
            return false;
        }

        // set stabilize flag
        state_.mav_mode_.set_stabilize_flag(true);
        print_util_dbg_print("[STATE_MACHINE]: passing to stabilize mode\r\n");
    }
    else
    {
        // clear stabilize flag
        state_.mav_mode_.set_stabilize_flag(false);
        print_util_dbg_print("[STATE_MACHINE]: leave stabilize mode\r\n");
    }

    return true;
}
