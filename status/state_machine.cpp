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
 * \file state_machine.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *
 * \brief Handles transitions between states and modes
 *
 ******************************************************************************/


#include "status/state_machine.hpp"
#include "status/state.hpp"
#include "drivers/spektrum_satellite.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/print_util.hpp"

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

    // check whether out_of_safety_geofence
    if (state_.out_of_safety_geofence)
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

    // check whether out_of_emergency_geofence
    if (state_.out_of_emergency_geofence)
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
    if (!ins_.is_healthy(INS::healthy_t::XYZ_REL_POSITION))
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
                            const INS& ins,
                            const Imu& imu,
                            const AHRS& ahrs,
                            Manual_control& manual_control,
                            Geofence& safety_geofence,
                            Geofence& emergency_geofence,
                            State_display& state_display,
                            conf_t config):
    config_(config),
    state_(state),
    ins_(ins),
    imu_(imu),
    ahrs_(ahrs),
    manual_control_(manual_control),
    safety_geofence_(safety_geofence),
    emergency_geofence_(emergency_geofence),
    state_display_(state_display)
{}

bool State_machine::update_task(State_machine* state_machine)
{
    return state_machine->update();
}

bool State_machine::update(void)
{
    Mav_mode mode_new;
    signal_quality_t rc_check;

    // Get current mode and state
    const Mav_mode mode_current = state_.mav_mode();
    const mav_state_t state_current = state_.mav_state_;

    // By default, set new state equal to current state
    mav_state_t state_new = state_current;
    Mav_mode::custom_mode_t mode_custom_new = state_.mav_mode_custom;

    // Check geofence breach
    state_.out_of_safety_geofence    = !safety_geofence_.is_allowed(ins_.position_gf());
    state_.out_of_emergency_geofence = !emergency_geofence_.is_allowed(ins_.position_gf());

    // Get remote signal strength
    rc_check = manual_control_.get_signal_strength();

    mode_new = manual_control_.get_mode_from_source(mode_current);

    // Update battery state
    state_.battery_.update();

    state_.connection_status();

    // try changing the control mode, if not allowed, reset flags of mode_new
    if(!set_ctrl_mode(mode_new))
    {
        mode_new.set_ctrl_mode(mode_current.ctrl_mode());
    }

    // try arming/disarming, if not allowed, reset flag in mode_new
    if(!state_.set_armed(mode_new.is_armed()))
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
            if (imu_.is_ready() && (ahrs_.is_healthy()))
            {
                state_new = MAV_STATE_STANDBY;
            }
            break;

        case MAV_STATE_STANDBY:
            //disable out of fence checks
            state_.out_of_safety_geofence = false;
            state_.out_of_emergency_geofence = false;

            if (mode_new.is_armed() && config_.is_emergency_disarming_enabled)
            {
                print_util_dbg_print("Switching from state_machine.\r\n");
                state_.switch_to_active_mode(&state_new);

                mode_custom_new = Mav_mode::CUSTOM_BASE_MODE;
            }

            if (!imu_.is_ready() || !(ahrs_.is_healthy()))
            {
                state_new = MAV_STATE_CALIBRATING;
            }

            break;

        case MAV_STATE_ACTIVE:
            if ((manual_control_.mode_source() == Manual_control::MODE_SOURCE_REMOTE) || (manual_control_.mode_source() == Manual_control::MODE_SOURCE_JOYSTICK))
            {
                // check connection with remote
                if ((manual_control_.mode_source() == Manual_control::MODE_SOURCE_REMOTE) && (rc_check != SIGNAL_GOOD))
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

            set_custom_mode(&mode_custom_new, &state_new);

            break;

        case MAV_STATE_CRITICAL:
            switch (rc_check)
            {
                case SIGNAL_GOOD:
                    if (!state_.battery_.is_low() &&
                            !state_.connection_lost &&
                            !state_.out_of_safety_geofence &&
                            !state_.out_of_emergency_geofence &&
                            ins_.is_healthy(INS::healthy_t::XYZ_REL_POSITION))
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

            set_custom_mode(&mode_custom_new, &state_new);

            if (!mode_new.is_armed())
            {
                state_new = MAV_STATE_STANDBY;
            }
            break;

        case MAV_STATE_EMERGENCY:
            // Recovery is not possible -> switch off motors
            state_.mav_mode().set_armed_flag(false);

            if (!state_.battery_.is_low())
            {
                // To get out of this state, if we are in the wrong use_mode_from_remote
                if (manual_control_.mode_source() != Manual_control::MODE_SOURCE_REMOTE)
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

    // Finally, write new modes and states
    state_.mav_state_       = state_new;
    state_.mav_mode_custom = mode_custom_new;
    state_display_.set_state(state_new);

    // overwrite internal state of joystick
    manual_control_.set_mode_of_source(state_.mav_mode_);

    return true;
}


bool State_machine::set_ctrl_mode(Mav_mode mode)
{
    bool success = true;

    // check if we can set/clear stabilize flag
    if(!is_set_stabilize_allowed(mode.is_stabilize()))
    {
        print_util_dbg_print("[STATE_MACHINE]: prevented passing to stabilize because imu or ahrs not ready\r\n");
        success = false;
    }

    // check if we can set/clear guided flag
    if(!is_set_guided_allowed(mode.is_guided()))
    {
        print_util_dbg_print("[STATE_MACHINE]: prevented passing to guided because position estimation is not healthy\r\n");
        success = false;
    }

    if(success)
    {
        state_.mav_mode_.set_ctrl_mode(static_cast<Mav_mode::ctrl_mode_t>(mode.bits()));
        state_.mav_mode_.set_custom_flag(mode.is_custom());
        state_.mav_mode_.set_test_flag(mode.is_test());
    }

    return success;
}


bool State_machine::is_set_guided_allowed(bool guided)
{
    bool success = true;

    // if already in desired state, skip tests and return true
    if(state_.is_guided() != guided)
    {
        // if we change to guided, test if position estimation is healthy
        if(success & guided)
        {
            // if position_estimation is not healthy, abort
            success &= ins_.is_healthy(INS::healthy_t::XYZ_VELOCITY);
        }
    }
    return success;
}

bool State_machine::is_set_stabilize_allowed(bool stabilize)
{
    bool success = true;

    // if already in desired state, skip tests and return true
    if(state_.is_stabilize() != stabilize)
    {
        // if we change to stabilize, test if imu is healthy and ahrs is ready
        if(success & stabilize)
        {
            // if position_estimation is not healthy, abort
            success &= (imu_.is_ready() && ahrs_.is_healthy());
        }
    }
    return success;
}
