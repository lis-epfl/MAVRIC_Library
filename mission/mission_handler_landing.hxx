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
 * \file mission_handler_landing.hxx
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler functions for the landing state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_LANDING_HXX__
#define MISSION_HANDLER_LANDING_HXX__

template <class T1, class T2>
Mission_handler_landing<T1, T2>::Mission_handler_landing(   T1& desc_to_small_alt_controller,
                                                            T2& desc_to_ground_controller,
                                                            const INS& ins,
                                                            Navigation& navigation,
                                                            State& state,
                                                            conf_t config):
            Mission_handler(),
            desc_to_small_alt_controller_(desc_to_small_alt_controller),
            desc_to_ground_controller_(desc_to_ground_controller),
            ins_(ins),
            navigation_(navigation),
            state_(state)
{
    waypoint_ = Waypoint (  MAV_FRAME_LOCAL_NED,
                            MAV_CMD_NAV_LAND,
                            0,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f);
    auto_landing_behavior_ = DESCENT_TO_SMALL_ALTITUDE;
    LPF_gain_ = config.LPF_gain;
}

template <class T1, class T2>
bool Mission_handler_landing<T1, T2>::can_handle(const Waypoint& wpt) const
{
    bool handleable = false;

    uint16_t cmd = wpt.command();
    if (cmd == MAV_CMD_NAV_LAND)
    {
        handleable = true;
    }

    return handleable;
}

template <class T1, class T2>
bool Mission_handler_landing<T1, T2>::setup(Mission_planner& mission_planner, const Waypoint& wpt)
{
    bool success = true;

    print_util_dbg_print("Automatic-landing: descent_to_small_altitude\r\n");
    auto_landing_behavior_ = DESCENT_TO_SMALL_ALTITUDE;
    state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
    state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_SMALL_ALTITUDE;
    navigation_.set_waiting_at_waypoint(false);
    is_landed_ = false;

    waypoint_ = wpt;

    return success;
}

template <class T1, class T2>
int Mission_handler_landing<T1, T2>::update(Mission_planner& mission_planner)
{
    /*****************************
    Handle internal landing states 
    *****************************/
    // Determine if we should switch between the landing states
    bool next_state = false;
    if (auto_landing_behavior_ == DESCENT_TO_GND)
    {
        alt_lpf_ = LPF_gain_ * (alt_lpf_) + (1.0f - LPF_gain_) * ins_.position_lf()[Z];
        if ((ins_.position_lf()[Z] > -0.1f) && (maths_f_abs(ins_.position_lf()[Z] - alt_lpf_) <= 0.2f))
        {
            next_state = true;
        }
    }
    else if (auto_landing_behavior_ == DESCENT_TO_SMALL_ALTITUDE)
    {
        if (maths_f_abs(ins_.position_lf()[Z] - navigation_.takeoff_altitude/2.0f) < 0.5f)
        {
            next_state = true;
        }
    }

    // If we are switching between states, ... then switch
    if (next_state)
    {
        switch (auto_landing_behavior_)
        {
            case DESCENT_TO_SMALL_ALTITUDE:
                print_util_dbg_print("Automatic-landing: descent_to_GND\r\n");
                auto_landing_behavior_ = DESCENT_TO_GND;
                state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
                state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_GND;
                alt_lpf_ = ins_.position_lf()[Z];
                break;

            case DESCENT_TO_GND:
                print_util_dbg_print("Auto-landing: disarming motors \r\n");
                auto_landing_behavior_ = DESCENT_TO_SMALL_ALTITUDE;
                //Do not reset custom flag here, to be able to check after landing
                // in case something went wrong. Is reset while arming
                state_.set_armed(false);
                state_.mav_state_ = MAV_STATE_STANDBY;
                navigation_.set_waiting_at_waypoint(true);
                is_landed_ = true;
                break;
        }
    }
    
    
    /*************
    Set controller 
    *************/
    bool ret = false;
    switch (auto_landing_behavior_)
    {
        case DESCENT_TO_SMALL_ALTITUDE:
            ret = set_desc_to_small_alt_control_command(mission_planner);
            break;

        case DESCENT_TO_GND:
            ret = set_desc_to_ground_control_command(mission_planner);
            break;
    }

    /********************
    Determine status code 
    ********************/
    if (waypoint_.autocontinue() == 1 && is_landed_)
    {
        return 1;
    }

    // Handle control command failed status
    if (!ret)
    {
        return -1;
    }

    return 0;
}

template <class T1, class T2>
Mission_planner::internal_state_t Mission_handler_landing<T1, T2>::handler_mission_state() const
{
    return Mission_planner::POSTMISSION;
}

#endif // MISSION_HANDLER_LANDING_HXX__
