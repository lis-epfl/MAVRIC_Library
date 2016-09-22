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
 * \file position_estimation_telemetry.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of sending periodic telemetric messages for
 * the position estimation
 *
 ******************************************************************************/


#include "sensing/position_estimation_telemetry.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/print_util.hpp"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Position estimation update step, performing position estimation then position correction (function to be used)
 *
 * \param   pos_est                 The pointer to the position estimation structure
 * \param   packet                  The pointer to the decoded MAVLink command long message
 */
static mav_result_t position_estimation_set_new_home_position(Position_estimation* pos_est, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t position_estimation_set_new_home_position(Position_estimation* pos_est, mavlink_command_long_t* packet)
{
    bool result = false;

    if (packet->param1 == 1)
    {
        result = pos_est->set_home_to_current_position();
    }
    else
    {
        // Set new home position from msg
        global_position_t new_home;
        new_home.latitude = packet->param5;
        new_home.longitude = packet->param6;
        new_home.altitude = packet->param7;

        result = pos_est->set_home_position_global(new_home);
    }

    return  result ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool position_estimation_telemetry_init(Position_estimation* pos_est, Mavlink_message_handler* message_handler)
{
    bool init_success = true;

    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id    = MAV_CMD_DO_SET_HOME; // 179
    callbackcmd.sysid_filter  = MAV_SYS_ID_ALL;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL;
    callbackcmd.function      = (Mavlink_message_handler::cmd_callback_func_t)   &position_estimation_set_new_home_position;
    callbackcmd.module_struct =                                     pos_est;
    init_success &= message_handler->add_cmd_callback(&callbackcmd);

    return init_success;
}
