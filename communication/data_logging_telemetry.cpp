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
 * \file data_logging_telemetry.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of sending periodic telemetric messages for
 * the data_logging module
 *
 ******************************************************************************/

#include "communication/data_logging_telemetry.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
}

/**
 * \brief   Toggle the data_logging
 *
 * \param   data_logging          The pointer to the data logging structure
 * \param   packet                  The pointer to the decoded MAVLink message long
 *
 * \return  The MAV_RESULT of the command
 */
static mav_result_t data_logging_telemetry_data_logging(Data_logging* data_logging, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t data_logging_telemetry_data_logging(Data_logging* data_logging, mavlink_command_long_t* packet)
{
    mav_result_t result;

    if (packet->param1 == 1)
    {
        if (data_logging->start())
        {
            print_util_dbg_print("Start logging from command message\r\n");
            result = MAV_RESULT_ACCEPTED;
        }
        else
        {
            result = MAV_RESULT_TEMPORARILY_REJECTED;
        }
    }
    else if (packet->param1 == 0)
    {
        if (data_logging->stop())
        {
            print_util_dbg_print("Stop logging from command message\r\n");
            result = MAV_RESULT_ACCEPTED;
        }
        else
        {
            result = MAV_RESULT_TEMPORARILY_REJECTED;
        }
    }
    else
    {
        result = MAV_RESULT_UNSUPPORTED;
    }

    return result;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool data_logging_telemetry_init(Data_logging* data_logging, Mavlink_message_handler* message_handler)
{
    bool init_success = true;

    // Add callbacks for data_logging commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_DO_SET_PARAMETER; // 180
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)            &data_logging_telemetry_data_logging;
    callbackcmd.module_struct  = (Mavlink_message_handler::handling_module_struct_t) data_logging;
    init_success &= message_handler->add_cmd_callback(&callbackcmd);

    return init_success;
}