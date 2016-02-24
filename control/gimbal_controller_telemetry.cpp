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
 * \file gimbal_control_telemetry.hpp
 *
 * \author MAV'RIC Team
 * \author Alexandre Cherpillod
 *
 * \brief This module takes care of handling the received packet for the gimbal
 *
 ******************************************************************************/


#include "control/gimbal_controller_telemetry.hpp"
#include "control/gimbal_controller.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
}
//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Parse received MAVLink message in structure
 *
 * \param   gimbal	    The pointer to the gimbal structure
 * \param   sysid       The sysid of the system
 * \param   msg         The pointer to the MAVLink message received
 *
 * \return  The MAV_RESULT of the command
 */
static void gimbal_telemetry_parse_msg(gimbal_controller_t* gimbal, uint32_t sysid, mavlink_message_t* msg);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void gimbal_telemetry_parse_msg(gimbal_controller_t* gimbal, uint32_t sysid, mavlink_message_t* msg)
{
	mavlink_gimbal_command_t packet;
	mavlink_msg_gimbal_command_decode(msg, &packet);

	gimbal->attitude_command.rpy[0] = packet.angle[0];
	gimbal->attitude_command.rpy[1] = packet.angle[1];
	gimbal->attitude_command.rpy[2] = packet.angle[2];

	print_util_dbg_print("received gimbal commands\r\n");
	print_util_dbg_print("roll ");
	print_util_dbg_putfloat(gimbal->attitude_command.rpy[0], 4);
	print_util_dbg_print("\r\npitch ");
	print_util_dbg_putfloat(gimbal->attitude_command.rpy[1], 4);
	print_util_dbg_print("\r\nyaw ");
	print_util_dbg_putfloat(gimbal->attitude_command.rpy[2], 4);
	print_util_dbg_print("\r\n");
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool gimbal_controller_telemetry_init(gimbal_controller_t* gimbal_controller, mavlink_message_handler_t* message_handler)
{
    bool init_success = true;

    // Add callbacks for waypoint handler messages requests
    mavlink_message_handler_msg_callback_t callback;

    callback.message_id     = MAVLINK_MSG_ID_GIMBAL_COMMAND; //185
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &gimbal_telemetry_parse_msg;
    callback.module_struct  = (handling_module_struct_t)        gimbal_controller;
    init_success &= mavlink_message_handler_add_msg_callback(message_handler, &callback);

    return init_success;
}


void gimbal_controller_telemetry_send(const gimbal_controller_t* gimbal_controller, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    ;
}
