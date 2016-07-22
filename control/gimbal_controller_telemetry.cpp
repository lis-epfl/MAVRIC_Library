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


void gimbal_telemetry_parse_msg(Gimbal_controller* gimbal_controller, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_gimbal_command_t packet;
    mavlink_msg_gimbal_command_decode(msg, &packet);

    //store the received angle as the desired ones
    gimbal_controller->attitude_command_desired_.rpy[0] = packet.angle[0];
    gimbal_controller->attitude_command_desired_.rpy[1] = packet.angle[1];
    gimbal_controller->attitude_command_desired_.rpy[2] = packet.angle[2];
}


bool gimbal_controller_telemetry_init(Gimbal_controller* gimbal_controller, Mavlink_message_handler* message_handler)
{
    bool init_success = true;

    // Add callbacks for waypoint handler messages requests
    Mavlink_message_handler::msg_callback_t callback;

    callback.message_id     = MAVLINK_MSG_ID_GIMBAL_COMMAND; //185
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &gimbal_telemetry_parse_msg;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) gimbal_controller;
    init_success &= message_handler->add_msg_callback(&callback);

    return init_success;
}
