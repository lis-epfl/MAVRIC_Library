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
 * \file mavlink_communication.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief This module takes care of sending periodic telemetric messages and
 * handling incoming messages
 *
 ******************************************************************************/


 #include <cstdlib>

#include "communication/mavlink_communication.hpp"
#include "util/print_util.hpp"
#include "hal/common/time_keeper.hpp"

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mavlink_communication::Mavlink_communication(Serial& serial, State& state, File& file_storage, const conf_t& config) :
    mavlink_stream_(serial, config.mavlink_stream_config),
    telemetry_(mavlink_stream_, config.telemetry_config),
    handler_(mavlink_stream_, config.message_handler_config),
    parameters_(file_storage, state, handler_, mavlink_stream_, config.onboard_parameters_config)
{
    bool init_success = true;

    // Add callback to activate / disactivate streams
    Mavlink_message_handler::msg_callback_t callback;

    callback.message_id     = MAVLINK_MSG_ID_REQUEST_DATA_STREAM; // 66
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)       &Periodic_telemetry::toggle_telemetry_stream;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t)  &telemetry_;

    init_success &= handler_.add_msg_callback(&callback);

    if(!init_success)
    {
        print_util_dbg_print("[MAVLINK COMMUNICATION] constructor error\r\n");
    }
}


// Scheduler& Mavlink_communication::scheduler()
// {
//     return scheduler_;
// }

Mavlink_message_handler& Mavlink_communication::handler()
{
    return handler_;
}

Mavlink_message_handler* Mavlink_communication::p_handler()
{
    return &handler_;
}

Mavlink_stream& Mavlink_communication::stream()
{
    return mavlink_stream_;
}

Mavlink_stream* Mavlink_communication::p_stream()
{
    return &mavlink_stream_;
}

Periodic_telemetry& Mavlink_communication::telemetry()
{
    return telemetry_;
}

Periodic_telemetry* Mavlink_communication::p_telemetry()
{
    return &telemetry_;
}

Onboard_parameters& Mavlink_communication::parameters()
{
    return parameters_;
}

Onboard_parameters* Mavlink_communication::p_parameters()
{
    return &parameters_;
}

uint32_t Mavlink_communication::sysid()
{
    return mavlink_stream_.sysid();
}

bool Mavlink_communication::update(void)
{
    // Receive new message
    Mavlink_stream::msg_received_t rec;
    while (mavlink_stream_.receive(&rec))
    {
            handler_.receive(&rec);
    }

    // Send messages
    telemetry_.update();

    // Send one onboard param, if necessary
    parameters_.send_first_scheduled_parameter();

    return true;
}


bool Mavlink_communication::update_task(Mavlink_communication* mavlink_communication)
{
    return mavlink_communication->update();
}
