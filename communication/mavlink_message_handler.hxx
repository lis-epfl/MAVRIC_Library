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
 * \file mavlink_message_handler.hxx
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief This module handles of all incoming MAVLink message by calling the
 * appropriate functions
 *
 ******************************************************************************/

template<typename T>
bool Mavlink_message_handler::add_msg_callback(uint8_t                          message_id,
                                               uint8_t                          sysid_filter,
                                               mav_component_t                  compid_filter,
                                               typename msg_function<T>::type_t function,
                                               T*                               module_struct)
{
    bool add_callback_success = true;

    if (function == NULL || module_struct == NULL )
    {
        print_util_dbg_print("[MESSAGE HANDLER] Error: null pointer.\r\n");

        add_callback_success &= false;
    }
    else
    {
        if (msg_callback_count_ <  msg_callback_max_count())
        {
            msg_callback_t* new_callback = &msg_callback_list()[msg_callback_count_];

            new_callback->message_id    = message_id;
            new_callback->sysid_filter  = sysid_filter;
            new_callback->compid_filter = compid_filter;
            new_callback->function      = reinterpret_cast<msg_function<void>::type_t>(function);      // we do dangerous casting here, but it is safe because
            new_callback->module_struct = reinterpret_cast<void*>(module_struct);                      // the types of telemetry_function and telemetry_argument are compatible

            sort_latest_msg_callback();

            msg_callback_count_ += 1;

            add_callback_success &= true;
        }
        else
        {
            print_util_dbg_print("[MESSAGE HANDLER] Error: Cannot add more msg callback\r\n");

            add_callback_success &= false;
        }
    }

    return add_callback_success;
}


template<typename T>
bool Mavlink_message_handler::add_cmd_callback( uint16_t                            command_id,
                                                uint8_t                             sysid_filter,
                                                mav_component_t                     compid_filter,
                                                mav_component_t                     compid_target,
                                                typename cmd_function<T>::type_t    function,
                                                T*                                  module_struct)
{
    bool add_callback_success = true;

    if (function == NULL || module_struct == NULL )
    {
        print_util_dbg_print("[MESSAGE HANDLER] Error: null pointer.\r\n");

        add_callback_success &= false;
    }
    else
    {
        if (cmd_callback_count_ <  cmd_callback_max_count())
        {
            cmd_callback_t* new_callback = &cmd_callback_list()[cmd_callback_count_];

            new_callback->command_id    = command_id;
            new_callback->sysid_filter  = sysid_filter;
            new_callback->compid_filter = compid_filter;
            new_callback->compid_target = compid_target;
            new_callback->function      = reinterpret_cast<cmd_function<void>::type_t>(function);      // we do dangerous casting here, but it is safe because
            new_callback->module_struct = reinterpret_cast<void*>(module_struct);                      // the types of telemetry_function and telemetry_argument are compatible

            sort_latest_cmd_callback();

            cmd_callback_count_ += 1;

            add_callback_success &= true;
        }
        else
        {
            print_util_dbg_print("[MESSAGE HANDLER] Error: Cannot add more msg callback\r\n");

            add_callback_success &= false;
        }
    }

    return add_callback_success;
}
