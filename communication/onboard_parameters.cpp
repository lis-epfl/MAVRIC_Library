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
 * \file onboard_parameters.c
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Onboard parameters
 *
 ******************************************************************************/


#include "communication/onboard_parameters.hpp"
#include "communication/mavlink_communication.hpp"
extern "C"
{
#include "util/print_util.h"
#include <stdlib.h>
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


Onboard_parameters::Onboard_parameters(Scheduler& scheduler, File& file, const State& state, Mavlink_message_handler& message_handler, const Mavlink_stream& mavlink_stream, const onboard_parameters_conf_t& config) :
        file(file),
        state(state),
        mavlink_stream(mavlink_stream),
        param_count(0)
{
    bool init_success = true;

    // Init debug mode
    debug = config.debug;

    // allocate memory for command callbacks
    for(max_param_count = config.max_param_count; max_param_count > 0; max_param_count--)
    {
        parameters = (onboard_parameters_entry_t*)malloc(sizeof(onboard_parameters_entry_t)*max_param_count);
        if(parameters != NULL)
        {
            break;
        }
    }
    if(max_param_count < config.max_param_count)
    {
        print_util_dbg_print("[ONBOARD PARAMETERS] constructor: tried to allocate list for ");
        print_util_dbg_print_num(config.max_param_count,10);
        print_util_dbg_print(" parameters; only space for ");
        print_util_dbg_print_num(max_param_count,10);
        print_util_dbg_print("\r\n");
    }

    // Add onboard parameter telemetry to the scheduler
    init_success &= scheduler.add_task(100000,
                                       Scheduler_task::Scheduler_task::RUN_REGULAR,
                                       Scheduler_task::Scheduler_task::PERIODIC_ABSOLUTE,
                                       Scheduler_task::Scheduler_task::PRIORITY_HIGHEST,
                                       (Scheduler_task::task_function_t)&send_all_scheduled_parameters,
                                       (Scheduler_task::task_argument_t)this,
                                       MAVLINK_MSG_ID_PARAM_VALUE);

    // Add callbacks for onboard parameters requests
    mavlink_message_handler_msg_callback_t callback;

    callback.message_id     = MAVLINK_MSG_ID_PARAM_REQUEST_LIST; // 21
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &schedule_all_parameters;
    callback.module_struct  = (handling_module_struct_t)this;
    init_success &= message_handler.add_msg_callback(&callback);

    callback.message_id     = MAVLINK_MSG_ID_PARAM_REQUEST_READ; // 20
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &send_parameter;
    callback.module_struct  = (handling_module_struct_t)this;
    init_success &= message_handler.add_msg_callback(&callback);

    callback.message_id     = MAVLINK_MSG_ID_PARAM_SET; // 23
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &receive_parameter;
    callback.module_struct  = (handling_module_struct_t)this;
    init_success &= message_handler.add_msg_callback(&callback);

    // Add callbacks for waypoint handler commands requests
    mavlink_message_handler_cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_PREFLIGHT_STORAGE; // 245
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL;
    callbackcmd.function = (mavlink_cmd_callback_function_t)    &preflight_storage;
    callbackcmd.module_struct = this;
    init_success &= message_handler.add_cmd_callback(&callbackcmd);
}


bool Onboard_parameters::add_parameter_uint32(uint32_t* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[ONBOARD PARAMETER] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (param_count < max_param_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                onboard_parameters_entry_t* new_param = &parameters[param_count++];

                new_param->param                     = (float*) val;
                strcpy(new_param->param_name,       param_name);
                new_param->data_type                 = MAVLINK_TYPE_UINT32_T;
                new_param->param_name_length         = strlen(param_name);
                new_param->schedule_for_transmission = true;

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[ONBOARD PARAMETER] Error: parameter name ");
                print_util_dbg_print(param_name);
                print_util_dbg_print(" is too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[ONBOARD PARAMETER] Error: Cannot add more param\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


bool Onboard_parameters::add_parameter_int32(int32_t* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[ONBOARD PARAMETER] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (param_count < max_param_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                onboard_parameters_entry_t* new_param = &parameters[param_count++];

                new_param->param                     = (float*) val;
                strcpy(new_param->param_name,       param_name);
                new_param->data_type                 = MAVLINK_TYPE_INT32_T;
                new_param->param_name_length         = strlen(param_name);
                new_param->schedule_for_transmission = true;

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[ONBOARD PARAMETER] Error: parameter name ");
                print_util_dbg_print(param_name);
                print_util_dbg_print(" is too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[ONBOARD PARAMETER] Error: Cannot add more param\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


bool Onboard_parameters::add_parameter_float(float* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[ONBOARD PARAMETER] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (param_count < max_param_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                onboard_parameters_entry_t* new_param = &parameters[param_count++];

                new_param->param                     = val;
                strcpy(new_param->param_name,       param_name);
                new_param->data_type                 = MAVLINK_TYPE_FLOAT;
                new_param->schedule_for_transmission = true;
                new_param->param_name_length         = strlen(param_name);

                add_success &= true;
            }
            else
            {
                print_util_dbg_print("[ONBOARD PARAMETER] Error: parameter name ");
                print_util_dbg_print(param_name);
                print_util_dbg_print(" is too long.\r\n");

                add_success &= false;
            }
        }
        else
        {
            print_util_dbg_print("[ONBOARD PARAMETER] Error: Cannot add more param\r\n");

            add_success &= false;
        }
    }

    return add_success;
}


bool Onboard_parameters::read_parameters_from_storage()
{
    bool success = false;

    float cksum1 = 0.0f;
    float cksum2 = 0.0f;

    // Declare a local array large enough
    uint32_t to_read = file.length();
    float* values    = (float*)malloc(sizeof(uint8_t) * to_read);

    // Read from file
    file.seek(0, FILE_SEEK_START);
    file.read((uint8_t*)values, to_read);

    // Do checksum
    for (uint32_t i = 0; i < (param_count + 1); i++)
    {
        cksum1 += values[i];
        cksum2 += cksum1;
    }

    // Copy params
    if ((param_count == values[0])
            && (cksum1 == values[param_count + 1])
            && (cksum2 == values[param_count + 2]))
    {
        print_util_dbg_print("[FLASH] Read successful\r\n");
        for (uint32_t i = 1; i < (param_count + 1); i++)
        {
            *(parameters[i - 1].param) = values[i];
        }
        success = true;
    }
    else
    {
        print_util_dbg_print("[FLASH] [ERROR] Failed to read\r\n");
    }

    // Free memory
    free(values);


    return success;
}


bool Onboard_parameters::write_parameters_to_storage()
{
    bool success = false;

    float cksum1 = 0.0f;
    float cksum2 = 0.0f;

    // Compute the required space in memory
    // (1 param_count + parameters + 2 checksums) floats
    uint32_t bytes_to_write = 4 * (param_count + 3);

    // Declare a local array large enough
    float* values = (float*)malloc(sizeof(uint8_t) * bytes_to_write);

    // Init checksums
    values[0] = param_count;
    cksum1 += values[0];
    cksum2 += cksum1;

    // Copy parameters to local array and do checksum
    for (uint32_t i = 1; i <= param_count; i++)
    {
        values[i] = *(parameters[i - 1].param);

        cksum1 += values[i];
        cksum2 += cksum1;
    }
    values[param_count + 1] = cksum1;
    values[param_count + 2] = cksum2;

    // Write to file
    file.seek(0, FILE_SEEK_START);
    success &= file.write((uint8_t*)values, bytes_to_write);

    // Free memory
    free(values);

    return success;
}


//------------------------------------------------------------------------------
// PRIVATE STATIC FUNCTIONS IMPLEMENTATION (CALLBACKS)
//------------------------------------------------------------------------------


bool Onboard_parameters::send_all_scheduled_parameters(Onboard_parameters* onboard_parameters)
{
    bool success = true;


    for (uint8_t i = 0; i < onboard_parameters->param_count; i++)
    {
        if (onboard_parameters->parameters[i].schedule_for_transmission)
        {
            success = onboard_parameters->send_one_parameter_now(i);
        }
    }//end of for loop

    return success;
}


void Onboard_parameters::schedule_all_parameters(Onboard_parameters* onboard_parameters, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_param_request_list_t packet;
    mavlink_msg_param_request_list_decode(msg, &packet);

    if ((uint8_t)packet.target_system == (uint8_t)sysid)
    {

        // schedule all parameters for transmission
        for (uint8_t i = 0; i < onboard_parameters->param_count; i++)
        {
            onboard_parameters->parameters[i].schedule_for_transmission = true;
        }
    }
}


void Onboard_parameters::send_parameter(Onboard_parameters* onboard_parameters, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_param_request_read_t request;
    mavlink_msg_param_request_read_decode(msg, &request);

    if ((uint8_t)request.target_system == (uint8_t)sysid)
    {

        // Check param_index to determine if the request is made by name (== -1) or by index (!= -1)
        if (request.param_index != -1)
        {
            // Control if the index is in the range of existing parameters and schedule it for transmission
            if ((uint32_t)request.param_index <= onboard_parameters->param_count)
            {
                // Send now
                onboard_parameters->send_one_parameter_now(request.param_index);
            }
        }
        else
        {
            char* key = (char*) request.param_id;
            for (uint16_t i = 0; i < onboard_parameters->param_count; i++)
            {
                bool match = true;

                // Get pointer to parameter number i
                onboard_parameters_entry_t* param = &onboard_parameters->parameters[i];

                for (uint16_t j = 0; j < param->param_name_length; j++)
                {
                    // Compare
                    if ((char)param->param_name[j] != (char)key[j])
                    {
                        match = false;
                    }

                    // End matching if null termination is reached
                    if (((char)param->param_name[j]) == '\0')
                    {
                        // Exit internal (j) for() loop
                        break;
                    }
                }

                // Check if matched
                if (match)
                {
                    // Send now
                    onboard_parameters->send_one_parameter_now(i);

                    // Exit external (i) for() loop
                    break;
                }
            } //end of for (uint16_t i = 0; i < param_set->param_count; i++)
        } //end of else
    } //end of if ((uint8_t)request.target_system == (uint8_t)sysid)
}



void Onboard_parameters::receive_parameter(Onboard_parameters* onboard_parameters, uint32_t sysid, mavlink_message_t* msg)
{
    bool match = true;

    mavlink_param_set_t set;
    mavlink_msg_param_set_decode(msg, &set);

    // Check if this message is for this system and subsystem
    if (((uint8_t)set.target_system       == (uint8_t)sysid)
            && (set.target_component == onboard_parameters->mavlink_stream.compid))
    {
        char* key = (char*) set.param_id;
        onboard_parameters_entry_t* param;

        if (onboard_parameters->debug == true)
        {
            print_util_dbg_print("Setting parameter ");
            print_util_dbg_print(set.param_id);
            print_util_dbg_print(" to ");
            print_util_dbg_putfloat(set.param_value, 2);
            print_util_dbg_print("\r\n");
        }

        for (uint16_t i = 0; i < onboard_parameters->param_count; i++)
        {
            param = &onboard_parameters->parameters[i];
            match = true;

            for (uint16_t j = 0; j < param->param_name_length; j++)
            {
                // Compare
                if ((char)param->param_name[j] != (char)key[j])
                {
                    match = false;
                }

                // End matching if null termination is reached
                if (((char)param->param_name[j]) == '\0')
                {
                    // Exit internal (j) for() loop
                    break;
                }
            }

            // Check if matched
            if (match)
            {
                // Only write if there is actually a difference
                if (*(param->param) != set.param_value && set.param_type == param->data_type)
                {
                    // onboard_parameters_update_parameter(onboard_parameters, i, set.param_value);
                    (*param->param) = set.param_value;
                    print_util_dbg_print("... OK \r\n");
                }

                // Send now
                onboard_parameters->send_one_parameter_now(i);

                break;
            }
        }
        if (!match)
        {
            if (onboard_parameters->debug == true)
            {
                print_util_dbg_print("Set parameter error! Parameter ");
                print_util_dbg_print(set.param_id);
                print_util_dbg_print(" not registred!\r\n");
            }
        }
    }
}


mav_result_t Onboard_parameters::preflight_storage(Onboard_parameters* onboard_parameters, mavlink_command_long_t* msg)
{
    mav_result_t result = MAV_RESULT_DENIED;

    if (!mav_modes_is_armed(onboard_parameters->state.mav_mode))
    {
        // Onboard parameters storage
        if (msg->param1 == 0)
        {
            // read parameters from flash
            print_util_dbg_print("Reading from flashc...\r\n");
            if (onboard_parameters->read_parameters_from_storage())
            {
                result = MAV_RESULT_ACCEPTED;
            }
            else
            {
                result = MAV_RESULT_DENIED;
            }
        }
        else if (msg->param1 == 1)
        {

            // write parameters to flash
            print_util_dbg_print("Writing to flashc\r\n");
            onboard_parameters->write_parameters_to_storage();

            result = MAV_RESULT_ACCEPTED;
        }
    }
    else
    {
        print_util_dbg_print("No action with flashc while motors armed!\r\n");

        result = MAV_RESULT_TEMPORARILY_REJECTED;
    }

    return result;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


bool Onboard_parameters::send_one_parameter_now(uint32_t index)
{
    bool success = true;

    mavlink_message_t msg;
    mavlink_msg_param_value_pack(mavlink_stream.sysid,
                                 mavlink_stream.compid,
                                 &msg,
                                 (char*)parameters[index].param_name,
                                 *(parameters[index].param),
                                 // onboard_parameters_read_parameter(onboard_parameters, index),
                                 parameters[index].data_type,
                                 param_count,
                                 index);

    // Schedule for transmission
    parameters[index].schedule_for_transmission = true;

    // Try to send
    success = mavlink_stream.send(&msg);

    if (success)
    {
        // If successfully sent, un-schedule
        parameters[index].schedule_for_transmission = false;
    }

    return success;
}

