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
 * \file onboard_parameters.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Onboard parameters
 *
 ******************************************************************************/

#include <cstdlib>

#include "communication/onboard_parameters.hpp"
#include "communication/mavlink_communication.hpp"
extern "C"
{
#include "util/print_util.hpp"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


Onboard_parameters::Onboard_parameters(File& file, const State& state, Mavlink_message_handler& message_handler, const Mavlink_stream& mavlink_stream, const conf_t& config) :
        file_(file),
        state_(state),
        mavlink_stream_(mavlink_stream),
        param_count_(0)
{
    bool init_success = true;

    // Init debug mode
    debug_ = config.debug;

    // allocate memory for command callbacks
    for(max_param_count_ = config.max_param_count; max_param_count_ > 0; max_param_count_--)
    {
        parameters_ = (param_entry_t*)malloc(sizeof(param_entry_t)*max_param_count_);
        if(parameters_ != NULL)
        {
            break;
        }
    }
    if(max_param_count_ < config.max_param_count)
    {
        print_util_dbg_print("[ONBOARD PARAMETERS] constructor: tried to allocate list for ");
        print_util_dbg_print_num(config.max_param_count,10);
        print_util_dbg_print(" parameters; only space for ");
        print_util_dbg_print_num(max_param_count_,10);
        print_util_dbg_print("\r\n");
    }

    // Add callbacks for onboard parameters requests
    Mavlink_message_handler::msg_callback_t callback;

    callback.message_id     = MAVLINK_MSG_ID_PARAM_REQUEST_LIST; // 21
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t) &schedule_all_parameters;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t)this;
    init_success &= message_handler.add_msg_callback(&callback);

    callback.message_id     = MAVLINK_MSG_ID_PARAM_REQUEST_READ; // 20
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t) &send_parameter;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t)this;
    init_success &= message_handler.add_msg_callback(&callback);

    callback.message_id     = MAVLINK_MSG_ID_PARAM_SET; // 23
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t) &receive_parameter;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t)this;
    init_success &= message_handler.add_msg_callback(&callback);

    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_PREFLIGHT_STORAGE; // 245
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL;
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)    &preflight_storage;
    callbackcmd.module_struct = this;
    init_success &= message_handler.add_cmd_callback(&callbackcmd);
}


bool Onboard_parameters::add(uint32_t* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[ONBOARD PARAMETER] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (param_count_ < max_param_count_)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                param_entry_t* new_param = &parameters_[param_count_++];

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


bool Onboard_parameters::add(int32_t* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[ONBOARD PARAMETER] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (param_count_ < max_param_count_)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                param_entry_t* new_param = &parameters_[param_count_++];

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


bool Onboard_parameters::add(float* val, const char* param_name)
{
    bool add_success = true;

    if (val == NULL)
    {
        print_util_dbg_print("[ONBOARD PARAMETER] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (param_count_ < max_param_count_)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                param_entry_t* new_param = &parameters_[param_count_++];

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


bool Onboard_parameters::read_from_storage()
{
    bool success = false;

    float cksum1 = 0.0f;
    float cksum2 = 0.0f;

    // Declare a local array large enough
    uint32_t to_read = file_.length();
    float* values    = (float*)malloc(sizeof(uint8_t) * to_read);

    // Read from file
    file_.seek(0, FILE_SEEK_START);
    file_.read((uint8_t*)values, to_read);

    // Do checksum
    for (uint32_t i = 0; i < (param_count_ + 1); i++)
    {
        cksum1 += values[i];
        cksum2 += cksum1;
    }

    // Copy params
    if ((param_count_ == values[0])
            && (cksum1 == values[param_count_ + 1])
            && (cksum2 == values[param_count_ + 2]))
    {
        print_util_dbg_print("[FLASH] Read successful\r\n");
        for (uint32_t i = 1; i < (param_count_ + 1); i++)
        {
            *(parameters_[i - 1].param) = values[i];
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


bool Onboard_parameters::write_to_storage()
{
    bool success = false;

    float cksum1 = 0.0f;
    float cksum2 = 0.0f;

    // Compute the required space in memory
    // (1 param_count + parameters + 2 checksums) floats
    uint32_t bytes_to_write = 4 * (param_count_ + 3);

    // Declare a local array large enough
    float* values = (float*)malloc(sizeof(uint8_t) * bytes_to_write);

    // Init checksums
    values[0] = param_count_;
    cksum1 += values[0];
    cksum2 += cksum1;

    // Copy parameters to local array and do checksum
    for (uint32_t i = 1; i <= param_count_; i++)
    {
        values[i] = *(parameters_[i - 1].param);

        cksum1 += values[i];
        cksum2 += cksum1;
    }
    values[param_count_ + 1] = cksum1;
    values[param_count_ + 2] = cksum2;

    // Write to file
    file_.seek(0, FILE_SEEK_START);
    success &= file_.write((uint8_t*)values, bytes_to_write);
    success &= file_.flush();
    // Free memory
    free(values);

    return success;
}

bool Onboard_parameters::send_first_scheduled_parameter(void)
{
    bool success = true;

    for (uint8_t i = 0; i < param_count_; i++)
    {
        if (parameters_[i].schedule_for_transmission)
        {
            success = send_one_parameter_now(i);
            return success;
        }
    }

    return success;
}

//------------------------------------------------------------------------------
// PRIVATE STATIC FUNCTIONS IMPLEMENTATION (CALLBACKS)
//------------------------------------------------------------------------------


bool Onboard_parameters::send_all_scheduled_parameters(Onboard_parameters* onboard_parameters)
{
    bool success = true;


    for (uint8_t i = 0; i < onboard_parameters->param_count_; i++)
    {
        if (onboard_parameters->parameters_[i].schedule_for_transmission)
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
        for (uint8_t i = 0; i < onboard_parameters->param_count_; i++)
        {
            onboard_parameters->parameters_[i].schedule_for_transmission = true;
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
            if ((uint32_t)request.param_index <= onboard_parameters->param_count_)
            {
                // Send now
                onboard_parameters->send_one_parameter_now(request.param_index);
            }
        }
        else
        {
            char* key = (char*) request.param_id;
            for (uint16_t i = 0; i < onboard_parameters->param_count_; i++)
            {
                bool match = true;

                // Get pointer to parameter number i
                param_entry_t* param = &onboard_parameters->parameters_[i];

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
            } //end of for (uint16_t i = 0; i < param_set->param_count_; i++)
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
            && (set.target_component == onboard_parameters->mavlink_stream_.compid()))
    {
        char* key = (char*) set.param_id;
        param_entry_t* param;

        if (onboard_parameters->debug_ == true)
        {
            print_util_dbg_print("Setting parameter ");
            print_util_dbg_print(set.param_id);
            print_util_dbg_print(" to ");
            print_util_dbg_putfloat(set.param_value, 2);
            print_util_dbg_print("\r\n");
        }

        for (uint16_t i = 0; i < onboard_parameters->param_count_; i++)
        {
            param = &onboard_parameters->parameters_[i];
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
                switch(param->data_type)
                {
                    case MAVLINK_TYPE_CHAR:
                        *((char*)(param->param)) = set.param_value;
                    break;

                    case MAVLINK_TYPE_UINT8_T:
                        *((uint8_t*)(param->param)) = set.param_value;
                    break;

                    case MAVLINK_TYPE_INT8_T:
                        *((int8_t*)(param->param)) = set.param_value;
                    break;

                    case MAVLINK_TYPE_UINT16_T:
                        *((uint16_t*)(param->param)) = set.param_value;
                    break;

                    case MAVLINK_TYPE_INT16_T:
                        *((int16_t*)(param->param)) = set.param_value;
                    break;

                    case MAVLINK_TYPE_UINT32_T:
                        *((uint32_t*)(param->param)) = set.param_value;
                    break;

                    case MAVLINK_TYPE_INT32_T:
                        *((int32_t*)(param->param)) = set.param_value;
                    break;

                    case MAVLINK_TYPE_FLOAT:
                        *(param->param) = set.param_value;
                    break;

                    case MAVLINK_TYPE_UINT64_T:
                    case MAVLINK_TYPE_INT64_T:
                    case MAVLINK_TYPE_DOUBLE:
                        print_util_dbg_print("Parameter type not supported");
                    break;
                }

                // Send now
                onboard_parameters->send_one_parameter_now(i);

                break;
            }
        }
        if (!match)
        {
            if (onboard_parameters->debug_ == true)
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

    if (!onboard_parameters->state_.is_armed())
    {
        // Onboard parameters storage
        if (msg->param1 == 0)
        {
            // read parameters from flash
            print_util_dbg_print("Reading from flashc...\r\n");
            if (onboard_parameters->read_from_storage())
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
            onboard_parameters->write_to_storage();

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

    // Copy parameter value into a float
    float param_value = 0.0f;
    switch(parameters_[index].data_type)
    {
        case MAVLINK_TYPE_CHAR:
            param_value = *((char*)(parameters_[index].param));
        break;

        case MAVLINK_TYPE_UINT8_T:
            param_value = *((uint8_t*)(parameters_[index].param));
        break;

        case MAVLINK_TYPE_INT8_T:
            param_value = *((int8_t*)(parameters_[index].param));
        break;

        case MAVLINK_TYPE_UINT16_T:
            param_value = *((uint16_t*)(parameters_[index].param));
        break;

        case MAVLINK_TYPE_INT16_T:
            param_value = *((int16_t*)(parameters_[index].param));
        break;

        case MAVLINK_TYPE_UINT32_T:
            param_value = *((uint32_t*)(parameters_[index].param));
        break;

        case MAVLINK_TYPE_INT32_T:
            param_value = *((int32_t*)(parameters_[index].param));
        break;

        case MAVLINK_TYPE_FLOAT:
            param_value = *(parameters_[index].param);
        break;

        case MAVLINK_TYPE_UINT64_T:
            // not supported
        break;

        case MAVLINK_TYPE_INT64_T:
            // not supported
        break;

        case MAVLINK_TYPE_DOUBLE:
            // not supported
        break;
    }

    // Prepare message
    mavlink_msg_param_value_pack(mavlink_stream_.sysid(),
                                 mavlink_stream_.compid(),
                                 &msg,
                                 (char*)parameters_[index].param_name,
                                 param_value,
                                 MAVLINK_TYPE_FLOAT,
                                 param_count_,
                                 index);

    // Schedule for transmission
    parameters_[index].schedule_for_transmission = true;

    // Try to send
    success = mavlink_stream_.send(&msg);

    if (success)
    {
        // If successfully sent, un-schedule
        parameters_[index].schedule_for_transmission = false;
    }

    return success;
}
