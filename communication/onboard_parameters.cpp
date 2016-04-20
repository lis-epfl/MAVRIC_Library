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
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Sends immediately one parameter
 *
 * \param   onboard_parameters  Pointer to onbard_parameter structure
 * \param   index               Index of the parameter to send
 *
 * \return  Success
 */
static bool onboard_parameters_send_one_parameter_now(onboard_parameters_t* onboard_parameters, uint32_t index);


/**
 * \brief   Sends all parameters that have been scheduled via MAVlink
 *
 * \param   onboard_parameters      The pointer to the onboard parameter structure
 *
 * \return  Returns the result of the task, currently only TASK_RUN_SUCCESS
 */
static bool onboard_parameters_send_all_scheduled_parameters(onboard_parameters_t* onboard_parameters);


/**
 * \brief   Marks all parameters to be scheduled for transmission
 *
 * \param   onboard_parameters      Pointer to module structure
 * \param   sysid                   The system ID
 * \param   msg                     Incoming MAVLink message
 */
static void onboard_parameters_schedule_all_parameters(onboard_parameters_t* onboard_parameters, uint32_t sysid, mavlink_message_t* msg);


/**
 * \brief   Callback to a MAVlink parameter request
 *
 * \param   onboard_parameters      Pointer to module structure
 * \param   sysid                   The system ID
 * \param   msg                     Incoming MAVLink message
 */
static void onboard_parameters_send_parameter(onboard_parameters_t* onboard_parameters, uint32_t sysid, mavlink_message_t* msg);


/**
 * \brief   Callback to a MAVlink parameter set
 *
 * \param   onboard_parameters      Pointer to module structure
 * \param   sysid                   The system ID
 * \param   msg                     Incoming MAVLink message
 */
static void onboard_parameters_receive_parameter(onboard_parameters_t* onboard_parameters, uint32_t sysid, mavlink_message_t* msg);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


static bool onboard_parameters_send_one_parameter_now(onboard_parameters_t* onboard_parameters, uint32_t index)
{
    bool success = true;
    onboard_parameters_set_t* param_set = onboard_parameters->param_set;

    mavlink_message_t msg;
    float param_value = *(param_set->parameters[index].param);
    mavlink_msg_param_value_pack(onboard_parameters->mavlink_stream->sysid,
                                 onboard_parameters->mavlink_stream->compid,
                                 &msg,
                                 (char*)param_set->parameters[index].param_name,
                                 param_value,
                                 MAVLINK_TYPE_FLOAT,
                                 param_set->param_count,
                                 index);

    // Schedule for transmission
    param_set->parameters[index].schedule_for_transmission = true;

    // Try to send
    success = mavlink_stream_send(onboard_parameters->mavlink_stream, &msg);

    if (success)
    {
        // If successfully sent, un-schedule
        param_set->parameters[index].schedule_for_transmission = false;
    }

    return success;
}


static bool onboard_parameters_send_all_scheduled_parameters(onboard_parameters_t* onboard_parameters)
{
    bool success = true;
    onboard_parameters_set_t* param_set = onboard_parameters->param_set;

    for (uint8_t i = 0; i < param_set->param_count; i++)
    {
        if (param_set->parameters[i].schedule_for_transmission)
        {
            success = onboard_parameters_send_one_parameter_now(onboard_parameters, i);
        }
    }//end of for loop

    return success;
}


static void onboard_parameters_schedule_all_parameters(onboard_parameters_t* onboard_parameters, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_param_request_list_t packet;
    mavlink_msg_param_request_list_decode(msg, &packet);

    if ((uint8_t)packet.target_system == (uint8_t)sysid)
    {
        onboard_parameters_set_t* param_set = onboard_parameters->param_set;

        // schedule all parameters for transmission
        for (uint8_t i = 0; i < param_set->param_count; i++)
        {
            param_set->parameters[i].schedule_for_transmission = true;
        }
    }
}


static void onboard_parameters_send_parameter(onboard_parameters_t* onboard_parameters, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_param_request_read_t request;
    mavlink_msg_param_request_read_decode(msg, &request);

    if ((uint8_t)request.target_system == (uint8_t)sysid)
    {
        onboard_parameters_set_t* param_set = onboard_parameters->param_set;

        // Check param_index to determine if the request is made by name (== -1) or by index (!= -1)
        if (request.param_index != -1)
        {
            // Control if the index is in the range of existing parameters and schedule it for transmission
            if ((uint32_t)request.param_index <= param_set->param_count)
            {
                // Send now
                onboard_parameters_send_one_parameter_now(onboard_parameters, request.param_index);
            }
        }
        else
        {
            char* key = (char*) request.param_id;
            for (uint16_t i = 0; i < param_set->param_count; i++)
            {
                bool match = true;

                // Get pointer to parameter number i
                onboard_parameters_entry_t* param = &param_set->parameters[i];

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
                    onboard_parameters_send_one_parameter_now(onboard_parameters, i);

                    // Exit external (i) for() loop
                    break;
                }
            } //end of for (uint16_t i = 0; i < param_set->param_count; i++)
        } //end of else
    } //end of if ((uint8_t)request.target_system == (uint8_t)sysid)
}



static void onboard_parameters_receive_parameter(onboard_parameters_t* onboard_parameters, uint32_t sysid, mavlink_message_t* msg)
{
    bool match = true;

    mavlink_param_set_t set;
    mavlink_msg_param_set_decode(msg, &set);

    // Check if this message is for this system and subsystem
    if (((uint8_t)set.target_system       == (uint8_t)sysid)
            && (set.target_component == onboard_parameters->mavlink_stream->compid))
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

        for (uint16_t i = 0; i < onboard_parameters->param_set->param_count; i++)
        {
            param = &onboard_parameters->param_set->parameters[i];
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
                if (*(param->param) != set.param_value)
                {
                    (*param->param) = set.param_value;
                    print_util_dbg_print("... OK \r\n");
                }

                // Send now
                onboard_parameters_send_one_parameter_now(onboard_parameters, i);

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

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool onboard_parameters_init(onboard_parameters_t* onboard_parameters, const onboard_parameters_conf_t* config, scheduler_t* scheduler, File* file, const State* state, mavlink_message_handler_t* message_handler, const mavlink_stream_t* mavlink_stream)
{
    bool init_success = true;

    // Init dependencies
    onboard_parameters->mavlink_stream = mavlink_stream;

    // Init debug mode
    onboard_parameters->debug = config->debug;

    onboard_parameters->file  = file;
    onboard_parameters->state = state;

    // Allocate memory for the onboard parameters
    onboard_parameters->param_set = (onboard_parameters_set_t*)malloc(sizeof(onboard_parameters_set_t) + sizeof(onboard_parameters_entry_t[config->max_param_count]));

    if (onboard_parameters->param_set != NULL)
    {
        onboard_parameters->param_set->max_param_count = config->max_param_count;
        onboard_parameters->param_set->param_count = 0;

        init_success &= true;
    }
    else
    {
        print_util_dbg_print("[ONBOARD PARAMETERS] ERROR ! Bad memory allocation\r\n");
        onboard_parameters->param_set->max_param_count = 0;
        onboard_parameters->param_set->param_count = 0;

        init_success &= false;
    }

    // Add onboard parameter telemetry to the scheduler
    init_success &= scheduler_add_task(scheduler,
                                       100000,
                                       RUN_REGULAR,
                                       PERIODIC_ABSOLUTE,
                                       PRIORITY_HIGHEST,
                                       (task_function_t)&onboard_parameters_send_all_scheduled_parameters,
                                       (task_argument_t)onboard_parameters,
                                       MAVLINK_MSG_ID_PARAM_VALUE);

    // Add callbacks for onboard parameters requests
    mavlink_message_handler_msg_callback_t callback;

    callback.message_id     = MAVLINK_MSG_ID_PARAM_REQUEST_LIST; // 21
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &onboard_parameters_schedule_all_parameters;
    callback.module_struct  = (handling_module_struct_t)        onboard_parameters;
    init_success &= mavlink_message_handler_add_msg_callback(message_handler, &callback);

    callback.message_id     = MAVLINK_MSG_ID_PARAM_REQUEST_READ; // 20
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &onboard_parameters_send_parameter;
    callback.module_struct  = (handling_module_struct_t)        onboard_parameters;
    init_success &= mavlink_message_handler_add_msg_callback(message_handler, &callback);

    callback.message_id     = MAVLINK_MSG_ID_PARAM_SET; // 23
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &onboard_parameters_receive_parameter;
    callback.module_struct  = (handling_module_struct_t)        onboard_parameters;
    init_success &= mavlink_message_handler_add_msg_callback(message_handler, &callback);

    // Add callbacks for waypoint handler commands requests
    mavlink_message_handler_cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_PREFLIGHT_STORAGE; // 245
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL;
    callbackcmd.function = (mavlink_cmd_callback_function_t)    &onboard_parameters_preflight_storage;
    callbackcmd.module_struct =                                 onboard_parameters;
    init_success &= mavlink_message_handler_add_cmd_callback(message_handler, &callbackcmd);

    return init_success;
}


bool onboard_parameters_add_parameter_uint32(onboard_parameters_t* onboard_parameters, uint32_t* val, const char* param_name)
{
    bool add_success = true;

    onboard_parameters_set_t* param_set = onboard_parameters->param_set;

    if ((val == NULL) || (param_set == NULL))
    {
        print_util_dbg_print("[ONBOARD PARAMETER] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (param_set->param_count < param_set->max_param_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                onboard_parameters_entry_t* new_param = &param_set->parameters[param_set->param_count];

                new_param->param                     = (float*) val;
                strcpy(new_param->param_name,       param_name);
                new_param->data_type                 = MAVLINK_TYPE_UINT32_T;
                new_param->param_name_length         = strlen(param_name);
                new_param->schedule_for_transmission = true;


                param_set->param_count += 1;

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


bool onboard_parameters_add_parameter_int32(onboard_parameters_t* onboard_parameters, int32_t* val, const char* param_name)
{
    bool add_success = true;

    onboard_parameters_set_t* param_set = onboard_parameters->param_set;

    if ((val == NULL) || (param_set == NULL))
    {
        print_util_dbg_print("[ONBOARD PARAMETER] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (param_set->param_count < param_set->max_param_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                onboard_parameters_entry_t* new_param = &param_set->parameters[param_set->param_count];

                new_param->param                     = (float*) val;
                strcpy(new_param->param_name,       param_name);
                new_param->data_type                 = MAVLINK_TYPE_INT32_T;
                new_param->param_name_length         = strlen(param_name);
                new_param->schedule_for_transmission = true;

                param_set->param_count += 1;

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


bool onboard_parameters_add_parameter_float(onboard_parameters_t* onboard_parameters, float* val, const char* param_name)
{
    bool add_success = true;

    onboard_parameters_set_t* param_set = onboard_parameters->param_set;

    if ((val == NULL) || (param_set == NULL))
    {
        print_util_dbg_print("[ONBOARD PARAMETER] Error: Null pointer!");

        add_success &= false;
    }
    else
    {
        if (param_set->param_count < param_set->max_param_count)
        {
            if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
            {
                onboard_parameters_entry_t* new_param = &param_set->parameters[param_set->param_count];

                new_param->param                     = val;
                strcpy(new_param->param_name,       param_name);
                new_param->data_type                 = MAVLINK_TYPE_FLOAT;
                new_param->schedule_for_transmission = true;
                new_param->param_name_length         = strlen(param_name);


                param_set->param_count += 1;

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


mav_result_t onboard_parameters_preflight_storage(onboard_parameters_t* onboard_parameters, mavlink_command_long_t* msg)
{
    mav_result_t result = MAV_RESULT_DENIED;

    if (!mav_modes_is_armed(onboard_parameters->state->mav_mode))
    {
        // Onboard parameters storage
        if (msg->param1 == 0)
        {
            // read parameters from flash
            print_util_dbg_print("Reading from flashc...\r\n");
            if (onboard_parameters_read_parameters_from_storage(onboard_parameters))
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
            onboard_parameters_write_parameters_to_storage(onboard_parameters);

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


bool onboard_parameters_read_parameters_from_storage(onboard_parameters_t* onboard_parameters)
{
    bool success = false;

    float cksum1 = 0.0f;
    float cksum2 = 0.0f;

    // Get pointer to set of parameters
    onboard_parameters_set_t* param_set = onboard_parameters->param_set;

    // Declare a local array large enough
    uint32_t to_read = onboard_parameters->file->length();
    float* values    = (float*)malloc(sizeof(uint8_t) * to_read);

    // Read from file
    onboard_parameters->file->seek(0, FILE_SEEK_START);
    onboard_parameters->file->read((uint8_t*)values, to_read);

    // Do checksum
    for (uint32_t i = 0; i < (param_set->param_count + 1); i++)
    {
        cksum1 += values[i];
        cksum2 += cksum1;
    }

    // Copy params
    if ((param_set->param_count == values[0])
            && (cksum1 == values[param_set->param_count + 1])
            && (cksum2 == values[param_set->param_count + 2]))
    {
        print_util_dbg_print("[FLASH] Read successful\r\n");
        for (uint32_t i = 1; i < (param_set->param_count + 1); i++)
        {
            *(param_set->parameters[i - 1].param) = values[i];
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


bool onboard_parameters_write_parameters_to_storage(onboard_parameters_t* onboard_parameters)
{
    bool success = false;

    float cksum1 = 0.0f;
    float cksum2 = 0.0f;

    // Get pointer to set of parameters
    onboard_parameters_set_t* param_set = onboard_parameters->param_set;

    // Compute the required space in memory
    // (1 param_count + parameters + 2 checksums) floats
    uint32_t bytes_to_write = 4 * (param_set->param_count + 3);

    // Declare a local array large enough
    float* values = (float*)malloc(sizeof(uint8_t) * bytes_to_write);

    // Init checksums
    values[0] = param_set->param_count;
    cksum1 += values[0];
    cksum2 += cksum1;

    // Copy parameters to local array and do checksum
    for (uint32_t i = 1; i <= param_set->param_count; i++)
    {
        values[i] = *(param_set->parameters[i - 1].param);

        cksum1 += values[i];
        cksum2 += cksum1;
    }
    values[param_set->param_count + 1] = cksum1;
    values[param_set->param_count + 2] = cksum2;

    // Write to file
    onboard_parameters->file->seek(0, FILE_SEEK_START);
    success &= onboard_parameters->file->write((uint8_t*)values, bytes_to_write);

    // Free memory
    free(values);

    return success;
}
