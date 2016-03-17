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
 * \file mavlink_communication.h
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief This module takes care of sending periodic telemetric messages and
 * handling incoming messages
 *
 ******************************************************************************/


#ifndef MAVLINK_COMMUNICATION_H_
#define MAVLINK_COMMUNICATION_H_

#include "communication/onboard_parameters.hpp"
#include "communication/mavlink_stream.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "runtime/scheduler.hpp"
#include "hal/common/file.hpp"


/**
 * \brief       Pointer a module's data structure
 *
 * \details     This is used as an alias to any data structure in the prototype of callback functions
 */
typedef void* handling_telemetry_module_struct_t;


/**
 * \brief       Prototype of callback functions for MAVLink messages
 */
typedef void (*mavlink_send_msg_function_t)(handling_telemetry_module_struct_t, Mavlink_stream*, mavlink_message_t*);


/**
 * \brief   MAVLink message handler structure
 */
typedef struct
{
    Mavlink_stream* mavlink_stream;                               ///<    Pointer to the MAVLink stream structure
    mavlink_send_msg_function_t function;                           ///<    Pointer to the function to be executed
    handling_telemetry_module_struct_t      module_struct;          ///<    Pointer to module data structure to be given as argument to the function
} mavlink_send_msg_handler_t;


/**
 * \brief   Configuration of the module Mavlink Communication
 */
typedef struct
{
    scheduler_conf_t                scheduler_config;
    mavlink_stream_conf_t           mavlink_stream_config;          ///<    Configuration for the module MAVLink stream
    mavlink_message_handler_conf_t  message_handler_config;         ///<    Configuration for the module message handler
    onboard_parameters_conf_t       onboard_parameters_config;      ///<    Configuration for the module onboard parameters

    uint32_t                        max_msg_sending_count;          ///<    Configuration for the sending message handler
} mavlink_communication_conf_t;


/**
 * \brief   Default configuration
 *
 * \param   sysid       System id (default value = 1)
 *
 * \return  Config structure
 */
static inline mavlink_communication_conf_t mavlink_communication_default_config(uint8_t sysid = 1);


class Mavlink_communication
{
public:
    /**
     * \brief   Initialisation of the module MAVLink communication
     *
     * \param   config                  Configuration
     * \param   serial                  Input/Output stream
     * \param   state                   MAV state
     * \param   file_storage            File used to store parameters between files
     *
     * \return  True if the init succeed, false otherwise
     */
    Mavlink_communication(Serial& serial, State& state, File& file_storage, const mavlink_communication_conf_t& config = mavlink_communication_default_config());

    /**
     * \brief   Adding new message to the MAVLink scheduler
     *
     * \param   repeat_period           Repeat period (us)
     * \param   run_mode                Run mode
     * \param   timing_mode             Timing mode
     * \param   priority                Priority
     * \param   function                Function pointer to be called
     * \param   module_structure        Argument to be passed to the function
     * \param   task_id                 Unique task identifier
     *
     * \return  True if the message was correctly added, false otherwise
     */
    bool add_msg_send(uint32_t repeat_period, task_run_mode_t run_mode, task_timing_mode_t timing_mode, task_priority_t priority, mavlink_send_msg_function_t function, handling_telemetry_module_struct_t module_structure, uint32_t task_id);



    /**
     * \brief   Suspending sending of messages
     *
     * \param   delay                   Delay of suspension in microsecond
     */
    void suspend_downstream(uint32_t delay);


    /**
     * \brief   Suspending sending of messages
     *
     * \param   msg_send    The MAVLink message sending handler
     *
     * \return  The result of execution of the task
     */
    static bool send_message(mavlink_send_msg_handler_t* msg_send);


    /**
     * \brief   Returns sysid of the underlying mavlink_stream
     *
     *
     * \return  sysid of the underlying mavlink_stream
     */
     uint32_t get_sysid();

    /**
     * \brief   Returns address of sysid of the underlying mavlink_stream
     *
     *
     * \return  address of sysid of the underlying mavlink_stream
     */
     uint32_t* get_sysid_ptr();


    friend bool mavlink_communication_update(Mavlink_communication* mavlink_communication);
    
    Scheduler& get_scheduler();
    Mavlink_message_handler& get_message_handler();
    Mavlink_stream& get_mavlink_stream();
    Onboard_parameters& get_onboard_parameters();
    
    
private:
    Scheduler                       scheduler;                      ///<    Task set for scheduling of down messages
    Mavlink_stream                  mavlink_stream;                 ///<    Mavlink interface using streams
    Mavlink_message_handler         message_handler;                ///<    Message handler
    Onboard_parameters              onboard_parameters;             ///<    Onboard parameters
    uint32_t msg_sending_count;                                     ///<    Number of message callback currently registered
    uint32_t max_msg_sending_count;                                 ///<    Maximum number of callback that can be registered
    mavlink_send_msg_handler_t* msg_send_list;                      ///<    List of message callbacks

    bool configure_communication(const mavlink_communication_conf_t& config);
};

/**
 * \brief   Run task scheduler update if the buffer is empty
 *
 * \param   mavlink_communication   Pointer to the MAVLink communication structure
 *
 * \return  Task status return
 */
bool mavlink_communication_update(Mavlink_communication* mavlink_communication);




static inline mavlink_communication_conf_t mavlink_communication_default_config(uint8_t sysid)
{
    mavlink_communication_conf_t conf                  = {};

    conf.scheduler_config                              = {};
    conf.scheduler_config.max_task_count               = 30;
    conf.scheduler_config.schedule_strategy            = FIXED_PRIORITY;
    conf.scheduler_config.debug                        = false;
    conf.mavlink_stream_config                         = {};
    conf.mavlink_stream_config.sysid                   = sysid;
    conf.mavlink_stream_config.compid                  = 50;
    conf.mavlink_stream_config.debug                   = false,
                               conf.message_handler_config                        = {};
    conf.message_handler_config.max_msg_callback_count = 20;
    conf.message_handler_config.max_cmd_callback_count = 20;
    conf.message_handler_config.debug                  = false;
    conf.onboard_parameters_config                     = {};
    conf.onboard_parameters_config.max_param_count     = MAX_ONBOARD_PARAM_COUNT;
    conf.onboard_parameters_config.debug               = false  ;
    conf.max_msg_sending_count                         = 22;

    return conf;
};


#endif /* MAVLINK_COMMUNICATION_H_ */