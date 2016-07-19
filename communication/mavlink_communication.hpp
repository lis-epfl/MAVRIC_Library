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


#ifndef MAVLINK_COMMUNICATION_HPP_
#define MAVLINK_COMMUNICATION_HPP_

#include "communication/onboard_parameters.hpp"
#include "communication/mavlink_stream.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "runtime/scheduler.hpp"
#include "hal/common/file.hpp"




class Mavlink_communication
{
public:

    /**
     * \brief       Pointer a module's data structure
     *
     * \details     This is used as an alias to any data structure in the prototype of callback functions
     */
    typedef void* handling_telemetry_module_struct_t;

    /**
     * \brief       Prototype of callback functions for MAVLink messages
     */
    typedef void (*send_msg_function_t)(handling_telemetry_module_struct_t, Mavlink_stream*, mavlink_message_t*);

    /**
     * \brief   Configuration of the module Mavlink Communication
     */
    struct conf_t
    {
        Scheduler::conf_t               scheduler_config;
        Mavlink_stream::conf_t          mavlink_stream_config;          ///<    Configuration for the module MAVLink stream
        Mavlink_message_handler::conf_t message_handler_config;         ///<    Configuration for the module message handler
        Onboard_parameters::conf_t      onboard_parameters_config;      ///<    Configuration for the module onboard parameters

        uint32_t                        max_msg_sending_count;          ///<    Configuration for the sending message handler
    };

    /**
     * \brief   Default configuration
     *
     * \param   sysid       System id (default value = 1)
     *
     * \return  Config structure
     */
    static inline conf_t default_config(uint8_t sysid = 1);

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
    Mavlink_communication(Serial& serial, State& state, File& file_storage, const conf_t& config = default_config());

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
    bool add_msg_send(uint32_t task_id,
                      uint32_t repeat_period,
                      send_msg_function_t function,
                      handling_telemetry_module_struct_t module_structure,
                      Scheduler_task::priority_t priority       = Scheduler_task::PRIORITY_NORMAL,
                      Scheduler_task::timing_mode_t timing_mode = Scheduler_task::PERIODIC_RELATIVE,
                      Scheduler_task::run_mode_t run_mode       = Scheduler_task::RUN_REGULAR);



    /**
     * \brief   Suspending sending of messages
     *
     * \param   delay                   Delay of suspension in microsecond
     */
    void suspend_downstream(uint32_t delay);


    /**
     * \brief   Returns sysid of the underlying mavlink_stream
     *
     *
     * \return  sysid of the underlying mavlink_stream
     */
     uint32_t sysid();

    /*
     * \brief   Returns scheduler
     */
    Scheduler& scheduler();

    /*
     * \brief   Returns message_handler
     */
    Mavlink_message_handler& message_handler();
    Mavlink_message_handler* p_message_handler();

    /*
     * \brief   Returns mavlink_stream
     */
    Mavlink_stream& mavlink_stream();
    Mavlink_stream* p_mavlink_stream();

    /*
     * \brief   Returns onboard_parameters struct
     */
    Onboard_parameters& onboard_parameters();
    Onboard_parameters* p_onboard_parameters();

    /**
     * \brief   Run task scheduler update if the buffer is empty
     *
     * \param   mavlink_communication   Pointer to the MAVLink communication structure
     *
     * \return  Task status return
     */
    static bool update(Mavlink_communication* mavlink_communication);

private:

    /**
     * \brief   MAVLink message handler structure
     */
    struct send_msg_handler_t
    {
        Mavlink_stream* mavlink_stream;                                 ///<    Pointer to the MAVLink stream structure
        send_msg_function_t function;                                   ///<    Pointer to the function to be executed
        handling_telemetry_module_struct_t      module_struct;          ///<    Pointer to module data structure to be given as argument to the function
    };

    Scheduler                   scheduler_;                      ///<    Task set for scheduling of down messages
    Mavlink_stream              mavlink_stream_;                 ///<    Mavlink interface using streams
    Mavlink_message_handler     message_handler_;                ///<    Message handler
    Onboard_parameters          onboard_parameters_;             ///<    Onboard parameters
    uint32_t                    msg_sending_count_;                                     ///<    Number of message callback currently registered
    uint32_t                    max_msg_sending_count_;                                 ///<    Maximum number of callback that can be registered
    send_msg_handler_t*         msg_send_list_;                      ///<    List of message callbacks


    /**
     * \brief   Suspending sending of messages
     *
     * \param   msg_send    The MAVLink message sending handler
     *
     * \return  The result of execution of the task
     */
    static bool send_message(send_msg_handler_t* msg_send);

    /**
     * \brief   Toggle mavlink telemetry stream
     *
     * \param scheduler     scheduler of the MAV
     * \param sysid         MAV sysid
     * \param msg           pointer to the stream you want to toggle
     */
    static void toggle_telemetry_stream(Scheduler* scheduler, uint32_t sysid, mavlink_message_t* msg);


};


Mavlink_communication::conf_t Mavlink_communication::default_config(uint8_t sysid)
{
    conf_t conf                                        = {};

    conf.scheduler_config                              = {};
    conf.scheduler_config.max_task_count               = 30;
    conf.scheduler_config.schedule_strategy            = Scheduler::FIXED_PRIORITY;
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


#endif /* MAVLINK_COMMUNICATION_HPP_ */
