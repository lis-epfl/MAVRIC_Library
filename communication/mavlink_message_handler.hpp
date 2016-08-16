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
 * \file mavlink_message_handler.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief This module handles of all incoming MAVLink message by calling the
 * appropriate functions
 *
 ******************************************************************************/


#ifndef MAVLINK_MESSAGE_HANDLING_HPP_
#define MAVLINK_MESSAGE_HANDLING_HPP_

#include "communication/mavlink_stream.hpp"


#define MAV_SYS_ID_ALL 0
#define MAV_MSG_ENUM_END 255


/**
 * \brief       Enumeration of command MAV result
 * \details     The enumeration MAV_RESULT is defined by MAVLink
 */
typedef enum MAV_RESULT mav_result_t;


/**
 * \brief       Main message handler structure
 *
 * \details     msg_callback_set and cmd_callback_set are implemented as pointer
 *              because their memory will be allocated during initialisation
 */
class Mavlink_message_handler
{
public:

    /**
     * \brief       Pointer a module's data structure
     *
     * \details     This is used as an alias to any data structure in the prototype of callback functions
     */
    typedef void* handling_module_struct_t;

    /**
     * \brief       Prototype of callback functions for MAVLink messages
     */
    typedef void (*msg_callback_func_t)(handling_module_struct_t, uint32_t sysid, mavlink_message_t*);


    /**
     * \brief       Prototype of callback functions for MAVLink commands
     */
    typedef mav_result_t (*cmd_callback_func_t)(handling_module_struct_t, mavlink_command_long_t*);


    /**
     * \brief       Enumeration of MAV components
     * \details     The enumeration MAV_COMPONENT is defined by MAVLink
     */
    typedef enum MAV_COMPONENT mav_component_t;

    /**
     * \brief       Message callback
     */
    typedef struct
    {
        uint8_t                         message_id;                     ///<    The function will be called only for messages with ID message_id
        uint8_t                         sysid_filter;                   ///<    The function will be called only for messages coming from MAVs with ID sysid_filter (0 for all)
        mav_component_t                 compid_filter;                  ///<    The function will be called only for messages coming from component compid_filter (0 for all)
        msg_callback_func_t             function;                       ///<    Pointer to the function to be executed
        handling_module_struct_t        module_struct;                  ///<    Pointer to module data structure to be given as argument to the function
    } msg_callback_t;


    /**
     * \brief       Command callback
     */
    typedef struct
    {
        uint16_t                        command_id;                     ///<    The function will be called only for commands with ID command_id
        uint8_t                         sysid_filter;                   ///<    The function will be called only for commands coming from MAVs with ID sysid_filter (0 for all)
        mav_component_t                 compid_filter;                  ///<    The function will be called only for commands coming from component compid_filter (0 for all)
        mav_component_t                 compid_target;                  ///<    The function will be called only if the commands targets the component compid_target of this system (0 for all)
        cmd_callback_func_t             function;                       ///<    Pointer to the function to be executed
        handling_module_struct_t        module_struct;                  ///<    Pointer to module data structure to be given as argument to the function
    } cmd_callback_t;


    /**
     * \brief   Structure used to hold parameters during initialisation
     */
    struct conf_t
    {
        uint32_t max_msg_callback_count;                                ///<    Maximum number of message callbacks
        uint32_t max_cmd_callback_count;                                ///<    Maximum number of command callbacks
        bool debug;                                                     ///<    Indicates whether debug message are written for every incoming message
    };

    /**
     * \brief                       Constructor
     *
     * \param   config              Config parameters
     * \param   mavlink_stream      mavlink stream
     *
     */
    Mavlink_message_handler(Mavlink_stream& mavlink_stream, const conf_t& config);

    /**
     * \brief                       Registers a new callback for a message
     * \details                     You should call mavlink_message_handler_sort_callback function after having added all callbacks,
     *                              as it is used by mavlink_message_handler_receive to speed up matching
     *
     * \param   msg_callback        Pointer to new message callback (this structure
     *                              is copied internally, so it does not need to be
     *                              kept in memory after the function is called)
     *
     * \return  True if the message callback was correctly added, false otherwise
     */
    bool add_msg_callback(msg_callback_t* msg_callback);

    /**
     * \brief                       Registers a new callback for a command
     * \details                     You should call mavlink_message_handler_sort_callback function after having added all callbacks,
     *                              as it is used by mavlink_message_handler_receive to speed up matching
     *
     * \param   cmd_callback        Pointer to new command callback (this structure
     *                              is copied internally, so it does not need to be
     *                              kept in memory after the function is called)
     *
     * \return  True if the command callback was correctly added, false otherwise
     */
    bool add_cmd_callback(cmd_callback_t* cmd_callback);

    /**
     * \brief       Main update function, handles the incoming message according to
     *              the registered message and command callbacks
     *
     * \param rec   Pointer to the MAVLink receive message structure
     */
    void receive(Mavlink_stream::msg_received_t* rec);

    /**
     * \brief           Dummy message callback for debug purpose
     * \details         Prints the fields of the incoming message to the debug console
     *
     * \param   msg     Pointer to incoming message
     */
    static void msg_default_dbg(mavlink_message_t* msg);

    /**
     * \brief           Dummy command callback for debug purpose
     * \details         Prints the fields of the incoming command to the debug console
     *
     * \param   cmd     Pointer to incoming command
     */
    static void cmd_default_dbg(mavlink_command_long_t* cmd);


private:
    bool debug_;                                                     ///<    Indicates whether debug message are written for every incoming message
    Mavlink_stream& mavlink_stream_;
    uint32_t msg_callback_count_;                                    ///<    Number of message callback currently registered
    uint32_t msg_callback_count_max_;                                ///<    Maximum number of message callback that can be registered
    uint32_t cmd_callback_count_;                                    ///<    Number of command callback currently registered
    uint32_t cmd_callback_count_max_;                                ///<    Maximum number of command callback that can be registered
    msg_callback_t* msg_callback_list;      ///<    List of command callbacks
    cmd_callback_t* cmd_callback_list;      ///<    List of message callbacks

    /**
    * \brief                Sort the latest added message callback
    *
    * \details              This should be done each time a callback is added to speed up the matching
    *                   while receiving a cmd message.
    */
    void sort_latest_msg_callback();

    /**
    * \brief                Sort the latest added command callback
    *
    * \details              This should be done each time a callback is added to speed up the matching
    *                       while receiving a cmd message.
    *
    */
    void sort_latest_cmd_callback();

    /**
     * \brief                   Checks whether a message matches with a registered callback
     *
     * \param   msg_callback    Pointer to a registered message callback
     * \param   msg             Incoming message
     *
     * \return                  Boolean (true if the message matches, false if not)
     */
    bool match_msg(msg_callback_t* msg_callback, mavlink_message_t* msg);

    /**
     * \brief                   Checks whether a command matches with a registered callback
     *
     * \param   msg_callback    Pointer to a registered command callback
     * \param   msg             Incoming message containing the command
     * \param   cmd             Incoming command encoded in the message
     *
     * \return                  Boolean (true if the message matches, false if not)
     */
    bool match_cmd(cmd_callback_t* cmd_callback, mavlink_message_t* msg, mavlink_command_long_t* cmd);

};

#endif /* MAVLINK_MESSAGE_HANDLING_H */
