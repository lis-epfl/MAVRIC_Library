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
 * \file onboard_parameters.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Onboard parameters
 *
 ******************************************************************************/


#ifndef ONBOARD_PARAMETERS_HPP_
#define ONBOARD_PARAMETERS_HPP_

#include <cstdbool>

#include "communication/mavlink_stream.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "communication/state.hpp"
#include "hal/common/file.hpp"


/**
 * \brief       Onboard parameters base class
 *
 * \details     This class is abstract and does not contains the task list,
 *              use the child class Onboard_parameters_T
 */
class Onboard_parameters
{
public:

    /**
     * \brief   Configuration for the module onboard parameters
     */
    struct conf_t
    {
        bool debug;                                                 ///< Indicates if debug messages should be printed for each param change
    };


    /**
     * \brief   Default configuration
     *
     * \return  Config
     */
    static inline conf_t default_config(void);


    /**
     * \brief   Constructor: Initialisation of the Parameter_Set structure by setting the number of onboard parameter to 0
     *
     * \param   config                  Configuration
     * \param   scheduler               Pointer to MAVLink scheduler
     * \param   file                    Pointer to file storage
     * \param   state                   Pointer to the state structure
     * \param   message_handler         Pointer to MAVLink message handler
     *
     * \return  True if the init succeed, false otherwise
     */
    Onboard_parameters(File& file, const State& state, Mavlink_message_handler& message_handler, const Mavlink_stream& mavlink_stream, const conf_t& config);

    /**
     * \brief   Register parameter in the internal parameter list that gets published to MAVlink
     *
     * \param   val                     Unsigned 32 - bits integer parameter value
     * \param   param_name              Name of the parameter
     *
     * \return  True if the parameter was added, false otherwise
     */
    bool add(uint32_t* val, const char* param_name);

    /**
     * \brief   Register parameter in the internal parameter list that gets published to MAVlink
     *
     * \param   val                     Signed 32 - bits integer parameter value
     * \param   param_name              Name of the parameter
     *
     * \return  True if the parameter was added, false otherwise
     */
    bool add(int32_t* val, const char* param_name);

    /**
     * \brief   Registers parameter in the internal parameter list that gets published to MAVlink
     *
     * \param   val                     Floating point parameter value
     * \param   param_name              Name of the parameter
     *
     * \return  True if the parameter was added, false otherwise
     */
    bool add(float* val, const char* param_name);

    /**
     * \brief   Read onboard parameters from the file storage
     *
     * \return  The result of the read procedure
     */
     bool read_from_storage();

    /**
     * \brief   Write onboard parameters to the file storage
     *
     * \return  The result of the write procedure
     */
    bool write_to_storage();

    /**
     * \brief   Searches through the list of parameters, and send only the first scheduled parameter
     *
     * \return  success
     */
    bool send_first_scheduled_parameter(void);


protected:

    /**
     * \brief   Structure of onboard parameter.
     */
    struct param_entry_t
    {
        float* param;                                               ///< Pointer to the parameter value
        char param_name[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];  ///< Parameter name composed of 16 characters
        mavlink_message_type_t data_type;                           ///< Parameter type
        uint8_t param_name_length;                                  ///< Length of the parameter name
        uint8_t param_id;                                           ///< Parameter ID
        bool  schedule_for_transmission;                            ///< Boolean to activate the transmission of the parameter
    };


    /**
     * \brief       Get maximum number of parameters
     *
     * \details     To be overriden by child class
     *
     * \return      Maximum number of parameters
     */
    virtual uint32_t max_count(void) = 0;


    /**
     * \brief       Get pointer to the list of parameters
     *
     * \details     Abstract method to be implemented in child classes
     *
     * \return      task list
     */
    virtual param_entry_t* parameters(void) = 0;


private:

    bool debug_;                                             ///< Indicates if debug messages should be printed for each param change
    File& file_;                                             ///< File storage to keep parameters between flights
    const State& state_;                                     ///< Pointer to the state structure
    const Mavlink_stream& mavlink_stream_;                   ///< Pointer to mavlink_stream
    uint32_t param_count_;                                   ///< Number of onboard parameter effectively in the array


    /**
     * \brief   Sends immediately one parameter
     *
     * \param   index               Index of the parameter to send
     *
     * \return  Success
     */
     bool send_one_parameter_now(uint32_t index);


    /******************************
     * static callback functions  *
     ******************************/
    /**
     * \brief   Sends all parameters that have been scheduled via MAVlink
     *
     * \param   onboard_parameters      The pointer to the onboard parameter structure
     *
     * \return  Returns the result of the task, currently only TASK_RUN_SUCCESS
     */
    static bool send_all_scheduled_parameters(Onboard_parameters* onboard_parameters);

    /**
     * \brief   Marks all parameters to be scheduled for transmission
     *
     * \param   onboard_parameters      Pointer to module structure
     * \param   sysid                   The system ID
     * \param   msg                     Incoming MAVLink message
     */
    static void schedule_all_parameters(Onboard_parameters* onboard_parameters, uint32_t sysid, mavlink_message_t* msg);

    /**
     * \brief   Callback to a MAVlink parameter request
     *
     * \param   onboard_parameters      Pointer to module structure
     * \param   sysid                   The system ID
     * \param   msg                     Incoming MAVLink message
     */
    static void send_parameter(Onboard_parameters* onboard_parameters, uint32_t sysid, mavlink_message_t* msg);

    /**
     * \brief   Callback to a MAVlink parameter set
     *
     * \param   onboard_parameters      Pointer to module structure
     * \param   sysid                   The system ID
     * \param   msg                     Incoming MAVLink message
     */
    static void receive_parameter(Onboard_parameters* onboard_parameters, uint32_t sysid, mavlink_message_t* msg);

    /**
     * \brief   Read/Write from/to flash depending on the parameters of the MAVLink command message
     *
     * \param   onboard_parameters      Pointer to module structure
     * \param   msg                     Incoming MAVLink message
     *
     * \return  The MAV_RESULT of the command
     */
    static mav_result_t preflight_storage(Onboard_parameters* onboard_parameters, mavlink_command_long_t* msg);

};

/**
 * \brief       Onboard parameters
 *
 * \tparam N    Maximum number of parameters
 */
template<uint32_t N>
class Onboard_parameters_T: public Onboard_parameters
{
public:
    /**
     * \brief   Constructor
     *
     * \param   config                  Configuration
     * \param   scheduler               Pointer to MAVLink scheduler
     * \param   file                    Pointer to file storage
     * \param   state                   Pointer to the state structure
     * \param   message_handler         Pointer to MAVLink message handler
     *
     * \return  True if the init succeed, false otherwise
     */
    Onboard_parameters_T(File& file, const State& state, Mavlink_message_handler& message_handler, const Mavlink_stream& mavlink_stream, const conf_t& config):
        Onboard_parameters(file, state, message_handler, mavlink_stream, config)
    {}


protected:

    /**
     * \brief       Get maximum number of parameters
     *
     * \return      Maximum number of parameters
     */
    uint32_t max_count(void)
    {
        return N;
    }


    /**
     * \brief       Get pointer to the list of parameters
     *
     * \return      task list
     */
    param_entry_t* parameters(void)
    {
        return parameters_;
    }


private:
    param_entry_t parameters_[N];         ///< Onboard parameters array
};


/**
 * \brief   Default configuration
 *
 * \return  Config
 */
Onboard_parameters::conf_t Onboard_parameters::default_config(void)
{
    conf_t conf = {};
    conf.debug = false;
    return conf;
}

#endif /* ONBOARD_PARAMETERS_HPP_ */
