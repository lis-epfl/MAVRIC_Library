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
 * \file periodic_telemetry.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Periodic telemetry
 *
 ******************************************************************************/


#ifndef PERIODIC_TELEMETRY_HPP_
#define PERIODIC_TELEMETRY_HPP_

#include <cstdbool>

#include "communication/mavlink_stream.hpp"
#include "runtime/scheduler.hpp"

class Periodic_telemetry
{
public:

    /**
     * \brief   Configuration structure
     */
    struct conf_t
    {
        Scheduler::conf_t  scheduler_config;   ///< Configuration for scheduler
    };


    /**
     * \brief   Default configuration structure
     *
     *\return   Config
     */
    static inline conf_t default_config(void);


    /**
     * \brief       Prototype of telemetry functions
     *
     * \detail  If gcc > 4.7 was supported on AVR32, we could use sth like
     *          ```
     *          template<typename T>
     *          using function_T = void(*)(const T*, const Mavlink_stream*, const mavlink_message_t*);
     *          ```
     */
    template<typename T>
    struct function
    {
        typedef void (*type_t)(const T*, const Mavlink_stream*, mavlink_message_t*);
    };


    /**
     * \brief   Constructor
     *
     * \param   mavlink_stream      Stream to which messages will be written
     * \param   config              Configuration structure
     */
    Periodic_telemetry(Mavlink_stream& mavlink_stream, conf_t config = default_config());


    /**
     * \brief Main update function
     */
    bool update(void);


    /**
     * \brief   Add new telemetry message to the scheduler
     *
     * \tparam  T                       Type of the module
     *
     * \param   task_id                 Unique task identifier
     * \param   repeat_period           Repeat period (us)
     * \param   telemetry_function      Function pointer to be called
     * \param   telemetry_module        Argument to be passed to the function
     * \param   priority                Priority
     * \param   timing_mode             Timing mode
     * \param   run_mode                Run mode
     *
     * \return  True if the message was correctly added, false otherwise
     */
    template<typename T>
    bool add(   uint32_t                        task_id,
                uint32_t                        repeat_period,
                typename function<T>::type_t    telemetry_function,
                T*                              telemetry_module,
                Scheduler_task::priority_t      priority    = Scheduler_task::PRIORITY_NORMAL,
                Scheduler_task::timing_mode_t   timing_mode = Scheduler_task::PERIODIC_RELATIVE,
                Scheduler_task::run_mode_t      run_mode    = Scheduler_task::RUN_REGULAR);


    /**
     * \brief                Sort telemetry items by decreasing priority, then by increasing repeat period
     *
     * \return               True if the task was successfully sorted, False if not
     */
    bool sort(void);


    /**
     * \brief   Toggle mavlink telemetry stream
     *
     * \param scheduler     scheduler of the MAV
     * \param sysid         MAV sysid
     * \param msg           pointer to the stream you want to toggle
     */
    static void toggle_telemetry_stream(Periodic_telemetry* scheduler, uint32_t sysid, const mavlink_message_t* msg);


protected:

    /**
    * \brief   MAVLink message handler structure
    */
    struct telemetry_entry_t
    {
        function<void>::type_t  function;           ///<    Pointer to the function to be executed
        void*                   module;             ///<    Pointer to module data structure to be given as argument to the function
        Mavlink_stream*         mavlink_stream;     ///<    Pointer to the MAVLink stream structure
    };

    /**
     * \brief       Get maximum number of telemetry messages
     * \details     To be overriden by child class
     *
     * \return      Maximum number
     */
    virtual uint32_t max_count(void) = 0;


    /**
     * \brief       Get reference to scheduler
     * \details     To be overriden by child class
     *
     * \return      Scheduler
     */
    virtual Scheduler& scheduler(void) = 0;


    /**
     * \brief       Get pointer to list of telemetry items
     * \details     To be overriden by child class
     *
     * \return      list
     */
    virtual telemetry_entry_t* list(void) = 0;


private:

    Mavlink_stream&             mavlink_stream_;                    ///<    Mavlink stream
    uint32_t                    count_;                             ///<    Number of telemetry items currently registered

    /**
     * \brief   Prepare and send a telemetry message
     *
     * \param   telemetry_entry     Message to send
     *
     * \return  The result of execution of the task
     */
    static bool send_message(telemetry_entry_t* telemetry_entry);
};


template<uint32_t N = 10>
class Periodic_telemetry_T: public Periodic_telemetry
{
public:

    /**
     * \brief   Constructor
     *
     * \param   mavlink_stream      Stream to which messages will be written
     * \param   config              Configuration structure
     */
    Periodic_telemetry_T(Mavlink_stream& mavlink_stream, conf_t config):
        Periodic_telemetry(mavlink_stream, config),
        scheduler_(config.scheduler_config)
    {};

protected:

    /**
     * \brief       Get maximum number of telemetry messages
     *
     * \return      Maximum number
     */
    uint32_t max_count(void)
    {
        return N;
    }

    /**
     * \brief       Get reference to scheduler
     * \details     To be overriden by child class
     *
     * \return      Scheduler
     */
    Scheduler& scheduler(void)
    {
        return scheduler_;
    }

    /**
     * \brief       Get pointer to list of telemetry items
     * \details     To be overriden by child class
     *
     * \return      list
     */
    telemetry_entry_t* list(void)
    {
        return list_;
    }

private:
    Scheduler_T<N>     scheduler_;          ///<    Task set for scheduling of down messages
    telemetry_entry_t    list_[N];            ///<    List of message callbacks
};



Periodic_telemetry::conf_t Periodic_telemetry::default_config(void)
{
    Periodic_telemetry::conf_t conf  = {};

    conf.scheduler_config.schedule_strategy = Scheduler::ROUND_ROBIN;
    conf.scheduler_config.debug             = false;

    return conf;
};

#include "periodic_telemetry.hxx"

#endif /* PERIODIC_TELEMETRY_HPP_ */
