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
 * \file scheduler.hpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *
 * \brief Scheduler
 *
 ******************************************************************************/


#ifndef SCHEDULER_HPP_
#define SCHEDULER_HPP_

#include <cstdint>
#include "runtime/scheduler_task.hpp"


/**
 * \brief   Scheduler base class
 *
 * \details This class is abstract and does not contains the task list,
 *          use the child class Scheduler_tpl
 */
class Scheduler
{

public:
    ///< time base for the scheduler
    static const uint32_t TIMEBASE = 1000000;

    /**
     * \brief   Scheduling strategy
     */
    enum strategy_t
    {
        ROUND_ROBIN,                ///<    Round robin scheduling
        FIXED_PRIORITY              ///<    Fixed priority scheduling
    };


    /**
     * \brief Scheduler configuration
     */
    struct conf_t
    {
        Scheduler::strategy_t schedule_strategy;    ///<    Schedule strategy
        bool debug;                                 ///<    Indicates whether the scheduler should print debug messages
    };


    /**
     * \brief               Constructor
     *
     * \param   config      Configuration
     */
    Scheduler(const Scheduler::conf_t config = default_config());


    /**
     * \brief                   Register a new task to the task set, in the first available slot
     *
     * \param repeat_period     Repeat period (us)
     * \param call_function     Function pointer to be called
     * \param function_argument Argument to be passed to the function
     * \param priority          Priority
     * \param task_id           Unique task identifier, if -1 the ID will be automatically chosen
     * \param timing_mode       Timing mode
     * \param run_mode          Run mode
     *
     * \return                  True if the task was successfully added, False if not
     */
    bool add_task(uint32_t repeat_period,
                  Scheduler_task::task_function_t call_function,
                  Scheduler_task::task_argument_t function_argument,
                  Scheduler_task::priority_t priority       = Scheduler_task::PRIORITY_NORMAL,
                  Scheduler_task::timing_mode_t timing_mode = Scheduler_task::PERIODIC_ABSOLUTE,
                  Scheduler_task::run_mode_t run_mode       = Scheduler_task::RUN_REGULAR,
                  int32_t task_id                           = -1);


    /**
     * \brief                Sort tasks by decreasing priority, then by increasing repeat period
     *
     * \return               True if the task was successfully sorted, False if not
     */
    bool sort_tasks(void);


    /**
     * \brief                Run update (check for tasks ready for execution and execute them)
     *
     * \return               Number of realtime violations
     */
    int32_t update(void);


    /**
     * \brief               Find a task according to its idea
     *
     * \param   task_id     ID of the target task
     *
     * \return              Pointer to the target task
     */
    const Scheduler_task* get_task_by_id(uint16_t task_id) const;
    Scheduler_task* get_task_by_id(uint16_t task_id);


    /**
     * \brief               Find a task according to its index
     *
     * \param task_index    Index of the target task
     *
     * \return              Pointer to the target task
     */
    const Scheduler_task* get_task_by_index(uint16_t task_index) const;
    Scheduler_task* get_task_by_index(uint16_t task_index);


    /**
     * \brief           Suspends all tasks
     *
     * \param delay     Duration (us)
     */
    void suspend_all_tasks(uint32_t delay);


    /**
     * \brief       Run all tasks immediately
     *
     */
    void run_all_tasks_now(void);


    /**
     * \brief       Return the number of tasks
     */
    uint32_t task_count(void) const;


    /**
     * \brief       Creates and returns default config for schedulers
     *
     * \return      default_config
     */
    static Scheduler::conf_t default_config(void);

protected:
    /**
     * \brief       Get maximum number of tasks
     * \details     To be overriden by child class
     *
     * \return      Maximum number of tasks
     */
    virtual uint32_t max_task_count(void) = 0;


    /**
     * \brief       Get pointer to the task list
     *
     * \details     Abstract method to be implemented in child classes
     *
     * \return      task list
     */
    virtual Scheduler_task* tasks(void) = 0;
    virtual const Scheduler_task* tasks(void) const = 0;

private:
    strategy_t schedule_strategy_;               ///<    Scheduling strategy
    bool debug_;                                 ///<    Indicates whether the scheduler should print debug messages
    uint32_t task_count_;                        ///<    Number_of_tasks
    uint32_t current_schedule_slot_;             ///<    Slot of the task being executed
};


/**
 * \brief   Scheduler
 *
 * \tparam  N   Maximum number of tasks
 */
template<uint32_t N = 10>
class Scheduler_tpl: public Scheduler
{
public:
    /**
     * \brief       Constructor
     */
    Scheduler_tpl(const Scheduler::conf_t config = default_config()):
        Scheduler(config)
    {;}

protected:

    /**
     * \brief       Get maximum number of tasks
     * \details     To be overriden by child class
     *
     * \return      Maximum number of tasks
     */
    uint32_t max_task_count(void)
    {
        return N;
    };

    /**
     * \brief Get pointer to the task list
     *
     * \return task list
     */
    Scheduler_task* tasks(void)
    {
        return tasks_;
    }

    const Scheduler_task* tasks(void) const
    {
        return tasks_;
    }

private:
    Scheduler_task  tasks_[N];           ///< Array of tasks to be executed
};


#endif /* SCHEDULER_HPP_ */
