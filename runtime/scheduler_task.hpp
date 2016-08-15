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
 * \file scheduler_task.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief Task managed by the scheduler
 *
 ******************************************************************************/


#ifndef SCHEDULER_TASK_HPP_
#define SCHEDULER_TASK_HPP_

#include <cstdint>


/**
 * \brief   Task entry
 */
class Scheduler_task
{
public:
    /**
     * \brief   Task return code
     */
    enum return_t
    {
        RUN_ERROR   = -1,      ///<    The task was not not successfully executed
        RUN_BLOCKED = 0,       ///<    If a task returns "TASK_RUN_BLOCKED", the scheduler will try to re-run at the next schedule update, and not update "next_run"
        RUN_SUCCESS = 1        ///<    The task was successfully executed
    };


    /**
     * \brief   Generic argument to be passed to a task function
     */
    typedef void* task_argument_t;


    /**
     * \brief   Prototype of a task function
     */
    typedef bool (*task_function_t)(task_argument_t);


    /**
     * \brief   Task run mode
     */
    enum run_mode_t
    {
        RUN_NEVER,                  ///<    The task will not be executed
        RUN_ONCE,                   ///<    The task will be executed only once
        RUN_REGULAR                 ///<    The task will be executed periodically
    };


    /**
     * \brief   Task timing mode
     */
    enum timing_mode_t
    {
        PERIODIC_ABSOLUTE,          ///<    The task will be executed according to a fixed schedule (ie. regardless of past delays or realtime violations)
        PERIODIC_RELATIVE           ///<    The task will be executed with constant period relative to the last execution
    } ;


    /**
     * \brief   Task priority
     */
    enum priority_t
    {
        PRIORITY_LOWEST  = 0,       ///<    Lowest priority
        PRIORITY_LOW     = 1,       ///<    Low priority
        PRIORITY_NORMAL  = 2,       ///<    Normal priority
        PRIORITY_HIGH    = 3,       ///<    High priority
        PRIORITY_HIGHEST = 4        ///<    Highest priority
    };

    Scheduler_task(void);
    Scheduler_task( uint32_t repeat_period,
                    Scheduler_task::run_mode_t run_mode,
                    Scheduler_task::timing_mode_t timing_mode,
                    Scheduler_task::priority_t priority,
                    Scheduler_task::task_function_t call_function,
                    Scheduler_task::task_argument_t function_argument,
                    int32_t task_id);


    /**
     * \brief               Modifies the run mode of an existing task
     *
     * \param new_run_mode  New run mode
     */
    void set_run_mode(run_mode_t mode);


    /**
     * \brief       Run a task immediately
     *
     */
    void run_now();


    /**
     * \brief           Suspends a task
     *
     * \param delay     Duration (us)
     */
    void suspend(uint32_t delay);


    /**
     * \brief                   Modifies the period of execution of an existing task
     *
     * \param repeat_period     New repeat period (us)
     */
    void change_period(uint32_t repeat_period);


    /**
     * \brief           Returns the ID of the task
     *
     * \return          task_id
     */
    int32_t get_id();

    /**
     * \brief           Executes tasks and updates statistics
     *
     * \return          True if no real-time violation occured, false otherwise
     */
    bool execute();

    /**
     * \brief           Checks if the task is due
     *
     * \return          True if the task is due
     */
    bool is_due();

    int32_t             task_id;                ///<    Unique task identifier
    run_mode_t          run_mode;               ///<    Run mode
    timing_mode_t       timing_mode;            ///<    Timing mode
    priority_t          priority;               ///<    Priority
    uint32_t            repeat_period;          ///<    Period between two calls (us)
    uint32_t            next_run;               ///<    Next execution time

    float               execution_time;         ///<    Execution time
    float               execution_time_avg;     ///<    Average execution time
    float               execution_time_var;     ///<    Variance of execution time
    float               execution_time_max;     ///<    Maximum execution time
    float               delay;                  ///<    Delay between expected execution and actual execution
    float               delay_avg;              ///<    Average delay
    float               delay_var;              ///<    Variance of delay
    float               delay_max;              ///<    Maximum delay
    uint32_t            rt_violations;          ///<    Number of Real-time violations, this is incremented each time an execution is skipped

private:
    task_function_t     call_function;          ///<    Function to be called
    task_argument_t     function_argument;      ///<    Argument to be passed to the function
};

#endif /* SCHEDULER_TASK_HPP_ */
