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

#include <stdint.h>


#define SchedulerIMEBASE 1000000  ///< time base for the scheduler

typedef uint8_t task_handle_t;


/**
 * \brief   Task return code
 */
typedef enum
{
    TASK_RUN_ERROR   = -1,      ///<    The task was not not successfully executed
    TASK_RUN_BLOCKED = 0,       ///<    If a task returns "TASK_RUN_BLOCKED", the scheduler will try to re-run at the next schedule update, and not update "next_run"
    TASK_RUN_SUCCESS = 1        ///<    The task was successfully executed
} task_return_t;


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
typedef enum
{
    RUN_NEVER,                  ///<    The task will not be executed
    RUN_ONCE,                   ///<    The task will be executed only once
    RUN_REGULAR                 ///<    The task will be executed periodically
} task_run_mode_t;


/**
 * \brief   Task timing mode
 */
typedef enum
{
    PERIODIC_ABSOLUTE,          ///<    The task will be executed according to a fixed schedule (ie. regardless of past delays or realtime violations)
    PERIODIC_RELATIVE           ///<    The task will be executed with constant period relative to the last execution
} task_timing_mode_t;


/**
 * \brief   Task priority
 */
typedef enum
{
    PRIORITY_LOWEST  = 0,       ///<    Lowest priority
    PRIORITY_LOW     = 1,       ///<    Low priority
    PRIORITY_NORMAL  = 2,       ///<    Normal priority
    PRIORITY_HIGH    = 3,       ///<    High priority
    PRIORITY_HIGHEST = 4        ///<    Highest priority
} task_priority_t;


/**
 * \brief   Scheduling strategy
 */
typedef enum
{
    ROUND_ROBIN,                ///<    Round robin scheduling
    FIXED_PRIORITY              ///<    Fixed priority scheduling
} schedule_strategy_t;


/**
 * \brief   Task entry
 */
typedef struct
{
    task_function_t     call_function;          ///<    Function to be called
    task_argument_t     function_argument;      ///<    Argument to be passed to the function
    uint32_t            task_id;                ///<    Unique task identifier
    task_run_mode_t     run_mode;               ///<    Run mode
    task_timing_mode_t  timing_mode;            ///<    Timing mode
    task_priority_t     priority;               ///<    Priority
    uint32_t            repeat_period;          ///<    Period between two calls (us)
    uint32_t            next_run;               ///<    Next execution time
    uint32_t            execution_time;         ///<    Execution time
    uint32_t            delay_max;              ///<    Maximum delay between expected execution and actual execution
    uint32_t            delay_avg;              ///<    Average delay between expected execution and actual execution
    uint32_t            delay_var_squared;      ///<    Standard deviation of the delay
    uint32_t            rt_violations;          ///<    Number of Real-time violations, this is incremented each time an execution is skipped
} task_entry_t;


/**
 * \brief   Task set
 *
 * \details     Uses C99's flexible member arrays: it is required to
 *              allocate memory for this structure
 */
typedef struct task_set_t
{
    uint32_t task_count;                        ///<    Number_of_tasks
    uint32_t max_task_count;                    ///<    Maximum number of tasks
    uint32_t current_schedule_slot;             ///<    Slot of the task being executed
    task_entry_t tasks[];                       ///<    Array of tasks_entry to be executed, needs memory allocation
} task_set_t;


/**
 * \brief Scheduler configuration
 */
typedef struct
{
    uint32_t max_task_count;                    ///<    Maximum number of tasks
    schedule_strategy_t schedule_strategy;      ///<    Schedule strategy
    bool debug;                                 ///<    Indicates whether the scheduler should print debug messages
} scheduler_conf_t;


/**
 * \brief   Scheduler
 *
 * \details     task_set is implemented as pointer because its memory will be
 *              allocated during initialisation
 */
class Scheduler
{
public:
    /**
     * \brief               Constructor
     *
     * \param   config      Configuration
     */
    Scheduler(const scheduler_conf_t config);


    /**
     * \brief                   Register a new task to the task set, in the first available slot
     *
     * \param repeat_period     Repeat period (us)
     * \param run_mode          Run mode
     * \param timing_mode       Timing mode
     * \param priority          Priority
     * \param call_function     Function pointer to be called
     * \param function_argument Argument to be passed to the function
     * \param task_id           Unique task identifier
     *
     * \return                  True if the task was successfully added, False if not
     */
    bool add_task(uint32_t repeat_period, task_run_mode_t run_mode, task_timing_mode_t timing_mode, task_priority_t priority, task_function_t call_function, task_argument_t function_argument, uint32_t task_id);


    /**
     * \brief                Sort tasks by decreasing priority, then by increasing repeat period
     *
     * \return               True if the task was successfully sorted, False if not
     */
    bool sort_tasks();


    /**
     * \brief                Run update (check for tasks ready for execution and execute them)
     *
     * \return               Number of realtime violations
     */
    int32_t update();


    /**
     * \brief               Find a task according to its idea
     *
     * \param   task_id     ID of the target task
     *
     * \return              Pointer to the target task
     */
    task_entry_t* get_task_by_id(uint16_t task_id) const;


    /**
     * \brief               Find a task according to its index
     *
     * \param task_index    Index of the target task
     *
     * \return              Pointer to the target task
     */
    task_entry_t* get_task_by_index(uint16_t task_index) const;


    /**
     * \brief               Returns whether scheduler is in debug mode (printing messages)
     *
     * \param task_index    Index of the target task
     *
     * \return              Pointer to the target task
     */
    bool is_debug();


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
    void run_all_tasks_now();


    /**
     * \brief               Modifies the run mode of an existing task
     *
     * \param te            Pointer to a task entry
     * \param new_run_mode  New run mode
     */
    static void change_run_mode(task_entry_t* te, task_run_mode_t new_run_mode);


    /**
     * \brief                   Modifies the period of execution of an existing task
     *
     * \param tep                Pointer to a task entry
     * \param repeat_period     New repeat period (us)
     */
    static void change_task_period(task_entry_t* te, uint32_t repeat_period);


    /**
     * \brief           Suspends a task
     *
     * \param te        Pointer to a task entry
     * \param delay     Duration (us)
     */
    static void suspend_task(task_entry_t* te, uint32_t delay);


    /**
     * \brief       Run a task immediately
     *
     * \param te    Pointer to a task entry
     */
    static void run_task_now(task_entry_t* te);


private:
    task_set_t* task_set;                       ///<    Pointer to task set, needs memory allocation
    schedule_strategy_t schedule_strategy;      ///<    Scheduling strategy
    bool debug;                                 ///<    Indicates whether the scheduler should print debug messages

};

#endif /* SCHEDULER_HPP_ */