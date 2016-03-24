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
 * \file scheduler.cpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *
 * \brief Scheduler
 *
 ******************************************************************************/


#include "runtime/scheduler.hpp"
#include "hal/common/time_keeper.hpp"
#include <stdlib.h>
#include "util/print_util.h"

Scheduler::Scheduler(const scheduler_conf_t config)
{
    bool init_success = true;

    // Init schedule strategy
    schedule_strategy = config.schedule_strategy;

    // Init debug mode
    debug = config.debug;

    
    // allocate memory for tasks
    for(max_task_count = config.max_task_count; max_task_count > 0; max_task_count--)
    {
        tasks = (task_entry_t*)malloc(sizeof(task_entry_t[config.max_task_count]));
        if(tasks != NULL)
        {
            break;
        }
    }
    if(max_task_count < config.max_task_count)
    {
        print_util_dbg_print("[Scheduler] constructor: tried to allocate task list for ");
        print_util_dbg_print_num(config.max_task_count,10);
        print_util_dbg_print(" tasks; only space for ");
        print_util_dbg_print_num(max_task_count,10);
        print_util_dbg_print("\r\n");
        init_success &= false;
    }

    task_count = 0;
    current_schedule_slot = 0;

    print_util_dbg_print("[SCHEDULER] Init\r\n");
}


bool Scheduler::add_task(uint32_t repeat_period, task_run_mode_t run_mode, task_timing_mode_t timing_mode, task_priority_t priority, task_function_t call_function, task_argument_t function_argument, uint32_t task_id)
{
    bool task_successfully_added = false;

    // Check if the scheduler is not full
    if (task_count < max_task_count)
    {
        // Check if there is already a task with this ID
        bool id_is_unique = true;
        for (uint32_t i = 0; i < task_count; ++i)
        {
            if (tasks[i].task_id == task_id)
            {
                id_is_unique = false;
                break;
            }
        }

        // Add new task
        if (id_is_unique == true)
        {
            task_entry_t* new_task = &tasks[task_count];

            new_task->call_function     = call_function;
            new_task->function_argument = function_argument;
            new_task->task_id           = task_id;
            new_task->run_mode          = run_mode;
            new_task->timing_mode       = timing_mode;
            new_task->priority          = priority;
            new_task->repeat_period     = repeat_period;
            new_task->next_run          = time_keeper_get_us();
            new_task->execution_time    = 0;
            new_task->delay_max         = 0;
            new_task->delay_avg         = 0;
            new_task->delay_var_squared = 0;

            task_count += 1;

            task_successfully_added = true;
        }
        else
        {
            print_util_dbg_print("[SCHEDULER] Error: There is already a task with this ID\r\n");
            task_successfully_added = false;
        }
    }
    else
    {
        print_util_dbg_print("[SCHEDULER] Error: Cannot add more task\r\n");
        task_successfully_added = false;
    }

    return task_successfully_added;
}


bool Scheduler::sort_tasks()
{
    bool sorted = false;

    task_entry_t tmp;

    if (task_count < 2)
    {
        sorted = true;
        return sorted;
    }

    while (sorted == false)
    {
        sorted = true;

        // Iterate through registered tasks
        for (uint32_t i = 0; i < (task_count - 1); i++)
        {
            if (tasks[i].priority < tasks[i + 1].priority)
            {
                // Task i has lower priority than task i+1 -> need swap
                sorted = false;
            }
            else if (tasks[i].priority == tasks[i + 1].priority)
            {
                if (tasks[i].repeat_period > tasks[i + 1].repeat_period)
                {
                    // Tasks i and i+1 have equal priority, but task i has higher
                    // repeat period than task i+1 -> need swap
                    sorted = false;
                }
            }

            // Swap tasks i and i+1 if necessary
            if (sorted == false)
            {
                tmp = tasks[i];
                tasks[i] = tasks[i + 1];
                tasks[i + 1] = tmp;
                sorted = false;
            }
        }
    }
    return sorted;
}


int32_t Scheduler::update()
{
    int32_t realtime_violation = 0;

    task_function_t call_task;
    task_argument_t function_argument;

    // Iterate through registered tasks
    for (uint32_t i = current_schedule_slot; i < task_count; i++)
    {
        uint32_t current_time = time_keeper_get_us();

        // If the task is active and has waited long enough...
        if ((tasks[i].run_mode != RUN_NEVER) && (current_time >= tasks[i].next_run))
        {
            uint32_t delay = current_time - (tasks[i].next_run);
            uint32_t task_start_time;

            task_start_time = time_keeper_get_us();

            // Get function pointer and function argument
            call_task = tasks[i].call_function;
            function_argument = tasks[i].function_argument;

            // Execute task
            bool task_success = call_task(function_argument);

            // Set the next execution time of the task
            if (task_success)
            {
                switch (tasks[i].timing_mode)
                {
                    case PERIODIC_ABSOLUTE:
                        // Do not take delays into account
                        tasks[i].next_run += tasks[i].repeat_period;
                        break;

                    case PERIODIC_RELATIVE:
                        // Take delays into account
                        tasks[i].next_run = time_keeper_get_us() + tasks[i].repeat_period;
                        break;
                }
            }

            // Set the task to inactive if it has to run only once
            if (tasks[i].run_mode == RUN_ONCE)
            {
                tasks[i].run_mode = RUN_NEVER;
            }

            // Check real time violations
            if (tasks[i].next_run < current_time)
            {
                realtime_violation = -i; //realtime violation!!
                tasks[i].rt_violations++;
                tasks[i].next_run = current_time + tasks[i].repeat_period;
            }

            // Compute real-time statistics
            tasks[i].delay_avg = (7 * tasks[i].delay_avg + delay) / 8;
            if (delay > tasks[i].delay_max)
            {
                tasks[i].delay_max = delay;
            }
            tasks[i].delay_var_squared = (15 * tasks[i].delay_var_squared + (delay - tasks[i].delay_avg) * (delay - tasks[i].delay_avg)) / 16;
            tasks[i].execution_time = (7 * tasks[i].execution_time + (time_keeper_get_us() - task_start_time)) / 8;

            // Depending on shceduling strategy, select next task slot
            switch (schedule_strategy)
            {
                case FIXED_PRIORITY:
                    // Fixed priority scheme - scheduler will start over with tasks with the highest priority
                    current_schedule_slot = 0;
                    break;

                case ROUND_ROBIN:
                    // Round robin scheme - scheduler will pick up where it left.
                    // if (i >= task_count - 1)
                    // {
                    //  current_schedule_slot = 0;
                    // }
                    // else
                    // {
                    current_schedule_slot = 0;
                    // }
                    break;
            }

            return realtime_violation;
        }
    }
    return realtime_violation;
}


task_entry_t* Scheduler::get_task_by_id(uint16_t task_id) const
{

    for (uint32_t i = 0; i < task_count; i++)
    {
        if (tasks[i].task_id == task_id)
        {
            return &tasks[i];
        }
    }

    return NULL;
}


task_entry_t* Scheduler::get_task_by_index(uint16_t task_index) const
{
    if (task_index < task_count)
    {
        return &tasks[task_index];
    }

    return NULL;
}


bool Scheduler::is_debug()
{
    return debug;
}


void Scheduler::suspend_all_tasks(uint32_t delay)
{
    for(uint32_t i = 0; i < task_count; i++)
    {
        suspend_task(&tasks[i], delay);
    }
}


void Scheduler::run_all_tasks_now()
{
    for(uint32_t i = 0; i < task_count; i++)
    {
        run_task_now(&tasks[i]);
    }
}


/*************************************************************************
 *                 static member functions                               *
 ************************************************************************/


void Scheduler::change_run_mode(task_entry_t* te, task_run_mode_t new_run_mode)
{
    te->run_mode = new_run_mode;
}


void Scheduler::change_task_period(task_entry_t* te, uint32_t repeat_period)
{
    te->repeat_period = repeat_period;
    change_run_mode(te, RUN_REGULAR);
    run_task_now(te);
}


void Scheduler::suspend_task(task_entry_t* te, uint32_t delay)
{
    te->next_run = time_keeper_get_us() + delay;
}


void Scheduler::run_task_now(task_entry_t* te)
{
    if (te->run_mode == RUN_NEVER)
    {
        te->run_mode = RUN_ONCE;
    }

    te->next_run = time_keeper_get_us();
}