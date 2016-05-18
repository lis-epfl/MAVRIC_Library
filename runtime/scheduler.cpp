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

Scheduler::Scheduler(const Scheduler::conf_t config) :
    schedule_strategy_(config.schedule_strategy),
    debug_(config.debug),
    task_count_(0),
    current_schedule_slot_(0)
{
    // allocate memory for tasks
    for(max_task_count_ = config.max_task_count; max_task_count_ > 0; max_task_count_--)
    {
        tasks_ = (Scheduler_task*)malloc(sizeof(Scheduler_task)*max_task_count_);

        if(tasks_ != NULL)
        {
            break;
        }
    }
    if(max_task_count_< config.max_task_count)
    {
        print_util_dbg_print("[Scheduler] constructor: tried to allocate task list for ");
        print_util_dbg_print_num(config.max_task_count,10);
        print_util_dbg_print(" tasks; only space for ");
        print_util_dbg_print_num(max_task_count_,10);
        print_util_dbg_print("\r\n");
    }
}


bool Scheduler::add_task(uint32_t repeat_period,
              Scheduler_task::task_function_t call_function,
              Scheduler_task::task_argument_t function_argument,
              Scheduler_task::priority_t priority,
              Scheduler_task::timing_mode_t timing_mode,
              Scheduler_task::run_mode_t run_mode,
              uint32_t task_id)
{
    bool task_successfully_added = false;

    // Check if the scheduler is not full
    if (task_count_ < max_task_count_)
    {
        if (task_id == 0)
        {
           task_id = task_count_;
        }

        // Check if there is already a task with this ID
        bool id_is_unique = true;
        for (uint32_t i = 0; i < task_count_; ++i)
        {
            if (tasks_[i].get_id() == task_id)
            {
                id_is_unique = false;
                break;
            }
        }

        // Add new task
        if (id_is_unique == true)
        {
            tasks_[task_count_++] = Scheduler_task(repeat_period, run_mode, timing_mode, priority, call_function, function_argument, task_id);
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

    if (task_count_ < 2)
    {
        sorted = true;
        return sorted;
    }

    while (sorted == false)
    {
        sorted = true;

        // Iterate through registered tasks
        for (uint32_t i = 0; i < (task_count_ - 1); i++)
        {
            if (tasks_[i].priority < tasks_[i + 1].priority)
            {
                // Task i has lower priority than task i+1 -> need swap
                sorted = false;
            }
            else if (tasks_[i].priority == tasks_[i + 1].priority)
            {
                if (tasks_[i].repeat_period > tasks_[i + 1].repeat_period)
                {
                    // Tasks i and i+1 have equal priority, but task i has higher
                    // repeat period than task i+1 -> need swap
                    sorted = false;
                }
            }

            // Swap tasks i and i+1 if necessary
            if (sorted == false)
            {
                Scheduler_task tmp(tasks_[i]);
                tasks_[i] = tasks_[i + 1];
                tasks_[i + 1] = tmp;
                sorted = false;
            }
        }
    }
    return sorted;
}


int32_t Scheduler::update()
{
    int32_t realtime_violation = 0;

    // Iterate through registered tasks
    uint32_t i = current_schedule_slot_;
    do
    {
        // If the task is active and has waited long enough...
        if (tasks_[i].is_due())
        {
            // Execute task
            if (!tasks_[i].execute())
            {
                realtime_violation++; //realtime violation!!
            }

            // Depending on shceduling strategy, select next task slot
            switch (schedule_strategy_)
            {
                case FIXED_PRIORITY:
                    // Fixed priority scheme - scheduler will start over with tasks with the highest priority
                    current_schedule_slot_ = 0;
                    break;

                case ROUND_ROBIN:
                    // Round robin scheme - scheduler will pick up where it left.
                    current_schedule_slot_ = (current_schedule_slot_+1)%task_count_;
                    break;
            }

            return realtime_violation;
        }
        i = (i+1)%task_count_;

    } while(i != current_schedule_slot_);

    return realtime_violation;
}


Scheduler_task* Scheduler::get_task_by_id(uint16_t task_id) const
{

    for (uint32_t i = 0; i < task_count_; i++)
    {
        if (tasks_[i].task_id == task_id)
        {
            return &tasks_[i];
        }
    }

    return NULL;
}


Scheduler_task* Scheduler::get_task_by_index(uint16_t task_index) const
{
    if (task_index < task_count_)
    {
        return &tasks_[task_index];
    }

    return NULL;
}


void Scheduler::suspend_all_tasks(uint32_t delay)
{
    for(uint32_t i = 0; i < task_count_; i++)
    {
        tasks_[i].suspend(delay);
    }
}


void Scheduler::run_all_tasks_now()
{
    for(uint32_t i = 0; i < task_count_; i++)
    {
        tasks_[i].run_now();
    }
}


/*************************************************************************
 *                 static member functions                               *
 ************************************************************************/

Scheduler::conf_t Scheduler::default_config()
{
    Scheduler::conf_t conf  = {};

    conf.max_task_count    = 15;
    // conf.schedule_strategy = FIXED_PRIORITY;
    conf.schedule_strategy = ROUND_ROBIN;
    conf.debug             = true;

    return conf;
};
