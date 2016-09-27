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
 * \file scheduler_task.cpp
 *
 * \author MAV'RIC Team
 *
 * \brief Task managed by the scheduler
 *
 ******************************************************************************/


#include "runtime/scheduler_task.hpp"
#include "hal/common/time_keeper.hpp"

Scheduler_task::Scheduler_task(void)
{}


void Scheduler_task::set_run_mode(run_mode_t mode)
{
    run_mode = mode;
}


void Scheduler_task::run_now()
{
    if (run_mode == RUN_NEVER)
    {
        run_mode = RUN_ONCE;
    }

    next_run = time_keeper_get_us();
}


void Scheduler_task::suspend(uint32_t delay)
{
    next_run = time_keeper_get_us() + delay;
}


void Scheduler_task::change_period(uint32_t repeat_period)
{
    repeat_period = repeat_period;
    set_run_mode(RUN_REGULAR);
    run_now();
}


int32_t Scheduler_task::get_id()
{
    return task_id;
}


bool Scheduler_task::execute()
{
    uint32_t task_start_time = time_keeper_get_us();

    // If the task runs for the first time, we assume it runs on time
    if(next_run == 0)
    {
        next_run = task_start_time;
    }

    delay = task_start_time - next_run;

    // Execute task
    bool success = task_function(task_argument);

    // Set the next execution time of the task
    if (success)
    {
        switch (timing_mode)
        {
            case PERIODIC_ABSOLUTE:
                // Do not take call_delays into account
                next_run += repeat_period;
                break;

            case PERIODIC_RELATIVE:
                // Take call_delays into account
                next_run = time_keeper_get_us() + repeat_period;
                break;
        }
    }

    // Set the task to inactive if it has to run only once
    if (run_mode == Scheduler_task::RUN_ONCE)
    {
        run_mode = Scheduler_task::RUN_NEVER;
    }

    // Check real time violations
    bool is_violation = false;
    if (next_run < task_start_time)
    {
        rt_violations++;
        next_run = task_start_time + repeat_period;
        is_violation = true;
    }

    // Compute real-time statistics on execution time
    execution_time     = time_keeper_get_us() - task_start_time;
    execution_time_avg = (7.0f * execution_time_avg + execution_time) / 8.0f;
    execution_time_var = (15.0f * execution_time_var + (execution_time - execution_time_avg) * (execution_time - execution_time_avg)) / 16.0f;
    if (execution_time > execution_time_max)
    {
        execution_time_max = execution_time;
    }

    // Compute real-time statistics on delay
    delay_avg = (7.0f * delay_avg + delay) / 8.0f;
    delay_var = (15.0f * delay_var + (delay - delay_avg) * (delay - delay_avg)) / 16.0f;
    if (delay > delay_max)
    {
        delay_max = delay;
    }

    return !is_violation;
}


bool Scheduler_task::is_due()
{
    return (run_mode != RUN_NEVER) && (time_keeper_get_us() >= next_run);
}
