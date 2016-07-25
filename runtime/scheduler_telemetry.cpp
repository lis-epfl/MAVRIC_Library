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
 * \file scheduler_telemetry.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of sending periodic telemetric messages for
 * the scheduler module
 *
 ******************************************************************************/

#include "runtime/scheduler_telemetry.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/maths.h"
}

void scheduler_telemetry_send_rt_stats(const Scheduler* scheduler, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    // static int id = 0
    // id ++;
    Scheduler_task* stab_task = scheduler->get_task_by_id(0);

    mavlink_msg_named_value_float_pack(mavlink_stream->sysid(),
                                       mavlink_stream->compid(),
                                       msg,
                                       time_keeper_get_ms(),
                                       "DelayAvg",
                                       stab_task->delay_avg);
    mavlink_stream->send(msg);

    mavlink_msg_named_value_float_pack(mavlink_stream->sysid(),
                                       mavlink_stream->compid(),
                                       msg,
                                       time_keeper_get_ms(),
                                       "DelayVar",
                                       maths_fast_sqrt(stab_task->delay_var));
    mavlink_stream->send(msg);

    mavlink_msg_named_value_float_pack(mavlink_stream->sysid(),
                                       mavlink_stream->compid(),
                                       msg,
                                       time_keeper_get_ms(),
                                       "DelayMax",
                                       stab_task->delay_max);
    mavlink_stream->send(msg);

    mavlink_msg_named_value_float_pack(mavlink_stream->sysid(),
                                       mavlink_stream->compid(),
                                       msg,
                                       time_keeper_get_ms(),
                                       "RTViol",
                                       stab_task->rt_violations);
    mavlink_stream->send(msg);

    mavlink_msg_named_value_float_pack(mavlink_stream->sysid(),
                                       mavlink_stream->compid(),
                                       msg,
                                       time_keeper_get_ms(),
                                       "ExTime",
                                       stab_task->execution_time);
    mavlink_stream->send(msg);

    mavlink_msg_named_value_float_pack(mavlink_stream->sysid(),
                                       mavlink_stream->compid(),
                                       msg,
                                       time_keeper_get_ms(),
                                       "ExTimeAvg",
                                       stab_task->execution_time_avg);
    mavlink_stream->send(msg);

    mavlink_msg_named_value_float_pack(mavlink_stream->sysid(),
                                       mavlink_stream->compid(),
                                       msg,
                                       time_keeper_get_ms(),
                                       "ExTimeVar",
                                       stab_task->execution_time_var);
    mavlink_stream->send(msg);

    mavlink_msg_named_value_float_pack(mavlink_stream->sysid(),
                                       mavlink_stream->compid(),
                                       msg,
                                       time_keeper_get_ms(),
                                       "ExTimeMax",
                                       stab_task->execution_time_max);
    mavlink_stream->send(msg);


    stab_task->rt_violations = 0;
    stab_task->delay_max = 0;
    stab_task->execution_time_max = 0;
}

void scheduler_telemetry_send_rt_stats_all(const Scheduler* scheduler, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    float data[60];

    uint32_t n_tasks = scheduler->task_count();
    if (n_tasks > 10)
    {
        n_tasks = 10;
    }

    for (uint32_t i = 0; i < n_tasks; ++i)
    {
        // Get i-th task
        Scheduler_task* task = scheduler->get_task_by_id(i);

        if (task != NULL)
        {
            // Save real time statistics in data array
            data[6*i + 0] = task->execution_time_avg;
            data[6*i + 1] = task->execution_time_var;
            data[6*i + 2] = task->execution_time_max;
            data[6*i + 3] = task->delay_avg;
            data[6*i + 4] = task->delay_var;
            data[6*i + 5] = task->delay_max;
        }
    }

    mavlink_msg_big_debug_vect_pack(mavlink_stream->sysid(),
                                    mavlink_stream->compid(),
                                    msg,
                                    "RTstat",
                                    time_keeper_get_us(),
                                    data);
}
