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
 * \file scheduler_telemetry.c
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of sending periodic telemetric messages for
 * the scheduler module
 *
 ******************************************************************************/

#include "runtime/scheduler_telemetry.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
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
