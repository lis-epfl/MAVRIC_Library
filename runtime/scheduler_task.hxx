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
 * \file scheduler_task.hxx
 *
 * \author MAV'RIC Team
 *
 * \brief Task managed by the scheduler
 *
 ******************************************************************************/


template<typename T>
Scheduler_task::Scheduler_task( uint32_t repeat_period,
                                run_mode_t run_mode,
                                timing_mode_t timing_mode,
                                priority_t priority,
                                typename function<T>::type_t task_function,
                                T* task_argument,
                                int32_t task_id):
    task_id(task_id),
    run_mode(run_mode),
    timing_mode(timing_mode),
    priority(priority),
    repeat_period(repeat_period),
    next_run(0),
    execution_time(0),
    execution_time_avg(0),
    execution_time_var(0),
    execution_time_max(0),
    delay(0),
    delay_avg(0),
    delay_var(0),
    delay_max(0),
    task_function(reinterpret_cast<function<void>::type_t>(task_function)),  // we do dangerous casting here, but it is safe because
    task_argument(reinterpret_cast<void*>(task_argument))                    // the types of task_function and task_argument are compatible
{}
