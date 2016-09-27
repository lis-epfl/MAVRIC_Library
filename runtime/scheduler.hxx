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
 * \file scheduler.hxx
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *
 * \brief Scheduler
 *
 ******************************************************************************/

template<typename T>
bool Scheduler::add_task( uint32_t                                      repeat_period,
                          typename Scheduler_task::function<T>::type_t  task_function,
                          T*                                            task_argument,
                          Scheduler_task::priority_t                    priority,
                          Scheduler_task::timing_mode_t                 timing_mode,
                          Scheduler_task::run_mode_t                    run_mode,
                          int32_t                                       task_id)
{
    bool task_successfully_added = false;

    // Check if the scheduler is not full
    if (task_count_ < max_task_count())
    {
        if (task_id == -1)
        {
           task_id = task_count_;
        }

        // Check if there is already a task with this ID
        bool id_is_unique = true;
        for (uint32_t i = 0; i < task_count_; ++i)
        {
            if (tasks()[i].get_id() == task_id)
            {
                id_is_unique = false;
                break;
            }
        }

        // Add new task
        if (id_is_unique == true)
        {
            tasks()[task_count_++] = Scheduler_task(repeat_period,
                                                    run_mode,
                                                    timing_mode,
                                                    priority,
                                                    task_function,
                                                    task_argument,
                                                    task_id);
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
