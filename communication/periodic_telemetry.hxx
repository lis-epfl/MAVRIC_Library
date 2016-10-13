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
 * \file periodic_telemetry.hxx
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Periodic telemetry
 *
 ******************************************************************************/

template<typename T>
bool Periodic_telemetry::add(   uint32_t                        task_id,
                                uint32_t                        repeat_period,
                                typename function<T>::type_t    telemetry_function,
                                T*                              telemetry_module,
                                Scheduler_task::priority_t      priority,
                                Scheduler_task::timing_mode_t   timing_mode,
                                Scheduler_task::run_mode_t      run_mode)
{
    bool add_success = true;

    if (count_ <  max_count())
    {
        telemetry_entry_t* new_entry = &list()[count_++];

        new_entry->mavlink_stream = &mavlink_stream_;
        new_entry->function       = reinterpret_cast<function<void>::type_t>(telemetry_function);   // we do dangerous casting here, but it is safe because
        new_entry->module         = reinterpret_cast<void*>(telemetry_module);                      // the types of telemetry_function and telemetry_argument are compatible

        add_success &= true;

        add_success &= scheduler().add_task(repeat_period,
                                           &send_message,
                                           new_entry,
                                           priority,
                                           timing_mode,
                                           run_mode,
                                           task_id);
    }
    else
    {
        print_util_dbg_print("[MAVLINK COMMUNICATION] Error: Cannot add more send msg\r\n");

        add_success &= false;
    }

    return add_success;
}
