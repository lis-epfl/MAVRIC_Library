/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 * \file scheduler.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *   
 * \brief Scheduler
 *
 ******************************************************************************/


#include "scheduler.h"
#include "time_keeper.h"
#include "print_util.h"
#include <stdlib.h>

bool scheduler_init(scheduler_t* scheduler, const scheduler_conf_t* config) 
{
	bool init_success = true;
	
	// Init schedule strategy
	scheduler->schedule_strategy = config->schedule_strategy;

	// Init debug mode
	scheduler->debug = config->debug;

	// Allocate memory for the task set
	scheduler->task_set = malloc( sizeof(task_set_t) + sizeof(task_entry_t[config->max_task_count]) );
	if ( scheduler->task_set != NULL ) 
	{
		scheduler->task_set->max_task_count = config->max_task_count;
		
		init_success &= true;
	}
	else
	{
		print_util_dbg_print("[SCHEDULER] ERROR ! Bad memory allocation\r\n");
		scheduler->task_set->max_task_count = 0;		
		
		init_success &= false;
	}

	scheduler->task_set->task_count = 0;
	scheduler->task_set->current_schedule_slot = 0;
	
	print_util_dbg_print("[SCHEDULER] Init\r\n");
	
	return init_success;
}


bool scheduler_add_task(scheduler_t* scheduler, uint32_t repeat_period, task_run_mode_t run_mode, task_timing_mode_t timing_mode, task_priority_t priority, task_function_t call_function, task_argument_t function_argument, uint32_t task_id) 
{
	bool task_successfully_added = false;
	task_set_t* ts = scheduler->task_set;

	// Check if the scheduler is not full
	if ( ts->task_count < ts->max_task_count ) 
	{
		// Check if there is already a task with this ID
		bool id_is_unique = true;
		for (uint32_t i = 0; i < ts->task_count; ++i)
		{
			if ( ts->tasks[i].task_id == task_id )
			{
				id_is_unique = false;
				break;
			}
		}

		// Add new task
		if ( id_is_unique == true )
		{
			task_entry_t* new_task = &ts->tasks[ts->task_count];

			new_task->call_function     = call_function;
			new_task->function_argument = function_argument;
			new_task->task_id           = task_id;		
			new_task->run_mode          = run_mode;
			new_task->timing_mode       = timing_mode;	
			new_task->priority          = priority;	
			new_task->repeat_period     = repeat_period;
			new_task->next_run          = time_keeper_get_micros();
			new_task->execution_time    = 0;
			new_task->delay_max         = 0;
			new_task->delay_avg         = 0;
			new_task->delay_var_squared = 0;

			ts->task_count += 1;

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


void scheduler_sort_tasks(scheduler_t* scheduler)
{
	bool sorted = false;

	task_set_t* ts = scheduler->task_set;	
	task_entry_t tmp;
	
	if (ts->task_count < 2) 
	{
		return;
	}

	while ( sorted == false ) 
	{
		sorted = true;
		
		// Iterate through registered tasks
		for (int32_t i = 0; i < (ts->task_count - 1); i++) 
		{
			if ( ts->tasks[i].priority < ts->tasks[i + 1].priority )
			{
				// Task i has lower priority than task i+1 -> need swap
				sorted = false;
			}
			else if ( ts->tasks[i].priority == ts->tasks[i + 1].priority )
			{
				if ( ts->tasks[i].repeat_period > ts->tasks[i + 1].repeat_period )
				{
					// Tasks i and i+1 have equal priority, but task i has higher
					// repeat period than task i+1 -> need swap
					sorted = false;
				}
			}
			
			// Swap tasks i and i+1 if necessary
			if ( sorted == false )
			{
				tmp = ts->tasks[i];
				ts->tasks[i] = ts->tasks[i + 1];
				ts->tasks[i + 1] = tmp;
				sorted = false;
			}
		}
	}	
}


int32_t scheduler_update(scheduler_t* scheduler) 
{
	int32_t realtime_violation = 0;

	task_set_t* ts = scheduler->task_set;

	task_function_t call_task;
	task_argument_t function_argument;
	task_return_t treturn;

	// Iterate through registered tasks
	for (int32_t i = ts->current_schedule_slot; i < ts->task_count; i++) 
	{
		uint32_t current_time = time_keeper_get_micros();

		// If the task is active and has waited long enough...
		if ( (ts->tasks[i].run_mode != RUN_NEVER) && (current_time >= ts->tasks[i].next_run) ) 
		{
			uint32_t delay = current_time - (ts->tasks[i].next_run);
			uint32_t task_start_time;

		    task_start_time = time_keeper_get_micros();

		    // Get function pointer and function argument
		    call_task = ts->tasks[i].call_function;
			function_argument = ts->tasks[i].function_argument;

			// Execute task
		    treturn = call_task(function_argument);
	
			// Set the next execution time of the task
			switch (ts->tasks[i].timing_mode) 
			{
				case PERIODIC_ABSOLUTE:
					// Do not take delays into account
					ts->tasks[i].next_run += ts->tasks[i].repeat_period;
				break;

				case PERIODIC_RELATIVE:
					// Take delays into account
					ts->tasks[i].next_run = time_keeper_get_micros() + ts->tasks[i].repeat_period;
				break;
			}
			
			// Set the task to inactive if it has to run only once
			if (ts->tasks[i].run_mode == RUN_ONCE)
			{
				ts->tasks[i].run_mode = RUN_NEVER;
			}

			// Check real time violations
			if (ts->tasks[i].next_run < current_time) 
			{
				realtime_violation = -i; //realtime violation!!
				ts->tasks[i].rt_violations++;
				ts->tasks[i].next_run = current_time + ts->tasks[i].repeat_period;
			}
			
			// Compute real-time statistics
			ts->tasks[i].delay_avg = (7 * ts->tasks[i].delay_avg + delay) / 8;
			if (delay > ts->tasks[i].delay_max) 
			{
				ts->tasks[i].delay_max = delay;
			}
			ts->tasks[i].delay_var_squared = (15 * ts->tasks[i].delay_var_squared + (delay - ts->tasks[i].delay_avg) * (delay - ts->tasks[i].delay_avg)) / 16;
			ts->tasks[i].execution_time = (7 * ts->tasks[i].execution_time + (time_keeper_get_micros() - task_start_time)) / 8;
				
			// Depending on shceduling strategy, select next task slot	
			switch (scheduler->schedule_strategy) 
			{
				case FIXED_PRIORITY: 
					// Fixed priority scheme - scheduler will start over with tasks with the highest priority
					ts->current_schedule_slot = 0;
				break;		
	
				case ROUND_ROBIN:
					// Round robin scheme - scheduler will pick up where it left.
					if (i >= ts->task_count)
					{ 
						ts->current_schedule_slot = 0;
					}
				break;

				default:
				break;
			}

			return realtime_violation;			
		}
	}
	return realtime_violation;
}


task_entry_t* scheduler_get_task_by_id(const scheduler_t* scheduler, uint16_t task_id)
{
	task_set_t* ts = scheduler->task_set;

	for (int32_t i = 0; i < ts->task_count; i++) 
	{
		if ( ts->tasks[i].task_id == task_id )
		{ 
			return &ts->tasks[i];
		}
	}

	return NULL;
}


task_entry_t* scheduler_get_task_by_index(const scheduler_t* scheduler, uint16_t task_index) 
{
	task_set_t* ts = scheduler->task_set;

	if (task_index < ts->task_count) 
	{
		return &ts->tasks[task_index];
	}

	return NULL;
}


void scheduler_change_run_mode(task_entry_t *te, task_run_mode_t new_run_mode) 
{
	te->run_mode = new_run_mode;
}


void scheduler_change_task_period(task_entry_t *te, uint32_t repeat_period) 
{
	te->repeat_period = repeat_period;
	scheduler_change_run_mode(te, RUN_REGULAR);
	scheduler_run_task_now(te);
}


void scheduler_suspend_task(task_entry_t *te, uint32_t delay) 
{
	te->next_run = time_keeper_get_micros() + delay;
}


void scheduler_run_task_now(task_entry_t *te) 
{
	if ((te->run_mode == RUN_NEVER))
	{
		te->run_mode = RUN_ONCE;
	} 

	te->next_run = time_keeper_get_micros();
}