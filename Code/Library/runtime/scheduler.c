/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file scheduler.c
 *
 * Scheduler
 */


#include "scheduler.h"
#include "time_keeper.h"

// TODO: change behaviour of task_set_t.number_of_tasks (ie: initialised at 0 and incremented each time a task is added)
// currently it is dangerous because the task_set_t can be iterated between 0 and number_of_task even if 
// number_of_task is superior to the actual number of tasks stored in the structure (risk of empty pointers)

void scheduler_init(task_set_t *ts) 
{
	int32_t i;
	
	for (i = 0; i < ts->number_of_tasks; i++) 
	{
		ts->tasks[i].call_function = NULL;
		ts->tasks[i].tasks = ts;
	}

	ts->running_task = -1;
	ts->current_schedule_slot = 0;
}


task_handle_t scheduler_register_task(task_set_t *ts, int32_t task_slot, uint32_t repeat_period, task_run_mode_t run_mode, task_function_t call_function, void* function_argument) 
{
	if ((task_slot < 0) || (task_slot >= ts->number_of_tasks)) 
	{
		return -1;
	}

	ts->tasks[task_slot].task_id = task_slot;
	ts->tasks[task_slot].call_function = call_function;
	ts->tasks[task_slot].function_argument = function_argument;
	ts->tasks[task_slot].run_mode = run_mode;
	ts->tasks[task_slot].repeat_period = repeat_period;
	ts->tasks[task_slot].next_run = GET_TIME;
	ts->tasks[task_slot].execution_time = 0;
	ts->tasks[task_slot].timing_mode = PERIODIC_ABSOLUTE;

#ifdef SCHEDULER_PROFILING	
	ts->tasks[task_slot].delay_max = 0;
	ts->tasks[task_slot].delay_avg = 0;
	ts->tasks[task_slot].delay_var_squared = 0;
	ts->tasks[task_slot].rt_violations = 0;
#endif

	return task_slot;
}


bool scheduler_add_task(task_set_t *ts, uint32_t repeat_period, task_run_mode_t run_mode, task_function_t call_function, void* function_argument, uint32_t task_id) 
{
	int32_t task_slot = 0;
	
	while ((task_slot < ts->number_of_tasks) && (ts->tasks[task_slot].call_function != NULL)) 
	{
		task_slot++;
	}

	if (task_slot >= ts->number_of_tasks) 
	{
		return false;
	}

	scheduler_register_task(ts,  task_slot,   repeat_period, run_mode,  call_function, function_argument);
	ts->tasks[task_slot].task_id = task_id;

	return true;
}


void scheduler_sort_taskset_by_period(task_set_t *ts){
	int32_t i;
	bool sorted = false;
	task_entry_t tmp;
	if (ts->number_of_tasks < 2) 
	{
		return;
	}

	while (!sorted) 
	{
		sorted = true;
		for (i = 0; i < ts->number_of_tasks - 1; i++) 
		{
			if ( ((ts->tasks[i].call_function == NULL) && (ts->tasks[i + 1].call_function != NULL)) ||
				((ts->tasks[i].call_function != NULL)&&(ts->tasks[i + 1].call_function != NULL) &&
				(ts->tasks[i].repeat_period > ts->tasks[i + 1].repeat_period))) 
			{
				tmp = ts->tasks[i];
				ts->tasks[i] = ts->tasks[i + 1];
				ts->tasks[i + 1] = tmp;		
				sorted = false;
			}
		}
	}		
}


int32_t scheduler_update(task_set_t *ts, uint8_t schedule_strategy) 
{
	int32_t i;
	int32_t realtime_violation = 0;
	volatile task_function_t call_task;
	void* function_argument;
	task_return_t treturn;

	for (i = ts->current_schedule_slot; i < ts->number_of_tasks; i++) 
	{
		uint32_t current_time = GET_TIME;
		if ((ts->tasks[i].call_function != NULL) && (ts->tasks[i].run_mode != RUN_NEVER) && (current_time >= ts->tasks[i].next_run)) 
		{
			uint32_t delay = current_time - (ts->tasks[i].next_run);
			uint32_t task_start_time;

		    task_start_time = GET_TIME;
		    call_task = ts->tasks[i].call_function;
			function_argument = ts->tasks[i].function_argument;
		    treturn = call_task(function_argument);
	
			switch (ts->tasks[i].timing_mode) 
			{
				case PERIODIC_ABSOLUTE:
					ts->tasks[i].next_run += ts->tasks[i].repeat_period;
				break;

				case PERIODIC_RELATIVE:
					ts->tasks[i].next_run = GET_TIME + ts->tasks[i].repeat_period;
				break;
			}
			
			if (ts->tasks[i].run_mode == RUN_ONCE)
			{
				ts->tasks[i].run_mode = RUN_NEVER;
			}

			if (ts->tasks[i].next_run < current_time) 
			{
				realtime_violation = -i; //realtime violation!!
				ts->tasks[i].rt_violations++;
				ts->tasks[i].next_run = current_time + ts->tasks[i].repeat_period;
			}
			
			ts->tasks[i].delay_avg = (7 * ts->tasks[i].delay_avg + delay) / 8;
			if (delay > ts->tasks[i].delay_max) 
			{
				ts->tasks[i].delay_max = delay;
			}
			
			ts->tasks[i].delay_var_squared = (15 * ts->tasks[i].delay_var_squared + (delay - ts->tasks[i].delay_avg) * (delay - ts->tasks[i].delay_avg)) / 16;
			
			ts->tasks[i].execution_time = (7 * ts->tasks[i].execution_time + (GET_TIME - task_start_time)) / 8;
						
			switch (schedule_strategy) 
			{
				case FIXED_PRIORITY: 
					ts->current_schedule_slot = 0;
				break;		
	
				case ROUND_ROBIN:
					// round robin scheme - scheduler will pick up where it left.
					if (i == ts->number_of_tasks)
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
	return realtime_violation;;
}


task_entry_t* scheduler_get_task_by_id(task_set_t *ts, uint16_t task_id)
{
	int32_t i = 0;
	for (i = 0; i < ts->number_of_tasks; i++) 
	{
		if (ts->tasks[i].task_id == task_id)
		{ 
			return &ts->tasks[i];
		}
	}

	return NULL;
}


task_entry_t* scheduler_get_task_by_index(task_set_t *ts, uint16_t task_index) 
{
	if (task_index < ts->number_of_tasks) 
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
	te->next_run = GET_TIME + delay;
}


void scheduler_run_task_now(task_entry_t *te) 
{
	if ((te->run_mode == RUN_NEVER))
	{
		te->run_mode = RUN_ONCE;
	} 

	te->next_run = GET_TIME;
}

