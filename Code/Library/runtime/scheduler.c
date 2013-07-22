/*
 * scheduler.c
 *
 * Created: 12/03/2013 15:51:22
 *  Author: sfx
 */ 


#include "scheduler.h"
#include "time_keeper.h"

void init_scheduler(task_set *ts) {
	int i;
	for (i=0; i<ts->number_of_tasks; i++) {
		ts->tasks[i].call_function=NULL;
		ts->tasks[i].tasks=ts;
	}
}

task_handle_t register_task(task_set *ts, int task_slot, unsigned long repeat_period, function_pointer *call_function) {
	if ((task_slot<0) || (task_slot>=ts->number_of_tasks)) {
		return -1;
	}
	
	ts->tasks[task_slot].call_function=call_function;
	
	ts->tasks[task_slot].repeat_period=repeat_period;
	ts->tasks[task_slot].next_run=GET_TIME;
	ts->tasks[task_slot].execution_time=0;
#ifdef SCHEDULER_PROFILING	
	ts->tasks[task_slot].delay_max=0;
	ts->tasks[task_slot].delay_avg=0;
	ts->tasks[task_slot].delay_var_squared=0;
	ts->tasks[task_slot].rt_violations=0;
#endif
	return task_slot;
}

bool add_task(task_set *ts, unsigned long repeat_period, function_pointer *call_function) {
	int task_slot=0;
	while ((task_slot < ts->number_of_tasks) && (ts->tasks[task_slot].call_function!=NULL)) task_slot++;
	if (task_slot < ts->number_of_tasks) return -1;
	register_task(ts,  task_slot,   repeat_period, call_function);
}


void sort_taskset_by_period(task_set *ts){
	int i, sorted_above;
	bool sorted=false;
	task_entry tmp;
	if (ts->number_of_tasks<2) return;
	sorted_above=ts->number_of_tasks-1;
	while (!sorted) {
		sorted=true;
		for (i=0; i<sorted_above; i++) {
			if ( ((ts->tasks[i].call_function==NULL)&&(ts->tasks[i+1].call_function!=NULL)) ||(ts->tasks[i].repeat_period>ts->tasks[i+1].repeat_period)) {
				tmp=ts->tasks[i];
				ts->tasks[i]=ts->tasks[i+1];
				ts->tasks[i+1]=tmp;		
				sorted_above=i;
				sorted=false;
			}
		}
	}		
}

int run_scheduler_update(task_set *ts, uint8_t schedule_strategy) {
	int i;
	int realtime_violation=0;
	function_pointer call_task;
	task_return_t treturn;
	for (i=0; i<ts->number_of_tasks; i++) {
		uint32_t current_time=GET_TIME;
		if ((ts->tasks[i].call_function!=NULL) && (current_time >= ts->tasks[i].next_run)) {
			uint32_t delay=current_time - (ts->tasks[i].next_run);
			uint32_t task_start_time;
			ts->tasks[i].delay_avg= (15*ts->tasks[i].delay_avg + delay)/16;
			if (delay>ts->tasks[i].delay_max) ts->tasks[i].delay_max=delay;
			
			ts->tasks[i].delay_var_squared=(15*ts->tasks[i].delay_var_squared+(delay - ts->tasks[i].delay_avg)*(delay - ts->tasks[i].delay_avg))/16;
			
			ts->tasks[i].next_run += ts->tasks[i].repeat_period;
			if (ts->tasks[i].next_run < current_time) {
				realtime_violation=-i; //realtime violation!!
				ts->tasks[i].rt_violations++;
				ts->tasks[i].next_run=current_time + ts->tasks[i].repeat_period;
			}
			
			
		    task_start_time=GET_TIME;
			call_task=ts->tasks[i].call_function;
			
			treturn = call_task();
			
			ts->tasks[i].execution_time= (7*ts->tasks[i].execution_time + (GET_TIME-task_start_time))/8;
			
			
			if (schedule_strategy==FIXED_PRIORITY) return realtime_violation;					
		}
	}
	return realtime_violation;;
}


