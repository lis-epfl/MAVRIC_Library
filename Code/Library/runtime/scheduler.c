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

task_handle_t register_task(task_set *ts, int task_slot, unsigned long repeat_period, task_run_mode_t run_mode, function_pointer *call_function) {
	if ((task_slot<0) || (task_slot>=ts->number_of_tasks)) {
		return -1;
	}
	ts->tasks[task_slot].task_id=task_slot;
	ts->tasks[task_slot].call_function=call_function;
	ts->tasks[task_slot].run_mode=run_mode;
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

bool add_task(task_set *ts, unsigned long repeat_period, task_run_mode_t run_mode, function_pointer *call_function, uint32_t task_id) {
	int task_slot=0;
	while ((task_slot < ts->number_of_tasks) && (ts->tasks[task_slot].call_function!=NULL)) task_slot++;
	if (task_slot >= ts->number_of_tasks) return false;
	register_task(ts,  task_slot,   repeat_period, run_mode,  call_function);
	ts->tasks[task_slot].task_id=task_id;
	return true;
}


void sort_taskset_by_period(task_set *ts){
	int i;
	bool sorted=false;
	task_entry tmp;
	if (ts->number_of_tasks<2) return;

	while (!sorted) {
		sorted=true;
		for (i=0; i<ts->number_of_tasks-1; i++) {
			if ( ((ts->tasks[i].call_function==NULL)&&(ts->tasks[i+1].call_function!=NULL)) ||
				((ts->tasks[i].call_function!=NULL)&&(ts->tasks[i+1].call_function!=NULL) &&
				(ts->tasks[i].repeat_period > ts->tasks[i+1].repeat_period))) {
				tmp=ts->tasks[i];
				ts->tasks[i]=ts->tasks[i+1];
				ts->tasks[i+1]=tmp;		
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
		if ((ts->tasks[i].call_function!=NULL)  && (ts->tasks[i].run_mode!=RUN_NEVER) &&(current_time >= ts->tasks[i].next_run)) {
			uint32_t delay=current_time - (ts->tasks[i].next_run);
			uint32_t task_start_time;
			ts->tasks[i].delay_avg= (15*ts->tasks[i].delay_avg + delay)/16;
			if (delay>ts->tasks[i].delay_max) ts->tasks[i].delay_max=delay;
			
			ts->tasks[i].delay_var_squared=(15*ts->tasks[i].delay_var_squared+(delay - ts->tasks[i].delay_avg)*(delay - ts->tasks[i].delay_avg))/16;
			
			ts->tasks[i].next_run += ts->tasks[i].repeat_period;
			if (ts->tasks[i].run_mode==RUN_ONCE) ts->tasks[i].run_mode=RUN_NEVER;
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


task_entry* get_task_by_id(task_set *ts, uint16_t task_id){
	int i=0;
	for (i=0; i<ts->number_of_tasks; i++) {
		if (ts->tasks[i].task_id==task_id) return &ts->tasks[i];
	}
	return NULL;
}

void change_run_mode(task_entry *te, task_run_mode_t new_run_mode) {
	te->run_mode=new_run_mode;
}

void change_task_period(task_entry *te, unsigned long repeat_period) {
	te->repeat_period=repeat_period;
	change_run_mode(te, RUN_REGULAR);
	run_task_now(te);
}

void suspend_task(task_entry *te, unsigned long delay) {
	te->next_run=GET_TIME + delay;
}

void run_task_now(task_entry *te) {
	if (te->run_mode==RUN_NEVER) te->run_mode=RUN_ONCE;
	te->next_run=GET_TIME;
}

