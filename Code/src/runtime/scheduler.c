/*
 * scheduler.c
 *
 * Created: 12/03/2013 15:51:22
 *  Author: sfx
 */ 


#include "scheduler.h"
#include "time_keeper.h"

//task_entry tasks[MAX_NUMBER_OF_TASKS];


void init_scheduler(task_set *ts) {
	int i;
	for (i=0; i<ts->number_of_tasks; i++) {
		ts->tasks[i].call_function=NULL;
		ts->tasks[i].param_object=NULL;
		ts->tasks[i].tasks=ts;
	}
}

task_handle_t register_task(task_set *ts, int task_slot, unsigned long repeat_period, function_pointer *call_function, void *param_object ) {
	if ((task_slot<0) || (task_slot>=ts->number_of_tasks)) {
		return -1;
	}
	
	ts->tasks[task_slot].call_function=call_function;
	ts->tasks[task_slot].param_object=param_object;
	ts->tasks[task_slot].repeat_period=repeat_period;
	ts->tasks[task_slot].next_run=GET_TIME;
	ts->tasks[task_slot].execution_time=0;
#ifdef SCHEDULER_PROFILING	
	ts->tasks[task_slot].delay_max=0;
	ts->tasks[task_slot].delay_avg=0;
	ts->tasks[task_slot].delay_var=0;
#endif
}



int run_scheduler_update(task_set *ts) {
	int i;
	int realtime_violation=0;
	for (i=0; i<ts->number_of_tasks; i++) {
		if ((ts->tasks[i].call_function!=NULL) && (GET_TIME >= ts->tasks[i].next_run)) {
			ts->tasks[i].next_run+=ts->tasks[i].repeat_period;
			ts->tasks[i].call_function(ts->tasks[i].param_object);
			if (ts->tasks[i].next_run<GET_TIME) realtime_violation=-i; //realtime violation!!
		}
	}
	return realtime_violation;
	
}


