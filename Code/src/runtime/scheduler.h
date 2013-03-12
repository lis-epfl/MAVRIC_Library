/*
 * scheduler.h
 *
 * Created: 12/03/2013 15:51:13
 *  Author: sfx
 */ 


#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#define GET_TIME get_micros()

#define SCHEDULER_PROFILING


typedef void (function_pointer)(void *param_object);

typedef struct {
	function_pointer *call_function;
	void *param_object;
	unsigned int repeat_period;
	unsigned long next_run;
	unsigned int execution_time;
	
#ifdef SCHEDULER_PROFILING	
	unsigned int delay_max;
	unsigned int delay_avg;
	unsigned int delay_var;
#endif
} task_entry;

typedef struct {
	int number_of_tasks;
	task_entry tasks[];
} task_set;

#define NEW_TASK_SET(NAME,NUMBER) task_entry NAME_entries[NUMBER]; task_set NAME={NUMBER, &NAME_entries}; 

void init_scheduler(task_set *ts);

int register_task(task_set *ts, int task_slot, unsigned long repeat_period, function_pointer *call_function, void *param_object );
int register_task_after(task_set *ts, int task_slot, function_pointer *call_function, int after_task_slot);

int run_scheduler_update(task_set *ts);



#endif /* SCHEDULER_H_ */