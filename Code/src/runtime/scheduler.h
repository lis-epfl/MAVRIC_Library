/*
 * scheduler.h
 *
 * Created: 12/03/2013 15:51:13
 *  Author: sfx
 */ 


#ifndef SCHEDULER_H_
#define SCHEDULER_H_
#include <asf.h>
#define GET_TIME get_micros()

#define SCHEDULER_PROFILING

typedef enum task_return_t {
	TASK_RUN_ERROR=-1,
	TASK_RUN_BLOCKED=0,     // if a task returns "TASK_RUN_BLOCKED", the scheduler will try to re-run at the next schedule update, and not update "next_run" 
	TASK_RUN_SUCCESS=1	
}task_return_t;

typedef task_return_t (function_pointer)(void *param_object);

typedef uint8_t task_handle_t;


typedef struct {
	struct task_set *tasks;	
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

typedef struct  {
	task_handle_t number_of_tasks;
	task_entry tasks[];
} task_set;


#define NEW_TASK_SET(NAME,NUMBER) task_entry NAME_entries[NUMBER]; task_set NAME={NUMBER, &NAME_entries}; 

void init_scheduler(task_set *ts);

task_handle_t register_task(task_set *ts, int task_slot, unsigned long repeat_period, function_pointer *call_function, void *param_object );



int run_scheduler_update(task_set *ts);



#endif /* SCHEDULER_H_ */