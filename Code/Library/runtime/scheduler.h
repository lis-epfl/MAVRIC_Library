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

typedef task_return_t (*function_pointer)();

typedef uint8_t task_handle_t;


typedef struct {
	struct task_set *tasks;	
	function_pointer call_function;
	
	uint32_t repeat_period;
	uint32_t next_run;
	uint32_t execution_time;
	
#ifdef SCHEDULER_PROFILING	
	uint32_t delay_max;
	uint32_t delay_avg;
	uint32_t delay_var_squared;
	uint32_t rt_violations;
#endif
} task_entry;

typedef struct  {
	task_handle_t number_of_tasks;
	task_entry tasks[];
} task_set;

#define NEW_TASK_SET(NAME,NUMBER) struct task_set {const task_handle_t number_of_tasks; task_entry tasks[NUMBER];} NAME = {.number_of_tasks=NUMBER}; 

void init_scheduler(task_set *ts);

task_handle_t register_task(task_set *ts, int task_slot, unsigned long repeat_period, function_pointer *call_function);

bool add_task(task_set *ts, unsigned long repeat_period, function_pointer *call_function);
void sort_taskset_by_period(task_set *ts);

enum schedule_strategy_t {ROUND_ROBIN, FIXED_PRIORITY};

int run_scheduler_update(task_set *ts, uint8_t schedule_strategy);



#endif /* SCHEDULER_H_ */