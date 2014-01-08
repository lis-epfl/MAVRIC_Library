/*
 * scheduler.h
 *
 * Created: 12/03/2013 15:51:13
 *  Author: sfx
 */ 


#ifndef SCHEDULER_H_
#define SCHEDULER_H_
//#include <asf.h>
#include "compiler.h"

#define GET_TIME get_micros()
#define SCHEDULER_TIMEBASE 1000000
#define MAX_NUMBER_OF_TASKS 30
#define SCHEDULER_PROFILING

typedef enum task_return_t {
	TASK_RUN_ERROR=-1,
	TASK_RUN_BLOCKED=0,     // if a task returns "TASK_RUN_BLOCKED", the scheduler will try to re-run at the next schedule update, and not update "next_run" 
	TASK_RUN_SUCCESS=1	
}task_return_t;

typedef enum task_run_mode_t {RUN_NEVER, RUN_ONCE, RUN_REGULAR} task_run_mode_t;
typedef enum task_timing_mode_t {PERIODIC_ABSOLUTE, PERIODIC_RELATIVE} task_timing_mode_t;
enum schedule_strategy_t {ROUND_ROBIN, FIXED_PRIORITY};

typedef task_return_t (*function_pointer)(void);

typedef uint8_t task_handle_t;


typedef struct {	
	struct task_set *tasks;	
	function_pointer call_function;
	uint16_t task_id;
	task_run_mode_t  run_mode;		
	task_timing_mode_t timing_mode;
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
	uint8_t number_of_tasks;
	int running_task;
	int current_schedule_slot;
	task_entry tasks[MAX_NUMBER_OF_TASKS];
} task_set;

//#define NEW_TASK_SET(NAME,NUMBER) struct task_set {const task_handle_t number_of_tasks; int running_task; int current_schedule_slot; task_entry tasks[NUMBER];} NAME = {.number_of_tasks=NUMBER}; 

#define NEW_TASK_SET(NAME,NUMBER) task_set NAME = {.number_of_tasks=MAX_NUMBER_OF_TASKS};

void init_scheduler(task_set *ts);

task_handle_t register_task(task_set *ts, int task_slot, unsigned long repeat_period, task_run_mode_t run_mode, function_pointer call_function);

bool add_task(task_set *ts, unsigned long repeat_period, task_run_mode_t run_mode, function_pointer call_function, uint32_t task_id);
void sort_taskset_by_period(task_set *ts);


int run_scheduler_update(task_set *ts, uint8_t schedule_strategy);

task_entry* get_task_by_id(task_set *ts, uint16_t task_id);
task_entry* get_task_by_index(task_set *ts, uint16_t task_index);

void change_run_mode(task_entry *te, task_run_mode_t new_run_mode);
void change_task_period(task_entry *te, unsigned long repeat_period);

void suspend_task(task_entry *te, unsigned long delay);

void run_task_now(task_entry *te);

#endif /* SCHEDULER_H_ */