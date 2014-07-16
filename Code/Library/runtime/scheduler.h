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
 * \file scheduler.h
 *
 * Scheduler
 */


#ifndef SCHEDULER_H_
#define SCHEDULER_H_


#ifdef __cplusplus
extern "C" 
{
#endif

#include "stdint.h"
#include <stdbool.h>

#define GET_TIME time_keeper_get_micros()
#define SCHEDULER_TIMEBASE 1000000
#define MAX_NUMBER_OF_TASKS 30


typedef uint8_t task_handle_t;

// TODO: update documentation


/**
 * \brief	Task return code
 */
typedef enum 
{
	TASK_RUN_ERROR 	 = -1,		///< 	The task was not not successfully executed
	TASK_RUN_BLOCKED = 0,     	///< 	If a task returns "TASK_RUN_BLOCKED", the scheduler will try to re-run at the next schedule update, and not update "next_run" 
	TASK_RUN_SUCCESS = 1		///< 	The task was successfully executed 
} task_return_t;


typedef void* task_argument_t;
typedef task_return_t (*task_function_t)(task_argument_t);


/**
 * \brief 	Task run mode
 */
typedef enum 
{
	RUN_NEVER, 			///<	The task will not be executed
	RUN_ONCE, 			///<	The task will be executed only once
	RUN_REGULAR			///<	The task will be executed periodically
} task_run_mode_t;


/**
 * \brief 	Task timing mode
 */
typedef enum 
{
	PERIODIC_ABSOLUTE, 	///<	The task will be executed accorded to a fixed schedule (ie. regardless of past delays or realtime violations)	
	PERIODIC_RELATIVE	///<	The task will be executed with constant period relative to the last execution
} task_timing_mode_t;


/**
 * \brief 	Scheduler strategy
 */
typedef enum  
{
	ROUND_ROBIN,		///<	Round robin scheduling
	FIXED_PRIORITY		///<	Fixed priority scheduling
} schedule_strategy_t;


/**
 * \brief 	Task entry
 */
typedef struct
{	
	task_function_t call_function;		///<	Function to be called
	task_argument_t function_argument;
	uint16_t task_id;					///<	Unique task identifier
	task_run_mode_t  run_mode;			///<	Run mode
	task_timing_mode_t timing_mode;		///<	Timing mode
	uint32_t repeat_period;   			///<	Period between two calls (us)
	uint32_t next_run;					///<	Next execution time
	uint32_t execution_time;			///<	Execution time
	uint32_t delay_max;					///<	Maximum delay between expected execution and actual execution
	uint32_t delay_avg;					///<	Average delay between expected execution and actual execution
	uint32_t delay_var_squared;			///<	Standard deviation of the delay
	uint32_t rt_violations;				///<	Number of Real-time violations, this is incremented each time an execution is skipped
} task_entry_t;


/**
 * \brief 	Task set
 */
typedef struct task_set_t
{
	uint32_t task_count;				///<	Number_of_tasks
	uint32_t max_task_count;			///<	Maximum number of tasks
	int32_t current_schedule_slot;		///<	Slot of the task being executed
	task_entry_t tasks[];				///<	Array of tasks_entry to be executed
} task_set_t;



typedef struct
{
	bool debug;
	task_set_t* task_set;
} scheduler_t;



typedef struct
{
	uint32_t max_task_count;
	bool debug;
} scheduler_conf_t;


/**
 * @brief Macro to instantiate a new task-set
 * 
 * @param NAME 		Name of the task-set
 * @param NUMBER 	Number of tasks in the task-set
 */
#define NEW_TASK_SET(NAME,NUMBER) task_set_t NAME = {.task_count = MAX_NUMBER_OF_TASKS};


/**
 * \brief 		Scheduler module initialisation
 * 
 * \param ts 	Pointer to task set
 */
void scheduler_init( scheduler_t* scheduler, const scheduler_conf_t* config); // task_set_t *ts);


/**
 * \brief 					Register a new task to the task set, in a precise slot
 * 
 * \param	ts 				Pointer to task set
 * \param 	task_slot 		Slot in which the new task should be inserted
 * \param 	repeat_period 	Repeat period (us)
 * \param 	run_mode 		Run mode
 * \param 	call_function 	Function pointer to be called
 * 
 * \return 	Task handle
 */
// task_handle_t scheduler_register_task( scheduler_t* scheduler, int32_t task_slot, uint32_t repeat_period, task_run_mode_t run_mode, task_function_t call_function, task_argument_t function_argument);
// task_handle_t scheduler_register_task( task_set_t *ts, int32_t task_slot, uint32_t repeat_period, task_run_mode_t run_mode, task_function_t call_function, task_argument_t function_argument);


/**
 * \brief               	Register a new task to the task set, in the first available slot
 * 
 * \param ts            	Pointer to task set
 * \param repeat_period     Repeat period (us)
 * \param run_mode      	Run mode
 * \param call_function 	Function pointer to be called
 * \param task_id       	Unique task identifier
 * 
 * \return              	True if the task was successfully added, False if not
 */
bool scheduler_add_task(scheduler_t* scheduler, uint32_t repeat_period, task_run_mode_t run_mode, task_timing_mode_t timing_mode, task_function_t call_function, task_argument_t function_argument, uint32_t task_id);
// bool scheduler_add_task(task_set_t *ts, uint32_t repeat_period, task_run_mode_t run_mode, task_function_t call_function, task_argument_t function_argument, uint32_t task_id);


/**
* \brief  	 Sort tasks by repeat perios
*
* \param ts  Pointer to task set
*/
void scheduler_sort_taskset_by_period(scheduler_t* scheduler);
// void scheduler_sort_taskset_by_period(task_set_t *ts);


/**
 * \brief                   	Run update (check for tasks ready for execution and execute them)
 * 
 * \param ts                	Pointer to task set
 * \param schedule_strategy 	Scheduler strategy
 * 
 * \return                  	Number of realtime violations
 */
int32_t scheduler_update(scheduler_t* scheduler, uint8_t schedule_strategy);
// int32_t scheduler_update(task_set_t *ts, uint8_t schedule_strategy);


/**
 * \brief         	Find a task according to its idea
 * 
 * \param ts      	Pointer to the task set
 * \param task_id 	ID of the target task
 * 
 * \return        	Pointer to the target task
 */
task_entry_t* scheduler_get_task_by_id(scheduler_t* scheduler, uint16_t task_id);
// task_entry_t* scheduler_get_task_by_id(task_set_t *ts, uint16_t task_id);


/**
 * \brief            	Find a task according to its index
 * 
 * \param ts         	Pointer to the task set
 * \param task_index 	Index of the target task
 * 
 * \return           	Pointer to the target task
 */
task_entry_t* scheduler_get_task_by_index(scheduler_t* scheduler, uint16_t task_index);
// task_entry_t* scheduler_get_task_by_index(task_set_t *ts, uint16_t task_index);


/**
 * \brief              	Modifies the run mode of an existing task
 * 
 * \param te           	Pointer to a task entry
 * \param new_run_mode 	New run mode
 */
void scheduler_change_run_mode(task_entry_t *te, task_run_mode_t new_run_mode);


/**
 * \brief      				Modifies the period of execution of an existing task
 * 
 * \param te   				Pointer to a task entry
 * \param repeat_period 	New repeat period (us)
 */
void scheduler_change_task_period(task_entry_t *te, uint32_t repeat_period);


/**
 * \brief      		Suspends a task
 * 
 * \param te   		Pointer to a task entry
 * \param delay 	Duration (us)
 */
// void scheduler_suspend_task(scheduler_t* scheduler, uint32_t delay);
void scheduler_suspend_task(task_entry_t *te, uint32_t delay);


/**
 * \brief    	Run a task immediately
 * 
 * \param te 	Pointer to a task entry
 */
// void scheduler_run_task_now(scheduler_t* scheduler);
void scheduler_run_task_now(task_entry_t *te);


#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H_ */