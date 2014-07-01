/**
* Scheduler
*
* The MAV'RIC Framework
* Copyright © 2011-2014
*
* Laboratory of Intelligent Systems, EPFL
*
* This file is part of the MAV'RIC Framework.
*/


#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#ifdef __cplusplus
extern "C" 
{
#endif

#include "compiler.h"

#define GET_TIME get_micros()
#define SCHEDULER_TIMEBASE 1000000
#define MAX_NUMBER_OF_TASKS 30
#define SCHEDULER_PROFILING



typedef uint8_t task_handle_t;


/**
 * \brief	Task return code
 * 
 * \param	TASK_RUN_ERROR		The task was not not successfully executed
 * \param	TASK_RUN_BLOCKED	If a task returns "TASK_RUN_BLOCKED", the scheduler will try to re-run at the next schedule update, and not update "next_run" 
 * \param	TASK_RUN_SUCCESS	The task was successfully executed 
 * 
 */
typedef enum task_return_t 
{
	TASK_RUN_ERROR=-1,
	TASK_RUN_BLOCKED=0,     
	TASK_RUN_SUCCESS=1	
} task_return_t;


typedef task_return_t (*function_pointer)(void);


/**
 * \brief 	Task run mode
 * 
 * \param 	RUN_NEVER	
 * \param 	RUN_ONCE  
 * \param 	RUN_REGULAR 
 * 
 */
typedef enum task_run_mode_t 
{
	RUN_NEVER, 
	RUN_ONCE, 
	RUN_REGULAR
} task_run_mode_t;


/**
 * \brief 	Task timing mode
 * 
 * \param 	PERIODIC_ABSOLUTE
 * \param 	PERIODIC_RELATIVE
 * 
 */
typedef enum task_timing_mode_t 
{
	PERIODIC_ABSOLUTE, 
	PERIODIC_RELATIVE
} task_timing_mode_t;


/**
 * \brief 	Scheduler strategy
 * 
 * \param 	ROUND_ROBIN
 * \param 	FIXED_PRIORITY
 * 
 */
enum schedule_strategy_t 
{
	ROUND_ROBIN,
	FIXED_PRIORITY
};


/**
 * \brief Task entry
 * 
 * \param task_set	 		Pointer to the task set to which this task belongs 
 * \param function_pointer	Function to be called
 * \param task_id	 		Unique task identifier
 * \param run_mode			Run mode
 * \param timing_mode		Timing mode
 * \param repeat_period		Period between two calls (us)
 * \param next_run			Next execution time
 * \param execution_time	Last execution time
 * \param delay_max			Maximum delay between expected execution and actual execution
 * \param delay_avg			Average delay between expected execution and actual execution
 * \param delay_var_squared	Standard deviation of the delay
 * \param rt_violations		Number of Real-time violations, this is incremented each time an execution is skipped
 * 
 */
typedef struct task_entry 
{	
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


/**
 * \brief 						Task set
 * 
 * \param 						Number_of_tasks
 * \param running_task 			ID of the task being executed
 * \param current_schedule_slot 
 * \param tasks  				Array of tasks_entry to be executed
 */
typedef struct task_set 
{
	uint8_t number_of_tasks;
	int running_task;
	int current_schedule_slot;
	struct task_entry tasks[30];
} task_set;


#define NEW_TASK_SET(NAME,NUMBER) task_set NAME = {.number_of_tasks=MAX_NUMBER_OF_TASKS};

/**
 * \brief 		Scheduler module initialisation
 * 
 * \param ts 	Pointer to task set
 */
void init_scheduler(task_set *ts);


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
task_handle_t register_task(task_set *ts, int task_slot, unsigned long repeat_period, task_run_mode_t run_mode, function_pointer call_function);


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
bool add_task(task_set *ts, unsigned long repeat_period, task_run_mode_t run_mode, function_pointer call_function, uint32_t task_id);


/**
* \brief  	 Sort tasks by repeat perios
*
* \param ts  Pointer to task set
*/
void sort_taskset_by_period(task_set *ts);


/**
 * \brief                   	Run update (check for tasks ready for execution and execute them)
 * 
 * \param ts                	Pointer to task set
 * \param schedule_strategy 	Scheduler strategy
 * 
 * \return                  	Number of realtime violations
 */
int run_scheduler_update(task_set *ts, uint8_t schedule_strategy);


/**
 * \brief         	Find a task according to its idea
 * 
 * \param ts      	Pointer to the task set
 * \param task_id 	ID of the target task
 * 
 * \return        	Pointer to the target task
 */
task_entry* get_task_by_id(task_set *ts, uint16_t task_id);


/**
 * \brief            	Find a task according to its index
 * 
 * \param ts         	Pointer to the task set
 * \param task_index 	Index of the target task
 * 
 * \return           	Pointer to the target task
 */
task_entry* get_task_by_index(task_set *ts, uint16_t task_index);


/**
 * \brief              	Modifies the run mode of an existing task
 * 
 * \param te           	Pointer to a task entry
 * \param new_run_mode 	New run mode
 */
void change_run_mode(task_entry *te, task_run_mode_t new_run_mode);


/**
 * \brief      				Modifies the period of execution of an existing task
 * 
 * \param te   				Pointer to a task entry
 * \param repeat_period 	New repeat period (us)
 */
void change_task_period(task_entry *te, unsigned long repeat_period);


/**
 * \brief      		Suspends a task
 * 
 * \param te   		Pointer to a task entry
 * \param delay 	Duration (us)
 */
void suspend_task(task_entry *te, unsigned long delay);


/**
 * \brief    	Run a task immediately
 * 
 * \param te 	Pointer to a task entry
 */
void run_task_now(task_entry *te);


#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H_ */