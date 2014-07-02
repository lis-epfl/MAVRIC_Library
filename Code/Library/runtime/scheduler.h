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


#include "compiler.h"

#define GET_TIME get_micros()
#define SCHEDULER_TIMEBASE 1000000
#define MAX_NUMBER_OF_TASKS 30
#define SCHEDULER_PROFILING


typedef uint8_t task_handle_t;


/**
 * \brief	Task return code
 */
typedef enum task_return_t 
{
	TASK_RUN_ERROR=-1,		///< 	The task was not not successfully executed
	TASK_RUN_BLOCKED=0,     ///< 	If a task returns "TASK_RUN_BLOCKED", the scheduler will try to re-run at the next schedule update, and not update "next_run" 
	TASK_RUN_SUCCESS=1		///< 	The task was successfully executed 
} task_return_t;


typedef task_return_t (*function_pointer)(void);


/**
 * \brief 	Task run mode
 */
typedef enum task_run_mode_t 
{
	RUN_NEVER, 			///<	The task will not be executed
	RUN_ONCE, 			///<	The task will be executed only once
	RUN_REGULAR			///<	The task will be executed periodically
} task_run_mode_t;


/**
 * \brief 	Task timing mode
 */
typedef enum task_timing_mode_t 
{
	PERIODIC_ABSOLUTE, 	///<	The task will be executed accorded to a fixed schedule (ie. regardless of past delays or realtime violations)	
	PERIODIC_RELATIVE	///<	The task will be executed with constant period relative to the last execution
} task_timing_mode_t;


/**
 * \brief 	Scheduler strategy
 */
enum schedule_strategy_t 
{
	ROUND_ROBIN,	///<	Round robin scheduling
	FIXED_PRIORITY	///<	Fixed priority scheduling
};


/**
 * \brief 	Task entry
 */
typedef struct task_entry 
{	
	struct task_set *tasks;				///<	Pointer to the task set to which this task belongs 
	function_pointer call_function;		///<	Function to be called
	uint16_t task_id;					///<	Unique task identifier
	task_run_mode_t  run_mode;			///<	Run mode
	task_timing_mode_t timing_mode;		///<	Timing mode
	uint32_t repeat_period;   			///<	Period between two calls (us)
	uint32_t next_run;					///<	Next execution time
	uint32_t execution_time;			///<	Execution time
#ifdef SCHEDULER_PROFILING		
	uint32_t delay_max;					///<	Maximum delay between expected execution and actual execution
	uint32_t delay_avg;					///<	Average delay between expected execution and actual execution
	uint32_t delay_var_squared;			///<	Standard deviation of the delay
	uint32_t rt_violations;				///<	Number of Real-time violations, this is incremented each time an execution is skipped
#endif
} task_entry;


/**
 * \brief 	Task set
 */
typedef struct task_set 
{
	uint8_t number_of_tasks;		///<	Number_of_tasks
	int running_task;				///<	ID of the task being executed
	int current_schedule_slot;		///<	Slot of the task being executed
	struct task_entry tasks[30];	///<	Array of tasks_entry to be executed
} task_set;


/**
 * @brief Macro to instantiate a new task-set
 * 
 * @param NAME 		Name of the task-set
 * @param NUMBER 	Number of tasks in the task-set
 */
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