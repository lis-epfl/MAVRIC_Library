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
 * \file tasks.h
 *
 * Definition of the tasks executed on the autopilot
 */ 


#ifndef TASKS_H_
#define TASKS_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "scheduler.h"
#include "mavlink_actions.h"
#include "radar_module_driver.h"


/**
 * \brief 			Initialises all the tasks
 */	
void tasks_create_tasks(void);


/**
 * \brief          	Returns a pointer to the main task-set  
 * 
 * \return          Pointer to the main task-set
 */
task_set_t* tasks_get_main_taskset(void);


/**
 * \brief            Updates the IMU
 */
void tasks_run_imu_update(void* arg);


/**
 * \brief            	This function does bullshit
 * \details  			1) Switch on/off collision avoidance
 * 						2) Switch on/off the motor
 * 						3) Check the receivers
 * 
 * \param	chanSwitch	The pointer to set the switch mode
 * \param	rc_check	The pointer to the state of the remote
 * \param	motorstate	The pointer to the motor state
 */
void tasks_rc_user_channels(uint8_t *chanSwitch, int8_t *rc_check, int8_t *motor_state);


/**
 * \brief            Updates the state and mode of the UAV
 */
task_return_t tasks_set_mav_mode_n_state(void* arg);


/**
 * \brief            Run the main stabilisation loop
 */
task_return_t tasks_run_stabilisation(void* arg);


/**
 * \brief            Run GPS update
 */
task_return_t tasks_run_gps_update(void* arg);


/**
 * \brief            Run the navigation task
 */
task_return_t tasks_run_navigation_update(void* arg);


/**
 * \brief            Run the barometer task
 */
task_return_t tasks_run_barometer_update(void* arg);

/**
 * \brief            Run the sonar task
 */
task_return_t sonar_update(void* arg);

/**
 * \brief            Run the analog to digital converter task
 */
task_return_t adc_update(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_H_ */