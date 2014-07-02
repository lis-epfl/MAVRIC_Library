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
void create_tasks(void);


/**
 * \brief          	Returns a pointer to the main task-set  
 * 
 * \return          Pointer to the main task-set
 */
task_set* get_main_taskset(void);


/**
 * \brief            Relevel the imu
 */
void relevel_imu(void);


/**
 * \brief            Updates the IMU
 */
void run_imu_update(void);


/**
 * \brief            	This function does bullshit
 * \details  			1) Switch on/off collision avoidance
 * 						2) Switch on/off the motor
 * 						3) Check the receivers
 * 
 * \param chanSwitch [ description]
 * \param rc_check   [ description]
 * \param motorbool  [ description]
 */
void rc_user_channels(uint8_t *chanSwitch, int8_t *rc_check, int8_t *motorbool);


/**
 * \brief            Updates the state and mode of the UAV
 */
task_return_t set_mav_mode_n_state(void);


/**
 * \brief            Run the main stabilisation loop
 */
task_return_t run_stabilisation(void);


/**
 * \brief            Run GPS update
 */
task_return_t gps_task(void);


/**
 * \brief            Run the navigation task
 */
task_return_t run_navigation_task(void);


/**
 * \brief            Run the barometer task
 */
task_return_t run_barometer(void);


#ifdef __cplusplus
}
#endif

#endif /* TASKS_H_ */