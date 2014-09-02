/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file tasks.h
 * 
 * \author MAV'RIC Team
 *   
 * \brief Definition of the tasks executed on the autopilot
 *
 ******************************************************************************/


#ifndef TASKS_H_
#define TASKS_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "scheduler.h"
#include "remote.h"

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
 * \details  			1) Switch on/off the motor
 * 						2) Check the receivers
 * 
 * \param	chan_switch	The pointer to set the switch mode
 * \param	rc_check	The pointer to the state of the remote
 * \param	motorstate	The pointer to the motor state
 */
void tasks_rc_user_channels(uint8_t* chan_switch, signal_quality_t* rc_check, int8_t* motor_state);


/**
 * \brief            Updates the state and mode of the UAV
 */
// task_return_t tasks_set_mav_mode_n_state(void* arg);


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

/**
 * \brief            Run the LED toggle task
 */
task_return_t tasks_led_toggle(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_H_ */