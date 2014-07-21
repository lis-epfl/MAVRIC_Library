/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 
 
/**
 * \file stabilisation.h
 *
 * Executing the PID controllers for stabilization
 */


#ifndef STABILISATION_H_
#define STABILISATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "imu.h"
#include "pid_control.h"
#include "scheduler.h"
 
typedef enum 
{
	VELOCITY_COMMAND_MODE, 
	ATTITUDE_COMMAND_MODE, 
	RATE_COMMAND_MODE
} control_mode_t;

typedef enum     
{
	YAW_RELATIVE, 
	YAW_ABSOLUTE, 
	YAW_COORDINATED
} yaw_mode_t;

typedef struct 
{
	float rpy[3];						///< roll pitch yaw rates/angles
	float thrust;						///< thrust
	float tvel[3];						///< target velocity in m/s
	float theading;						///< absolute target heading	
	control_mode_t control_mode;		///< control mode
	yaw_mode_t     yaw_mode;			///< yaw mode
} Control_Command_t;

typedef struct {
	PID_Controller_t rpy_controller[3];	 ///< roll pitch yaw  controllers
	PID_Controller_t thrust_controller;  ///< thrust controller
	Control_Command_t output;			 ///< output
} Stabiliser_t;

/**
 * \brief				Execute the PID controllers used for stabilization
 * 
 * \param	stabiliser	Pointer to the structure containing the PID controllers
 * \param	dt			Timestep
 * \param	errors		Array containing the errors of the controlling variables
 */
void stabilisation_run(Stabiliser_t *stabiliser, float dt, float errors[]);

/**
 * \brief	Task to send the mavlink roll, pitch, yaw angular speeds and thrust setpoints message
 *
 * \param	stabiliser	Pointer to the structure containing the PID controllers
 * 
 * \return	The status of execution of the task
 */
task_return_t stabilisation_send_rpy_speed_thrust_setpoint(Stabiliser_t* rate_stabiliser);

/**
 * \brief	Task to send the mavlink roll, pitch and yaw errors message
 * 
 * \param	stabiliser	Pointer to the structure containing the PID controllers
 *
 * \return	The status of execution of the task
 */
task_return_t stabilisation_send_rpy_rates_error(Stabiliser_t* rate_stabiliser);

/**
 * \brief	Task to send the mavlink roll, pitch, yaw and thrust setpoints message
 *
 * \param	stabiliser	Pointer to the structure containing the PID controllers
 * 
 * \return	The status of execution of the task
 */
task_return_t stabilisation_send_rpy_thrust_setpoint(Control_Command_t* controls);

#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_H_ */