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

#include "compiler.h"
#include "imu.h"
#include "pid_control.h"
 
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

typedef enum
{
	MOTORS_OFF, 
	MOTORS_ON, 
	SIMULATE
} run_mode_t;

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

#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_H_ */