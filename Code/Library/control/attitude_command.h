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
 * \file attitude_command.h
 *
 * Data structures to hold attitude commands
 */

#include "quaternions.h"


/**
 * \brief	Attitude command mode, either quaternion, either roll pitch and yaw angles
 */
typedef enum
{
	ATTITUDE_COMMAND_MODE_QUATERNION	= 0,
	ATTITUDE_COMMAND_MODE_RPY			= 1
} attitude_command_mode_t;


/**
 * \brief	Attitude command structure
 */
typedef struct 
{	
	quat_t quat;					///<	Attitude quaternion
	float rpy[3];					///<	Roll, pitch and yaw angles 
 	attitude_command_mode_t mode;	///< 	Command mode, defines whether the quaternion or the rpy angles should be used
} attitude_command_t;