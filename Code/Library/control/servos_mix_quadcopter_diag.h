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
 * \file servos_mix_quadcopter_diag.h
 *
 * Links between torque commands and servos PWM command for quadcopters in diagonal configuration
 */


// TODO: update documentation

#ifndef SERVOS_MIX_QUADCOPTER_DIAG_H_
#define SERVOS_MIX_QUADCOPTER_DIAG_H_

#ifdef __cplusplus
	extern "C" {
#endif


#include "control_command.h"
#include "servos.h"


typedef enum
{
	CW 	= 1,
	CCW	= -1
} rot_dir_t;


typedef struct
{
	uint8_t 	motor_front_right;
	uint8_t 	motor_front_left;
	uint8_t 	motor_rear_right;
	uint8_t		motor_rear_left;
	rot_dir_t 	motor_front_right_dir;
	rot_dir_t 	motor_front_left_dir;
	rot_dir_t 	motor_rear_right_dir;
	rot_dir_t 	motor_rear_left_dir;
	float 		min_thrust;
	float		max_thrust;
} servo_mix_quadcopter_diag_conf_t;


/**
 * \brief	servos mix structure
 */
typedef struct 
{	
	uint8_t   	motor_front_right;
	uint8_t   	motor_front_left;
	uint8_t   	motor_rear_right;
	uint8_t   	motor_rear_left;
	rot_dir_t 	motor_front_right_dir;
	rot_dir_t 	motor_front_left_dir;
	rot_dir_t 	motor_rear_right_dir;
	rot_dir_t 	motor_rear_left_dir;
	float 		min_thrust;
	float		max_thrust;
	const torque_command_t* torque_command;
	const thrust_command_t* thrust_command;
	servos_t*          		servos;
} servo_mix_quadcotper_diag_t;


/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param servo_mix [description]
 * @param config [description]
 * @param torque_command [description]
 * @param servo_pwm [description]
 */
void servo_mix_quadcotper_diag_init(servo_mix_quadcotper_diag_t* mix, const servo_mix_quadcopter_diag_conf_t* config, const torque_command_t* torque_command, const thrust_command_t* thrust_command, servos_t* servos);


/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param servo_mix [description]
 */
void servos_mix_quadcopter_diag_update(servo_mix_quadcotper_diag_t* mix);


#ifdef __cplusplus
	}
#endif

#endif