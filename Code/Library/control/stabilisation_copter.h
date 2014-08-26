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
 * \file stabilisation_copter.h
 *
 * This file handles the stabilization of the platform
 */


#ifndef STABILISATION_COPTER_H_
#define STABILISATION_COPTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stabilisation.h"
#include "imu.h"
#include "position_estimation.h"
#include "imu.h"
#include "servo_pwm.h"
#include "mavlink_communication.h"

/**
 * \brief Structure containing the stacked controller
 */
typedef struct 
{
	stabiliser_t rate_stabiliser;								///< The rate controller structure
	stabiliser_t attitude_stabiliser;							///< The attitude controller structure
	stabiliser_t velocity_stabiliser;							///< The velocity controller structure
	float yaw_coordination_velocity;							///< the yaw coordination value in velocity control mode
} Stabiliser_Stack_copter_t;

/**
 * \brief Structure containing the pointers to the data needed in this module
 */
typedef struct
{
	Stabiliser_Stack_copter_t* stabiliser_stack;	///< The pointer to the PID parameters values for the stacked controller 
	const control_command_t* controls;					///< The pointer to the control structure
	const imu_t* imu;								///< The pointer to the IMU structure
	const ahrs_t* ahrs;								///< The pointer to the attitude estimation structure
	const position_estimator_t* pos_est;			///< The pointer to the position estimation structure
	servo_output_t* servos;							///< The pointer to the servos structure
} stabilise_copter_t;

/**
 * \brief							Initialize module stabilization
 *
 * \param	stabilisation_copter	The pointer to the stabilisation copter structure
 * \param	stabiliser_stack		The pointer to structure with all controllers
 * \param	control_input			The pointer to the controlling inputs
 * \param	imu						The pointer to the IMU structure
 * \param	ahrs		The pointer to the attitude estimation structure
 * \param	pos_est					The pointer to the position estimation structure
 * \param	servos					The pointer to the array of servos command values
 * \param	mavlink_communication	The pointer to the mavlink communication structure
 */
void stabilisation_copter_init(stabilise_copter_t* stabilisation_copter, Stabiliser_Stack_copter_t* stabiliser_stack, const control_command_t* controls, const imu_t* imu, const ahrs_t* ahrs, const position_estimator_t* pos_est,servo_output_t* servos, mavlink_communication_t* mavlink_communication);

/**
 * \brief							Main Controller for controlling and stabilizing the quad
 *
 * \param	stabilisation_copter	The stabilisation structure
 */
void stabilisation_copter_cascade_stabilise(stabilise_copter_t* stabilisation_copter);

/**
 * \brief							Mix to servo for quad configuration diagonal
 *
 * \param	control					Pointer to controlling inputs
 * \param	servos					The array of servos structure
 */
void stabilisation_copter_mix_to_servos_diag_quad(control_command_t *control, servo_output_t servos[4]);

/**
 * \brief							Mix to servo for quad configuration cross
 *
 * \param	control					Pointer to controlling inputs
 * \param	servos					The array of servos structure
 */
void stabilisation_copter_mix_to_servos_cross_quad(control_command_t *control, servo_output_t servos[4]);

void stabilisation_copter_joystick_input(control_command_t *control, mavlink_received_t* rec);

#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_COPTER_H_ */