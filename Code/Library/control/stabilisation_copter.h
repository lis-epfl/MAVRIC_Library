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

/**
 * \brief Structure containing the stacked controller
 */
typedef struct 
{
	Stabiliser_t rate_stabiliser;								///< The rate controller structure
	Stabiliser_t attitude_stabiliser;							///< The attitude controller structure
	Stabiliser_t velocity_stabiliser;							///< The velocity controller structure
	float yaw_coordination_velocity;							///< the yaw coordination value in velocity control mode
} Stabiliser_Stack_copter_t;

/**
 * \brief Structure containing the pointers to the data needed in this module
 */
typedef struct
{
	Stabiliser_Stack_copter_t *stabiliser_stack;				///< The pointer to the PID parameters values for the stacked controller 
	Control_Command_t *controls;								///< The pointer to the control structure
	run_mode_t * run_mode;										///< The pointer to the motor run mode
	Imu_Data_t *imu;											///< The pointer to the IMU structure
	AHRS_t *attitude_estimation;								///< The pointer to the attitude estimation structure
	position_estimator_t *pos_est;								///< The pointer to the position estimation structure
	servo_output_t* servos;										///< The pointer to the servos structure
} stabilise_copter_t;

/**
 * \brief							Initialize module stabilization
 *
 * \param	stabilisation_copter	The pointer to the stabilisation copter structure
 * \param	stabiliser_stack		The pointer to structure with all controllers
 * \param	control_input			The pointer to the controlling inputs
 * \param	run_mode				The pointer to the mode of the motors (MOTORS_ON, MOTORS_OFF,
 * \param	imu						The pointer to the IMU structure
 * \param	attitude_estimation		The pointer to the attitude estimation structure
 * \param	pos_est					The pointer to the position estimation structure
 */
void stabilisation_copter_init(stabilise_copter_t* stabilisation_copter, Stabiliser_Stack_copter_t* stabiliser_stack, Control_Command_t* controls, run_mode_t* run_mode, Imu_Data_t* imu, AHRS_t* attitude_estimation, position_estimator_t* pos_est, servo_output_t* servos);

/**
 * \brief							Gets the velocity vector from the remote
 *
 * \param	tvel					The pointer to array for storing the velocity vector 
 * \param	stabilisation_copter	The pointer to the stabilisation copter structure
 */
void stabilisation_copter_get_velocity_vector_from_remote(float tvel[], stabilise_copter_t* stabilisation_copter);

/**
 * \brief							Main Controller for controlling and stabilizing the quad
 *
 * \param	stabilisationParam		The stabilisation structure
 */
void stabilisation_copter_cascade_stabilise(stabilise_copter_t* stabilisation_copter);

/**
 * \brief							Mix to servo for quad configuration diagonal
 *
 * \param	control					Pointer to controlling inputs
 * \param	servos					The array of servos structure
 */
void stabilisation_copter_mix_to_servos_diag_quad(Control_Command_t *control, servo_output_t servos[4]);

/**
 * \brief							Mix to servo for quad configuration cross
 *
 * \param	control					Pointer to controlling inputs
 * \param	servos					The array of servos structure
 */
void stabilisation_copter_mix_to_servos_cross_quad(Control_Command_t *control, servo_output_t servos[4]);

#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_COPTER_H_ */