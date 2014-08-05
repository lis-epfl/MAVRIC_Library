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
 * \file attitude_controller_p2.h
 *
 * A simple quaternion based proportionnal squared controller for attitude control. It takes as input the desired attitude, the estimated attitude and the gyro rates.
 * The control scheme is the following:
 * - first the controller computes local roll pitch and yaw angular errors ( \f$ qerror \f$ ) using quaternion formulation
 * - angular errors are multiplied by a proportional gain \f$ P_{q} \f$
 * - feedback from the gyro is added to the output with gain \f$ P_{g} \f$
 * 
 * in short:
 * \f$  
 * 			output = (P_{q} . qerror) - (P_{g} . gyro)  
 * \f$
 */


#ifndef ATTITUDE_CONTROLLER_P2_H_
#define ATTITUDE_CONTROLLER_P2_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "attitude_error_estimator.h"
#include "control_command.h"
#include "ahrs.h"


/**
 * \brief P^2 Attitude controller structure
 */
typedef struct 
{
	const attitude_command_t* 	attitude_command;				///< Pointer to attitude command
	const ahrs_t* 				ahrs; 							///< Pointer to attitude estimation
	attitude_error_estimator_t  attitude_error_estimator;		///< Attitude error estimator
	float 						p_gain_angle[3];				///< Proportionnal gain for angular errors
	float 						p_gain_rate[3];					///< Proportionnal gain applied to gyros rates
	float 						output[3];						///< Output of the controller on the 3 axes
} attitude_controller_p2_t;


/**
 * \brief P^2 Attitude controller configuration
 */
typedef struct
{
	float p_gain_angle[3];										///< Proportionnal gain for angular errors
	float p_gain_rate[3];										///< Proportionnal gain applied to gyros rates
} attitude_controller_p2_conf_t;	


/**
 * \brief               		Initialises the attitude controller structure
 * 
 * \param 	controller    		Pointer to data structure
 * \param 	config				Pointer to configuration
 * \param 	attitude_command 	Pointer to attitude command
 * \param 	ahrs		 		Pointer to the estimated attitude
 */
void attitude_controller_p2_init(attitude_controller_p2_t* controller, const attitude_controller_p2_conf_t* config, const attitude_command_t* attitude_command, const ahrs_t* ahrs);


/**
 * \brief               	Main update function
 * 
 * \param 	controller    	Pointer to data structure
 */
void attitude_controller_p2_update(attitude_controller_p2_t* controller);


#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_CONTROLLER_P2_H_ */