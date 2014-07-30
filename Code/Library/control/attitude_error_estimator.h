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
 * \file attitude_error_estimator.h
 *
 * A quaternion-based attitude-error estimator, takes a reference quaternion as input
 * and provides angular errors for roll, pitch and yaw in the local frame.
 */


#ifndef QUATERNION_CONTROLLER_H_
#define QUATERNION_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "quaternions.h"
#include "coord_conventions.h"
#include "ahrs.h"

/**
 * \brief Quaternion attitude error estimator data structure
 */
typedef struct {
	quat_t quat_ref;			///<	Reference attitude, the errors in roll pitch and yaw will be computed relative to this reference
	float rpy_errors[3];		///<	Local errors in roll, pitch and yaw
	const ahrs_t* ahrs;			///<	Pointer to AHRS (current attitude), must be updated externally
} attitude_error_estimator_t;


/**
 * \brief               	Initialises the Uqternion estimator structure
 * 
 * \param 	estimator    	Pointer to data structure
 * \param 	ahrs		 	Pointer to the estimated attitude
 */
void attitude_error_estimator_init(attitude_error_estimator_t* estimator, const ahrs_t* ahrs);


/**
 * \brief               	Updates the reference attitude
 * 
 * \param 	estimator    	Pointer to data structure
 * \param 	quat_ref      	New attitude quaternion to be used as reference
 */
void attitude_error_estimator_set_quat_ref(attitude_error_estimator_t* estimator, quat_t quat_ref);


/**
 * \brief               	Function to update the reference attitude from Euler angles
 * 
 * \param 	estimator    	Pointer to data structure
 * \param 	aero          	Roll, pitch and yaw angle given in aero_attitude_t structure (radians)
 */
void attitude_error_estimator_set_quat_ref_from_aero(attitude_error_estimator_t* estimator, Aero_Attitude_t aero);


/**
 * \brief               	Function to update the reference attitude from Euler angles
 * 
 * \param 	estimator    	Pointer to data structure
 * \param 	rpy           	Roll, pitch and yaw angle
 */
void attitude_error_estimator_set_quat_ref_from_rpy(attitude_error_estimator_t* estimator, float rpy[3]);


/**
 * \brief               	Main update function, computes the local angular errors is roll, pitch and yaw
 * 
 * \param 	estimator    	Pointer to data structure (radians)
 */
void attitude_error_estimator_update(attitude_error_estimator_t* estimator);


#ifdef __cplusplus
}
#endif

#endif /* QUATERNION_CONTROLLER_H_ */