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
 * \file quaternion_controler.c
 *
 * A quaternion-based attitude controller, takes a reference quaternion as input
 * and provides angular errors for roll, pitch and yaw in the local frame.
 */


#ifndef QUATERNION_CONTROLLER_H_
#define QUATERNION_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "maths.h"
#include "coord_conventions.h"

/**
 * \brief Quaternion controller data structure
 */
typedef struct {
	UQuat_t* quat_attitude;	 	///<	Pointer to attitude, must be updated externally (imu)
	UQuat_t quat_ref;			///<	Reference attitude, the errors in roll pitch and yaw will be computed relative to this reference
	float rpy_errors[3];		///<	Local errors in roll, pitch and yaw
} quaternion_controller_t;


/**
 * \brief               	Initialises the Uqternion controller structure
 * 
 * \param 	controller    	Pointer to data structure
 * \param 	quat_attitude 	Pointer to the quaternion attitude used as input
 */
void quaternion_controler_init(quaternion_controller_t* controller, UQuat_t* quat_attitude);


/**
 * \brief               	Updates the reference attitude
 * 
 * \param 	controller    	Pointer to data structure
 * \param 	quat_ref      	New attitude quaternion to be used as reference
 */
void quaternion_controller_set_quat_ref(quaternion_controller_t* controller, UQuat_t quat_ref);


/**
 * \brief               	Function to update the reference attitude from Euler angles
 * 
 * \param 	controller    	Pointer to data structure
 * \param 	aero          	Roll, pitch and yaw angle given in aero_attitude_t structure (radians)
 */
void quaternion_controller_set_quat_ref_from_aero(quaternion_controller_t* controller, Aero_Attitude_t aero);


/**
 * \brief               	Function to update the reference attitude from Euler angles
 * 
 * \param 	controller    	Pointer to data structure
 * \param 	rpy           	Roll, pitch and yaw angle
 */
void quaternion_controller_set_quat_ref_from_rpy(quaternion_controller_t* controller, float rpy[3]);


/**
 * \brief               	Main update function, computes the local angular errors is roll, pitch and yaw
 * 
 * \param 	controller    	Pointer to data structure (radians)
 */
void quaternion_controller_update(quaternion_controller_t* controller);


#ifdef __cplusplus
}
#endif

#endif /* QUATERNION_CONTROLLER_H_ */