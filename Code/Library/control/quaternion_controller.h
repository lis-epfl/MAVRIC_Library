/*
 * quaternion_controler.h
 *
 * Created: 24/04/2013 12:14:04
 *  Author: julien
 * 
 * A quaternion-based attitude controller, takes a reference quaternion as input
 * and provides angular errors for roll, pitch and yaw in the local frame. 
 */ 


#ifndef QUATERNION_CONTROLLER_H_
#define QUATERNION_CONTROLLER_H_

#include "maths.h"
#include "coord_conventions.h"

typedef struct {
	UQuat_t* quat_attitude;	 // pointer to attitude, must be updated externally (imu)
	UQuat_t quat_ref;
	float rpy_errors[3];
} Quaternion_Controller_t;

void quaternion_controler_init(Quaternion_Controller_t* controller, UQuat_t* quat_attitude);
void quaternion_controller_set_quat_ref(Quaternion_Controller_t* controller, UQuat_t quat_ref);
void quaternion_controller_set_quat_ref_from_aero(Quaternion_Controller_t* controller, Aero_Attitude_t aero);
void quaternion_controller_set_quat_ref_from_rpy(Quaternion_Controller_t* controller, float rpy[3]);
void quaternion_controller_update(Quaternion_Controller_t* controller);

#endif /* QUATERNION_CONTROLLER_H_ */