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


#include "quaternion_controller.h"
#include "coord_conventions.h"

void quaternion_controler_init(quaternion_controller_t* controller, UQuat_t* quat_attitude)
{
	// Init ref to quat_attitude
	controller->quat_attitude = quat_attitude;

	// Init quat_ref
	controller->quat_ref.s = 1;
	controller->quat_ref.v[0] = 0;
	controller->quat_ref.v[1] = 0;
	controller->quat_ref.v[2] = 0;

	// Init output
	controller->rpy_errors[0] = 0;
	controller->rpy_errors[1] = 0;
	controller->rpy_errors[2] = 0;
}


void quaternion_controller_set_quat_ref(quaternion_controller_t* controller, UQuat_t quat_ref)
{
	controller->quat_ref = quat_ref;
}


void quaternion_controller_set_quat_ref_from_aero(quaternion_controller_t* controller, Aero_Attitude_t aero)
{
	controller->quat_ref = coord_conventions_quaternion_from_aero(aero);
}


void quaternion_controller_set_quat_ref_from_rpy(quaternion_controller_t* controller, float rpy[3])
{
	Aero_Attitude_t aero;
	aero.rpy[0] = rpy[0];
	aero.rpy[1] = rpy[1];
	aero.rpy[2] = rpy[2];

	controller->quat_ref = coord_conventions_quaternion_from_aero(aero);
}


void quaternion_controller_update(quaternion_controller_t* controller)
{
	UQuat_t quat_attitude = *controller->quat_attitude;
	UQuat_t quat_ref = controller->quat_ref;
	UQuat_t quat_error;

	// Compute quaternioin error in global frame
	quat_error = quaternions_multiply(quat_ref, quaternions_inverse(quat_attitude));
	
	// Express error in local coordinates
	quat_error = quaternions_multiply(quaternions_inverse(quat_attitude),
							quaternions_multiply(quat_error, quat_attitude));

	// Find shortest rotation
	if (quat_error.s < 0)
	{
		quat_error = quaternions_inverse(quat_error);
	}

	// Approximate roll pitch and yaw errors with quat_error vector part
	controller->rpy_errors[0] = 2 * quat_error.v[0];
	controller->rpy_errors[1] = 2 * quat_error.v[1];
	controller->rpy_errors[2] = 2 * quat_error.v[2];
}
