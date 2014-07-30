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
 * \file attitude_error_estimator.c
 *
 * A quaternion-based attitude-error estimator, takes a reference quaternion as input
 * and provides angular errors for roll, pitch and yaw in the local frame.
 */


#include "attitude_error_estimator.h"
#include "coord_conventions.h"

void attitude_error_estimator_init(attitude_error_estimator_t* estimator, const ahrs_t* ahrs)
{
	// Init ref to quat_attitude
	estimator->ahrs = ahrs;

	// Init quat_ref
	estimator->quat_ref.s = 1;
	estimator->quat_ref.v[0] = 0;
	estimator->quat_ref.v[1] = 0;
	estimator->quat_ref.v[2] = 0;

	// Init output
	estimator->rpy_errors[0] = 0;
	estimator->rpy_errors[1] = 0;
	estimator->rpy_errors[2] = 0;
}


void attitude_error_estimator_set_quat_ref(attitude_error_estimator_t* estimator, quat_t quat_ref)
{
	estimator->quat_ref = quat_ref;
}


void attitude_error_estimator_set_quat_ref_from_aero(attitude_error_estimator_t* estimator, Aero_Attitude_t aero)
{
	estimator->quat_ref = coord_conventions_quaternion_from_aero(aero);
}


void attitude_error_estimator_set_quat_ref_from_rpy(attitude_error_estimator_t* estimator, float rpy[3])
{
	Aero_Attitude_t aero;
	aero.rpy[0] = rpy[0];
	aero.rpy[1] = rpy[1];
	aero.rpy[2] = rpy[2];

	estimator->quat_ref = coord_conventions_quaternion_from_aero(aero);
}


void attitude_error_estimator_update(attitude_error_estimator_t* estimator)
{
	quat_t quat_attitude = estimator->ahrs->qe;
	quat_t quat_ref = estimator->quat_ref;
	quat_t quat_error;

	// Compute quaternioin error in global frame
	quat_error = quaternions_multiply(quat_ref, quaternions_inverse(quat_attitude));
	
	// Express error in local coordinates
	quat_error = quaternions_multiply(	quaternions_inverse(quat_attitude),
										quaternions_multiply(quat_error, 
															 quat_attitude)  );

	// Find shortest rotation
	if (quat_error.s < 0)
	{
		quat_error = quaternions_inverse(quat_error);
	}

	// Approximate roll pitch and yaw errors with quat_error vector part
	estimator->rpy_errors[0] = 2 * quat_error.v[0];
	estimator->rpy_errors[1] = 2 * quat_error.v[1];
	estimator->rpy_errors[2] = 2 * quat_error.v[2];
}
