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
 * \file qfilter.c
 *
 * This file implements a complementary filter for the attitude estimation
 */


#include "qfilter.h"
#include "conf_platform.h"
#include "coord_conventions.h"
#include "print_util.h"
#include <math.h>
#include "maths.h"


float front_mag_vect_z;

uint8_t counter = 0;

void qfilter_init(qfilter_t *attitude_filter, Imu_Data_t *imu1, AHRS_t *attitude_estimation) 
{
	uint8_t i;
	
	attitude_filter->imu1 = imu1;
	attitude_filter->attitude_estimation = attitude_estimation;

	/*for (i = 0; i < 9; i++)
	{
		attitude->sf[i] = 1.0f / (float)scalefactor[i];
		attitude->be[i] = bias[i];
	}*/
	for (i = 0; i < 3; i++)
	{
		attitude_filter->acc_bf[i] = 0.0f;
	}

//	imu1->calib_sensor.bias[3] = -0.03f;
//	imu1->calib_sensor.bias[4] = 0.08f;
//	imu1->calib_sensor.bias[5] = 0.15f;

	attitude_filter->attitude_estimation->qe.s = 1.0f;
	attitude_filter->attitude_estimation->qe.v[0] = 0.0f;
	attitude_filter->attitude_estimation->qe.v[1] = 0.0f;
	attitude_filter->attitude_estimation->qe.v[2] = 0.0f;
	
	attitude_filter->kp = 0.07f;
	attitude_filter->ki = attitude_filter->kp / 15.0f;
	
	attitude_filter->kp_mag = 0.1f;
	attitude_filter->ki_mag = attitude_filter->kp_mag / 15.0f;
}

void qfilter_init_quaternion(qfilter_t *attitude_filter)
{
	uint8_t i;
	float init_angle;
	
	for(i = 0; i < 3; i++)
	{
		attitude_filter->imu1->scaled_compass.data[i] = ((float)attitude_filter->raw_mag_mean[i] - attitude_filter->imu1->calib_compass.bias[i]) * attitude_filter->imu1->calib_compass.scale_factor[i];
	}
	
	init_angle = atan2( -attitude_filter->imu1->scaled_compass.data[1],attitude_filter->imu1->scaled_compass.data[0]);

	print_util_dbg_print("Initial yaw:");
	print_util_dbg_print_num(init_angle * 100.0f,10);
	print_util_dbg_print(" = atan2(mag_y,mag_x) =");
	print_util_dbg_print_num( -attitude_filter->imu1->scaled_compass.data[1] * 100.0f,10);
	print_util_dbg_print(" ,");
	print_util_dbg_print_num(attitude_filter->imu1->scaled_compass.data[0] * 100.0f,10);
	print_util_dbg_print("\n");

	front_mag_vect_z = attitude_filter->imu1->scaled_compass.data[2];
	print_util_dbg_print("Front mag(z) (*100):");
	print_util_dbg_print_num(front_mag_vect_z * 100.0f,10);
	print_util_dbg_print("\n");

	attitude_filter->attitude_estimation->qe.s = cos(init_angle / 2.0f);
	attitude_filter->attitude_estimation->qe.v[0] = 0.0f;
	attitude_filter->attitude_estimation->qe.v[1] = 0.0f;
	//attitude_filter->attitude_estimation->qe.v[2] = sin((PI + init_angle) / 2.0f);
	attitude_filter->attitude_estimation->qe.v[2] = sin(init_angle / 2.0f);
}

void qfilter_attitude_estimation(qfilter_t *attitude_filter, float dt){
	uint8_t i;
	float  omc[3], omc_mag[3] , tmp[3], snorm, norm, s_acc_norm, acc_norm, s_mag_norm, mag_norm;
	UQuat_t qed, qtmp1, up, up_bf;
	UQuat_t mag_global, mag_corrected_local;
	UQuat_t front_vec_global = {.s = 0.0f, .v = {1.0f, 0.0f, 0.0f}};
	float kp, kp_mag;

	// up_bf = qe^-1 *(0,0,0,-1) * qe
	up.s = 0; up.v[0] = UPVECTOR_X; up.v[1] = UPVECTOR_Y; up.v[2] = UPVECTOR_Z;
	up_bf = quaternions_global_to_local(attitude_filter->attitude_estimation->qe, up);
	
	// calculate norm of acceleration vector
	s_acc_norm = attitude_filter->imu1->scaled_accelero.data[0] * attitude_filter->imu1->scaled_accelero.data[0] + attitude_filter->imu1->scaled_accelero.data[1] * attitude_filter->imu1->scaled_accelero.data[1] + attitude_filter->imu1->scaled_accelero.data[2] * attitude_filter->imu1->scaled_accelero.data[2];
	if ((s_acc_norm > 0.7f * 0.7f)&&(s_acc_norm < 1.3f * 1.3f)) {
		// approximate square root by running 2 iterations of newton method
		acc_norm = maths_fast_sqrt(s_acc_norm);

		tmp[0] = attitude_filter->imu1->scaled_accelero.data[0] / acc_norm;
		tmp[1] = attitude_filter->imu1->scaled_accelero.data[1] / acc_norm;
		tmp[2] = attitude_filter->imu1->scaled_accelero.data[2] / acc_norm;
		// omc = a x up_bf.v
		CROSS(tmp, up_bf.v, omc);
	}
	else
	{
		omc[0] = 0;
		omc[1] = 0;
		omc[2] = 0;
	}

	// Heading computation
	// transfer 
	qtmp1 = quaternions_create_from_vector(attitude_filter->imu1->scaled_compass.data); 
	mag_global = quaternions_local_to_global(attitude_filter->attitude_estimation->qe, qtmp1);
	
	// calculate norm of compass vector
	//s_mag_norm = SQR(mag_global.v[0]) + SQR(mag_global.v[1]) + SQR(mag_global.v[2]);
	s_mag_norm = SQR(mag_global.v[0]) + SQR(mag_global.v[1]);
	if ((s_mag_norm > 0.004f * 0.004f)&&(s_mag_norm < 1.8f * 1.8f)) 
	{
		mag_norm = maths_fast_sqrt(s_mag_norm);

		mag_global.v[0] /= mag_norm;
		mag_global.v[1] /= mag_norm;
		mag_global.v[2] = 0.0f;   // set z component in global frame to 0

		// transfer magneto vector back to body frame 
		attitude_filter->attitude_estimation->north_vec = quaternions_global_to_local(attitude_filter->attitude_estimation->qe, front_vec_global);		
		mag_corrected_local = quaternions_global_to_local(attitude_filter->attitude_estimation->qe, mag_global);		
		// omc = a x up_bf.v
		CROSS(mag_corrected_local.v, attitude_filter->attitude_estimation->north_vec.v,  omc_mag);
		
	}
	else
	{
		omc_mag[0] = 0;
		omc_mag[1] = 0;
		omc_mag[2] = 0;
	}


	// get error correction gains depending on mode
	switch (attitude_filter->calibration_level)
	{
		case OFF:
			kp = attitude_filter->kp;//*(0.1f / (0.1f + s_acc_norm - 1.0f));
			kp_mag = attitude_filter->kp_mag;
			attitude_filter->ki = attitude_filter->kp / 15.0f;
			break;
			
		case LEVELING:
			kp = 0.5f;
			kp_mag = 0.5f;
			attitude_filter->ki = attitude_filter->kp / 10.0f;
			break;
			
		case LEVEL_PLUS_ACCEL:  // experimental - do not use
			kp = 0.3f;
			attitude_filter->ki = attitude_filter->kp / 10.0f;
			attitude_filter->imu1->calib_accelero.bias[0] += dt * attitude_filter->kp * (attitude_filter->imu1->scaled_accelero.data[0] - up_bf.v[0]);
			attitude_filter->imu1->calib_accelero.bias[1] += dt * attitude_filter->kp * (attitude_filter->imu1->scaled_accelero.data[1] - up_bf.v[1]);
			attitude_filter->imu1->calib_accelero.bias[2] += dt * attitude_filter->kp * (attitude_filter->imu1->scaled_accelero.data[2] - up_bf.v[2]);
			kp_mag = attitude_filter->kp_mag;
			break;
			
		default:
			kp = attitude_filter->kp;
			kp_mag = attitude_filter->kp_mag;
			attitude_filter->ki = attitude_filter->kp / 15.0f;
			break;
	}

	// apply error correction with appropriate gains for accelerometer and compass

	for (i = 0; i < 3; i++)
	{
		qtmp1.v[i] = (attitude_filter->imu1->scaled_gyro.data[i] + kp * omc[i] + kp_mag * omc_mag[i]);//0.5f*
	}
	qtmp1.s = 0;

	// apply step rotation with corrections
	qed = quaternions_multiply(attitude_filter->attitude_estimation->qe,qtmp1);

	attitude_filter->attitude_estimation->qe.s = attitude_filter->attitude_estimation->qe.s + qed.s * dt;
	attitude_filter->attitude_estimation->qe.v[0] += qed.v[0] * dt;
	attitude_filter->attitude_estimation->qe.v[1] += qed.v[1] * dt;
	attitude_filter->attitude_estimation->qe.v[2] += qed.v[2] * dt;

/*
	float wx = attitude_filter->imu1->scaled_gyro.data[X] + kp * omc[X] + kp_mag * omc_mag[X];
	float wy = attitude_filter->imu1->scaled_gyro.data[Y] + kp * omc[Y] + kp_mag * omc_mag[Y];
	float wz = attitude_filter->imu1->scaled_gyro.data[Z] + kp * omc[Z] + kp_mag * omc_mag[Z];

	float q0 = attitude_filter->attitude_estimation->qe.s;
	float q1 = attitude_filter->attitude_estimation->qe.v[0];
	float q2 = attitude_filter->attitude_estimation->qe.v[1];
	float q3 = attitude_filter->attitude_estimation->qe.v[2];

	attitude_filter->attitude_estimation->qe.s = q0 + dt / 2 * ( - q1 * wx - q2 * wy - q3 * wz);
	attitude_filter->attitude_estimation->qe.v[0] = q1 + dt / 2 * ( q0 * wx - q3 * wy + q2 * wz);
	attitude_filter->attitude_estimation->qe.v[1] = q2 + dt / 2 * ( q3 * wx + q0 * wy - q1 * wz);
	attitude_filter->attitude_estimation->qe.v[2] = q3 + dt / 2 * ( - q2 * wx + q1 * wy + q0 * wz);
*/

	snorm = attitude_filter->attitude_estimation->qe.s * attitude_filter->attitude_estimation->qe.s + attitude_filter->attitude_estimation->qe.v[0] * attitude_filter->attitude_estimation->qe.v[0] + attitude_filter->attitude_estimation->qe.v[1] * attitude_filter->attitude_estimation->qe.v[1] + attitude_filter->attitude_estimation->qe.v[2] * attitude_filter->attitude_estimation->qe.v[2];
	if (snorm < 0.0001f)
	{
		norm = 0.01f;
	}
	else
	{
		// approximate square root by running 2 iterations of newton method
		norm = maths_fast_sqrt(snorm);
	}
	attitude_filter->attitude_estimation->qe.s /= norm;
	attitude_filter->attitude_estimation->qe.v[0] /= norm;
	attitude_filter->attitude_estimation->qe.v[1] /= norm;
	attitude_filter->attitude_estimation->qe.v[2] /= norm;

	// bias estimate update
	attitude_filter->imu1->calib_gyro.bias[0] += - dt * attitude_filter->ki * omc[0] / attitude_filter->imu1->calib_gyro.scale_factor[0];
	attitude_filter->imu1->calib_gyro.bias[1] += - dt * attitude_filter->ki * omc[1] / attitude_filter->imu1->calib_gyro.scale_factor[1];
	attitude_filter->imu1->calib_gyro.bias[2] += - dt * attitude_filter->ki * omc[2] / attitude_filter->imu1->calib_gyro.scale_factor[2];

	// bias estimate update
	//attitude_filter->imu1->calib_sensor.bias[6] += - dt * attitude_filter->ki_mag * omc[0];
	//attitude_filter->imu1->calib_sensor.bias[7] += - dt * attitude_filter->ki_mag * omc[1];
	//attitude_filter->imu1->calib_sensor.bias[8] += - dt * attitude_filter->ki_mag * omc[2];

	// set up-vector (bodyframe) in attitude
	attitude_filter->attitude_estimation->up_vec.v[0] = up_bf.v[0];
	attitude_filter->attitude_estimation->up_vec.v[1] = up_bf.v[1];
	attitude_filter->attitude_estimation->up_vec.v[2] = up_bf.v[2];
}
