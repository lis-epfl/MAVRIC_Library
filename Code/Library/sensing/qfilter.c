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

uint8_t counter=0;

void qfInit(Quat_Attitude_t *attitude,  float *scalefactor, float *bias) {
	uint8_t i;

	for (i=0; i<9; i++){
		attitude->sf[i]=1.0/(float)scalefactor[i];
		attitude->be[i]=bias[i];
		
	}
	for (i=0; i<3; i++){
		attitude->acc_bf[i]=0.0;
	}

//	attitude->be[3]=-0.03;
//	attitude->be[4]=0.08;
//	attitude->be[5]=0.15;

	attitude->qe.s=1.0;
	attitude->qe.v[0]=0.0;
	attitude->qe.v[1]=0.0;
	attitude->qe.v[2]=0.0;
	
	attitude->kp=0.07;
	attitude->ki=attitude->kp/15.0;
	
	attitude->kp_mag = 0.1;
	attitude->ki_mag = attitude->kp_mag/15.0;

	//dt=1.0/samplingrate;
}

void initQuat(Quat_Attitude_t *attitude)
{
	uint8_t i;
	float init_angle;
	
	for(i=0; i<3; i++)
	{
		attitude->mag[i]=((float)attitude->raw_mag_mean[i]-attitude->be[i+COMPASS_OFFSET])*attitude->sf[i+COMPASS_OFFSET];
	}
	
	init_angle = atan2(-attitude->mag[1],attitude->mag[0]);

	dbg_print("Initial yaw:");
	dbg_print_num(init_angle*100.0,10);
	dbg_print(" = atan2(mag_y,mag_x) =");
	dbg_print_num(-attitude->mag[1]*100.0,10);
	dbg_print(" ,");
	dbg_print_num(attitude->mag[0]*100.0,10);
	dbg_print("\n");

	front_mag_vect_z = attitude->mag[2];
	dbg_print("Front mag(z) (*100):");
	dbg_print_num(front_mag_vect_z*100.0,10);
	dbg_print("\n");

	attitude->qe.s = cos(init_angle/2.0);
	attitude->qe.v[0]=0.0;
	attitude->qe.v[1]=0.0;
	//attitude->qe.v[2]=sin((PI + init_angle)/2.0);
	attitude->qe.v[2]=sin(init_angle/2.0);
}

void qfilter(Quat_Attitude_t *attitude, float *rates, float dt){
	uint8_t i;
	float  omc[3], omc_mag[3] , tmp[3], snorm, norm, s_acc_norm, acc_norm, s_mag_norm, mag_norm;
	// float rvc[3];
	UQuat_t qed, qtmp1, up, up_bf;
	// UQuat_t qtmp2, qtmp3;
	UQuat_t mag_global, mag_corrected_local;
	UQuat_t front_vec_global = {.s=0.0, .v={1.0, 0.0, 0.0}};
	float kp, kp_mag;
	
	
	for (i=0; i<3; i++){
		attitude->om[i]  = (1.0-GYRO_LPF)*attitude->om[i]+GYRO_LPF*(((float)rates[i+GYRO_OFFSET]-attitude->be[i+GYRO_OFFSET])*attitude->sf[i+GYRO_OFFSET]);
		attitude->a[i]   = (1.0-ACC_LPF)*attitude->a[i]+ACC_LPF*(((float)rates[i+ACC_OFFSET]-attitude->be[i+ACC_OFFSET])*attitude->sf[i+ACC_OFFSET]);
		attitude->mag[i] = (1.0-MAG_LPF)*attitude->mag[i]+MAG_LPF*(((float)rates[i+COMPASS_OFFSET]-attitude->be[i+COMPASS_OFFSET])*attitude->sf[i+COMPASS_OFFSET]);
	}

	// up_bf = qe^-1 *(0,0,0,-1) * qe
	up.s=0; up.v[0]=UPVECTOR_X; up.v[1]=UPVECTOR_Y; up.v[2]=UPVECTOR_Z;
	up_bf = quat_global_to_local(attitude->qe, up);
	
	// calculate norm of acceleration vector
	s_acc_norm=attitude->a[0]*attitude->a[0]+attitude->a[1]*attitude->a[1]+attitude->a[2]*attitude->a[2];
	if ((s_acc_norm>0.7*0.7)&&(s_acc_norm<1.3*1.3)) {
		// approximate square root by running 2 iterations of newton method
		acc_norm=fast_sqrt(s_acc_norm);

		tmp[0]=attitude->a[0]/acc_norm;
		tmp[1]=attitude->a[1]/acc_norm;
		tmp[2]=attitude->a[2]/acc_norm;
		// omc = a x up_bf.v
		CROSS(tmp, up_bf.v, omc);
	} else {
		omc[0]=0;		omc[1]=0; 		omc[2]=0;
	}

	// Heading computation
	// transfer 
	qtmp1=quat_from_vector(attitude->mag); 
	mag_global = quat_local_to_global(attitude->qe, qtmp1);
	
	// calculate norm of compass vector
	//s_mag_norm=SQR(mag_global.v[0])+SQR(mag_global.v[1])+SQR(mag_global.v[2]);
	s_mag_norm=SQR(mag_global.v[0])+SQR(mag_global.v[1]);
	if ((s_mag_norm>0.004*0.004)&&(s_mag_norm<1.8*1.8)) 
	{
		mag_norm=fast_sqrt(s_mag_norm);

		mag_global.v[0]/=mag_norm;
		mag_global.v[1]/=mag_norm;
		mag_global.v[2]=0.0;   // set z component in global frame to 0

		// transfer magneto vector back to body frame 
		attitude->north_vec=quat_global_to_local(attitude->qe, front_vec_global);		
		mag_corrected_local=quat_global_to_local(attitude->qe, mag_global);		
		// omc = a x up_bf.v
		CROSS(mag_corrected_local.v, attitude->north_vec.v,  omc_mag);
		
	} else {
		omc_mag[0]=0;		omc_mag[1]=0; 		omc_mag[2]=0;
	}


	// get error correction gains depending on mode
	switch (attitude->calibration_level) {
		case OFF:
			kp=attitude->kp;//*(0.1/(0.1+s_acc_norm-1.0));
			kp_mag = attitude->kp_mag;
			attitude->ki=attitude->kp/15.0;
			break;
		case LEVELING:
			kp=0.5;
			kp_mag = 0.5;
			attitude->ki=attitude->kp/10.0;
			break;
		case LEVEL_PLUS_ACCEL:  // experimental - do not use
			kp=0.3;
			attitude->ki=attitude->kp/10.0;
			attitude->be[3]+=   dt * attitude->kp * (attitude->a[0]-up_bf.v[0]);
			attitude->be[4]+=   dt * attitude->kp * (attitude->a[1]-up_bf.v[1]);
			attitude->be[5]+=   dt * attitude->kp * (attitude->a[2]-up_bf.v[2]);
			break;
		default:
			kp=attitude->kp;
			kp_mag = attitude->kp_mag;
			attitude->ki=attitude->kp/15.0;
			break;
	}

	// apply error correction with appropriate gains for accelerometer and compass

	for (i=0; i<3; i++){
		qtmp1.v[i] = (attitude->om[i] + kp*omc[i] + kp_mag*omc_mag[i]);//0.5*
	}
	qtmp1.s=0;

	// apply step rotation with corrections
	qed = quat_multi(attitude->qe,qtmp1);

	attitude->qe.s=attitude->qe.s+qed.s*dt;
	attitude->qe.v[0]+=qed.v[0]*dt;
	attitude->qe.v[1]+=qed.v[1]*dt;
	attitude->qe.v[2]+=qed.v[2]*dt;

/*
	float wx = attitude->om[X] + kp*omc[X] + kp_mag*omc_mag[X];
	float wy = attitude->om[Y] + kp*omc[Y] + kp_mag*omc_mag[Y];
	float wz = attitude->om[Z] + kp*omc[Z] + kp_mag*omc_mag[Z];

	float q0 = attitude->qe.s;
	float q1 = attitude->qe.v[0];
	float q2 = attitude->qe.v[1];
	float q3 = attitude->qe.v[2];

	attitude->qe.s = q0 + dt/2*(-q1*wx - q2*wy - q3*wz);
	attitude->qe.v[0] = q1 + dt/2*( q0*wx - q3*wy + q2*wz);
	attitude->qe.v[1] = q2 + dt/2*( q3*wx + q0*wy - q1*wz);
	attitude->qe.v[2] = q3 + dt/2*(-q2*wx + q1*wy + q0*wz);
*/

	snorm=attitude->qe.s*attitude->qe.s+attitude->qe.v[0]*attitude->qe.v[0] + attitude->qe.v[1] * attitude->qe.v[1] + attitude->qe.v[2] * attitude->qe.v[2];
	if (snorm<0.0001) norm=0.01; else {
		// approximate square root by running 2 iterations of newton method
		norm=fast_sqrt(snorm);
		//norm=0.5*(norm+(snorm/norm));
	}	
	attitude->qe.s/= norm;
	attitude->qe.v[0] /= norm;
	attitude->qe.v[1] /= norm;
	attitude->qe.v[2] /= norm;

	// bias estimate update
	attitude->be[0]+= - dt * attitude->ki * omc[0] / attitude->sf[0];
	attitude->be[1]+= - dt * attitude->ki * omc[1] / attitude->sf[1];
	attitude->be[2]+= - dt * attitude->ki * omc[2] / attitude->sf[2];

	// bias estimate update
	//attitude->be[6]+= - dt * attitude->ki_mag * omc[0];
	//attitude->be[7]+= - dt * attitude->ki_mag * omc[1];
	//attitude->be[8]+= - dt * attitude->ki_mag * omc[2];



	// set up-vector (bodyframe) in attitude
	attitude->up_vec.v[0]=up_bf.v[0];
	attitude->up_vec.v[1]=up_bf.v[1];
	attitude->up_vec.v[2]=up_bf.v[2];
	
}
