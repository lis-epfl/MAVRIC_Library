/*
 * qfilter.c
 *
 *  Created on: Apr 13, 2010
 *      Author: felix
 */

#include "qfilter.h"
#include "imu.h"
#include "coord_conventions.h"
#include "print_util.h"
#include <math.h>

#define CROSS(u,v,out) \
	out[0] = u[1] * v[2] - u[2]*v[1];\
	out[1] = u[2] * v[0] - u[0]*v[2];\
	out[2] = u[0] * v[1] - u[1]*v[0];

#define SCP(u,v) \
	(u[0]*v[0]+u[1]*v[1]+u[2]*v[2]);

#define QI(q, out) \
	out.s = q.s;\
	out.v[0] = -q.v[0];\
	out.v[1] = -q.v[1];\
	out.v[2] = -q.v[2];

#define QUAT(q, s, v0, v1, v2) \
	q.s=s; q.v[0]=v0; q.v[1]=v1; q.v[2]=v2;

#define QMUL(q1,q2,out) \
	tmp[0] = q1.v[1] * q2.v[2] - q1.v[2]*q2.v[1];\
	tmp[1] = q1.v[2] * q2.v[0] - q1.v[0]*q2.v[2];\
	tmp[2] = q1.v[0] * q2.v[1] - q1.v[1]*q2.v[0];\
	out.v[0] = q2.s*q1.v[0] + q1.s *q2.v[0] +tmp[0];\
	out.v[1] = q2.s*q1.v[1] + q1.s *q2.v[1] +tmp[1];\
	out.v[2] = q2.s*q1.v[2] + q1.s *q2.v[2] +tmp[2];\
	out.s= q1.s*q2.s - SCP(q1.v, q2.v);

#define SQR(in) \
		(in)*(in)

float front_mag_vect_z;

float static inline scalar_product(float u[], float v[])
{
	float scp = (u[0]*v[0]+u[1]*v[1]+u[2]*v[2]);
	return scp;
}

UQuat_t static inline quat_multi(UQuat_t q1, UQuat_t q2)
{
	float tmp[3];
	UQuat_t out;
	tmp[0] = q1.v[1] * q2.v[2] - q1.v[2]*q2.v[1];
	tmp[1] = q1.v[2] * q2.v[0] - q1.v[0]*q2.v[2];
	tmp[2] = q1.v[0] * q2.v[1] - q1.v[1]*q2.v[0];
	out.v[0] = q2.s*q1.v[0] + q1.s *q2.v[0] +tmp[0];
	out.v[1] = q2.s*q1.v[1] + q1.s *q2.v[1] +tmp[1];
	out.v[2] = q2.s*q1.v[2] + q1.s *q2.v[2] +tmp[2];
	out.s= q1.s*q2.s - scalar_product(q1.v, q2.v);
	
	return out;
}

UQuat_t static inline quat_inv(UQuat_t q)
{
	int i;
	
	UQuat_t qinv;
	qinv.s = q.s;
	for (i=0;i<3;i++)
	{
		qinv.v[i] = -q.v[i];
	}
	return qinv;
}

UQuat_t quat_global_to_local(UQuat_t qe, UQuat_t qvect)
{
	UQuat_t qinv, qtmp;
	
	qinv = quat_inv(qe);
	qtmp = quat_multi(qinv,qvect);
	qtmp = quat_multi(qtmp,qe);

	return qtmp;
}

UQuat_t quat_local_to_global(UQuat_t qe, UQuat_t qvect)
{
	UQuat_t qinv, qtmp;
	
	qtmp = quat_multi(qe, qvect);
	qinv = quat_inv(qe);
	qtmp = quat_multi(qtmp, qinv);
	
	return qtmp;
}

void qfInit(Quat_Attitude_t *attitude,  float *scalefactor, float *bias) {
	uint8_t i;
	float init_angle;

	for (i=0; i<9; i++){
		attitude->sf[i]=1.0/(float)scalefactor[i];
		attitude->be[i]=bias[i]*attitude->sf[i];
		
	}
	for (i=0; i<3; i++){
		attitude->acc_bf[i]=0.0;
		attitude->vel_bf[i]=0.0;
		attitude->vel[i]=0.0;
		attitude->localPosition.pos[i]=0.0;
	}

//	attitude->be[3]=-0.03;
//	attitude->be[4]=0.08;
//	attitude->be[5]=0.15;

	attitude->qe.s=1.0;
	attitude->qe.v[0]=0.0;
	attitude->qe.v[1]=0.0;
	attitude->qe.v[2]=0.0;

	for(i=0; i<3; i++)
	{
		attitude->mag[i]=((float)attitude->raw_mag_mean[i])*attitude->sf[i+COMPASS_OFFSET]-attitude->be[i+COMPASS_OFFSET];
	}
	
	init_angle = atan2(-attitude->mag[1],attitude->mag[0]);

	dbg_print("Initial yaw:");
	dbg_print_num(init_angle*100.0,10);
	dbg_print(" = atan2(mag_y,mag_x) =");
	dbg_print_num(attitude->mag[1]*100.0,10);
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
	
	attitude->kp=0.05;
	attitude->ki=attitude->kp/15.0;
	
	attitude->kp_mag = 0.025;
	attitude->ki_mag = attitude->kp_mag/15.0;
	
	attitude->calibration_level=LEVELING;
	//dt=1.0/samplingrate;
}


void qfilter(Quat_Attitude_t *attitude, float *rates, float dt, bool simu_mode){
	uint8_t i;
	float  omc[3], omc_mag[3], rvc[3], tmp[3], snorm, norm, s_acc_norm, acc_norm, s_mag_norm, mag_norm;
	UQuat_t qed, qtmp1, up, up_bf, qtmp2, qtmp3;
	
	UQuat_t qed2, qtmp4, front, front_bf,qtmp5;

	for (i=0; i<3; i++){
		attitude->om[i]  = (1.0-GYRO_LPF)*attitude->om[i]+GYRO_LPF*(((float)rates[GYRO_OFFSET+i])*attitude->sf[i]-attitude->be[GYRO_OFFSET+i]);
		attitude->a[i]   = (1.0-ACC_LPF)*attitude->a[i]+ACC_LPF*(((float)rates[i+ACC_OFFSET])*attitude->sf[i+ACC_OFFSET]-attitude->be[i+ACC_OFFSET]);
		attitude->mag[i] = (1.0-MAG_LPF)*attitude->mag[i]+MAG_LPF*(((float)rates[i+COMPASS_OFFSET])*attitude->sf[i+COMPASS_OFFSET]-attitude->be[i+COMPASS_OFFSET]);
	}

	// up_bf = qe^-1 *(0,0,0,-1) * qe
	up.s=0; up.v[0]=UPVECTOR_X; up.v[1]=UPVECTOR_Y; up.v[2]=UPVECTOR_Z;
	up_bf = quat_global_to_local(attitude->qe, up);
	
	//up_bf.s=0; up_bf.v[0]=UPVECTOR_X; up_bf.v[1]=UPVECTOR_Y; up_bf.v[2]=UPVECTOR_Z;
	//QI(attitude->qe, qtmp1);
	//QMUL(qtmp1, up_bf, qtmp3);
	//QMUL(qtmp3, attitude->qe, up_bf);

	// calculate norm of acceleration vector
	s_acc_norm=attitude->a[0]*attitude->a[0]+attitude->a[1]*attitude->a[1]+attitude->a[2]*attitude->a[2];
	if ((s_acc_norm>0.7*0.7)&&(s_acc_norm<1.3*1.3)) {
		// approximate square root by running 2 iterations of newton method
		acc_norm=1.0;
		acc_norm=0.5*(acc_norm+(s_acc_norm/acc_norm));
		acc_norm=0.5*(acc_norm+(s_acc_norm/acc_norm));

		tmp[0]=attitude->a[0]/acc_norm;
		tmp[1]=attitude->a[1]/acc_norm;
		tmp[2]=attitude->a[2]/acc_norm;
		// omc = a x up_bf.v
		CROSS(tmp, up_bf.v, omc);
	} else {
		omc[0]=0;		omc[1]=0; 		omc[2]=0;
	}

	// Heading computation
	front.s = 0;front.v[0] = FRONTVECTOR_X;front.v[1] = FRONTVECTOR_Y;front.v[2] = front_mag_vect_z;
	front_bf = quat_global_to_local(attitude->qe,front);
	
	//front_bf.s = 0;front_bf.v[0] = FRONTVECTOR_X;front_bf.v[1] = FRONTVECTOR_Y;front_bf.v[2] = front_mag_vect_z;
	//QI(attitude->qe,qtmp4);
	//QMUL(qtmp4, front_bf, qtmp5);
	//QMUL(qtmp5, attitude->qe, front_bf);
	
	// calculate norm of acceleration vector
	s_mag_norm=attitude->mag[0]*attitude->mag[0]+attitude->mag[1]*attitude->mag[1]+attitude->mag[2]*attitude->mag[2];
	if ((s_mag_norm>0.7*0.7)&&(s_mag_norm<1.3*1.3)) {
		// approximate square root by running 2 iterations of newton method
		mag_norm=1.0;
		mag_norm=0.5*(mag_norm+(s_acc_norm/mag_norm));
		mag_norm=0.5*(mag_norm+(s_acc_norm/mag_norm));

		tmp[0]=attitude->mag[0]/mag_norm;
		tmp[1]=attitude->mag[1]/mag_norm;
		tmp[2]=attitude->mag[2]/mag_norm;
		// omc = a x up_bf.v
		CROSS(tmp, front_bf.v, omc_mag);
	} else {
		omc_mag[0]=0;		omc_mag[1]=0; 		omc_mag[2]=0;
	}

	for (i=0; i<3; i++){
		if (simu_mode)
		{
			qtmp1.v[i] = attitude->om[i] +attitude->kp*omc[i];
		}else{
			qtmp1.v[i] = attitude->om[i] +attitude->kp*omc[i] +attitude->kp_mag*omc_mag[i];
		}
	}
	qtmp1.s=0;

	qed = quat_multi(attitude->qe,qtmp1);

	//QMUL(attitude->qe, qtmp1, qed);

	attitude->qe.s=attitude->qe.s+qed.s*dt;
	attitude->qe.v[0]+=qed.v[0]*dt;
	attitude->qe.v[1]+=qed.v[1]*dt;
	attitude->qe.v[2]+=qed.v[2]*dt;

	snorm=attitude->qe.s*attitude->qe.s+attitude->qe.v[0]*attitude->qe.v[0] + attitude->qe.v[1] * attitude->qe.v[1] + attitude->qe.v[2] * attitude->qe.v[2];
	if (snorm<0.0001) norm=0; else {
		
		// approximate square root by running 2 iterations of newton method
		norm=1.0;
		norm=0.5*(norm+(snorm/norm));
		norm=0.5*(norm+(snorm/norm));
		norm=0.5*(norm+(snorm/norm));
		//norm=0.5*(norm+(snorm/norm));
	}	
	attitude->qe.s/= norm;
	attitude->qe.v[0] /= norm;
	attitude->qe.v[1] /= norm;
	attitude->qe.v[2] /= norm;

	// bias estimate update
	attitude->be[0]+= - dt * attitude->ki * omc[0];
	attitude->be[1]+= - dt * attitude->ki * omc[1];
	attitude->be[2]+= - dt * attitude->ki * omc[2];

	// bias estimate update
	attitude->be[6]+= - dt * attitude->ki_mag * omc[0];
	attitude->be[7]+= - dt * attitude->ki_mag * omc[1];
	attitude->be[8]+= - dt * attitude->ki_mag * omc[2];

	switch (attitude->calibration_level) {
		case OFF:
			attitude->kp=0.08;//*(0.1/(0.1+s_acc_norm-1.0));
			attitude->ki=attitude->kp/15.0;
			break;
		case LEVELING:
			attitude->kp=0.3;
			attitude->ki=attitude->kp/10.0;
			for (i=0; i<3; i++) {
				// reset velocity estimate to zero 
				attitude->vel_bf[i]=0.0;
				attitude->vel[i]=0.0;
			}
			break;
		case LEVEL_PLUS_ACCEL:
			attitude->kp=0.3;
			attitude->ki=attitude->kp/10.0;
			attitude->be[3]+=   dt * attitude->kp * (attitude->a[0]-up_bf.v[0]);
			attitude->be[4]+=   dt * attitude->kp * (attitude->a[1]-up_bf.v[1]);
			attitude->be[5]+=   dt * attitude->kp * (attitude->a[2]-up_bf.v[2]);
			for (i=0; i<3; i++) {
				// reset velocity estimate to zero
				attitude->vel_bf[i]=0.0;
				attitude->vel[i]=0.0;
			}
			break;
		default:
			attitude->kp=0.02;
			attitude->ki=attitude->kp/15.0;
			break;
	}

	// calculate up-vector from qe
	//up_vec.s=0; up_vec.v[0]=0; up_vec.v[1]=0; up_vec.v[2]=1;
	//QMUL(qe, up_vec, qtmp1);
	attitude->up_vec.v[0]=up_bf.v[0];
	attitude->up_vec.v[1]=up_bf.v[1];
	attitude->up_vec.v[2]=up_bf.v[2];
		//QMUL(qtmp1, qtmp3, up_vec);
	
}



/*
void qfilter_f (int16_t *rates, float dt) {
	uint8_t i;
	float omc[3], rvc[3], cp2[3], snorm, norm;
	UQuat_t qed;

	for (i=0; i<3; i++){
		om[i]  = ((float)rates[i]-be[i])*sf[i];
		a[i] = ((float)rates[i+3]-be[i+3])*sf[i+3];
	}


	cp2[0] = (qe.v[0]-qe.v[2])*qe.v[2] - qe.s*qe.v[1];          //u[1] * v[2] - u[2]*v[1];
	cp2[1] = qe.s*qe.v[0] + qe.v[1] * qe.v[2];                  //u[2] * v[0] - u[0]*v[2];
	cp2[2] = -qe.v[1] * qe.v[1] - (qe.v[0]-qe.v[2])* qe.v[0];   //u[0] * v[1] - v[1]*u[0];

// calculate angular deviation between "up" estimate and acceleration vector
	omc[0]= -qe.s * qe.v[1]         - qe.v[1]*qe.v[0] + cp2[0];
	omc[1]=  qe.s*(qe.v[0]-qe.v[2]) - qe.v[2]*qe.v[1] + cp2[1];
	omc[2]=                           qe.v[2]*qe.v[2] + cp2[2];
	CROSS(a,omc,omc);

	for (i=0; i<3; i++){
		rvc[i] = om[i];// +kp*omc[i];
	}
	qed.s=-SCP(qe.v,rvc);
	qed.v[0]=qe.s*rvc[0] +qe.v[1]*rvc[2]-qe.v[2]*rvc[1];
	qed.v[1]=qe.s*rvc[1] +qe.v[2]*rvc[0]-qe.v[0]*rvc[2];
	qed.v[2]=qe.s*rvc[2] +qe.v[0]*rvc[1]-qe.v[1]*rvc[0];

	qe.s=qe.s+qed.s*dt;

	qe.v[0]+=qed.v[0]*dt;
	qe.v[1]+=qed.v[1]*dt;
	qe.v[2]+=qed.v[2]*dt;
	snorm=qe.s*qe.s+qe.v[0]*qe.v[0] + qe.v[1] * qe.v[1] + qe.v[2] * qe.v[2];
	// approximate square root by running 2 iterations of newton method
	norm=1.0;
	norm=0.5*(norm+(snorm/norm));
	norm=0.5*(norm+(snorm/norm));

	qe.s/= norm;
	qe.v[0] /= norm;
	qe.v[1] /= norm;
	qe.v[2] /= norm;

	// bias estimate update
	//be[0]+= - dt * ki * omc[0];
	//be[1]+= - dt * ki * omc[1];
	//be[2]+= - dt * ki * omc[2];


}
*/