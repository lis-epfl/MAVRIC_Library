/*
* ins_kalman.c
* Quaternion kalman attitude filter
*
*  Created on: Apr 15, 2014
*      Author: ndousse
*/

#include "ins_kalman.h"
#include "kalman_filter.h"
#include "matrixlib_float.h"
#include <math.h>
#include "maths.h"
#include "filter.h"
#include "conf_platform.h"
#include "print_util.h"
#include "central_data.h"

ins_param ins_p;

//! Kalman filter object
kf_State    ins_mainKF;

//! The final output of the inertial navigation system
ins_Output  ins_out;

// step counter
unsigned long ins_counter;

// time counter
//unsigned short ins_time[INS_TIME_MEASURES];
int ins_t;

//! state
float x[7];

central_data_t *centralData;

void ins_kalman_init(Quat_Attitude_t *attitude,  float *scalefactor, float *bias) {
	centralData = get_central_data();
	
	uint8_t i;
	float init_angle;

	for (i=0; i<9; i++){
		attitude->sf[i]=1.0/(float)scalefactor[i];
		attitude->be[i]=bias[i];
		
	}
	for (i=0; i<3; i++){
		attitude->acc_bf[i]=0.0;
	}
	
	KF_INIT(INS_NSTATES, INS_NMEASURES, ins_mainKF);
	
	float* x = kf_GetX(&ins_mainKF);
	
	for (i=0;i<INS_NSTATES;i++)
	{
		x[i] = 0.0;
	}
	
	ins_p.bias_max        = 0.5;   // = 28.648 °/sec
	ins_p.sigw = 0.00052360;         // std deviation of gyros : 0.38 °/sec^2 (-> in rad :)
	
	ins_p.sigq_init = 1e-5;		// UPDATE 9.8.13, from 1e-2 to 1e-5, because otherwise large yaw drift after every sensor calibration ! (biases and quaternions would go crazy for a while until stabilizing, except yaw bias would stabilize shitily)
	ins_p.sigb_init = 1e-5;	// from 0.0001 to 1e-5 for same reasons

	ins_p.Qb_var = 1e-9; // UPDATE 5.8.13, realized this improved a lot the results (biases take longer time to stabilize, but it's much better)
	
	ins_p.R_var0 = 1.;
	ins_p.R_k1 = 1.;
	ins_p.R_k2 = 2.;
	ins_p.R_var= 4.;
}

/*!
*	Called when the bias are made, to reset the state assuming a still situation
*/
void ins_bias(Quat_Attitude_t *attitude, float *rates)
{
	float acc[3];
	acc[X] = (rates[X+ACC_OFFSET]-attitude->be[X+ACC_OFFSET])*attitude->sf[X+ACC_OFFSET];
	acc[Y] = (rates[Y+ACC_OFFSET]-attitude->be[Y+ACC_OFFSET])*attitude->sf[Y+ACC_OFFSET];
	acc[Z] = (rates[Z+ACC_OFFSET]-attitude->be[Z+ACC_OFFSET])*attitude->sf[Z+ACC_OFFSET];
	
	float theta,phi,psi;
	theta = asinf(acc[X]);     // pitch
	// roll
	if(acc[Y]==0 && acc[Z]==0)
	{
		phi = 0;
	}else{
		phi = atan2(-acc[Y], -acc[Z]);
	}
	
	dbg_print("init acc: (");
	dbg_print_num(acc[X]*100,10);
	dbg_print(", ");
	dbg_print_num(acc[Y]*100,10);
	dbg_print(", ");
	dbg_print_num(acc[Z]*100,10);
	dbg_print(")\n");
	
	dbg_print("Initial roll angle:");
	dbg_print_num(phi*100.0,10);
	dbg_print("\n");
	
	dbg_print("Initial pitch angle:");
	dbg_print_num(theta*100.0,10);
	dbg_print("\n");
	
	//if(utils_IsNan(p) || utils_IsNan(r)){p=0.; r=0.;}

	int i;
	for(i=0; i<3; i++)
	{
		attitude->mag[i]=((float)attitude->raw_mag_mean[i])*attitude->sf[i+COMPASS_OFFSET]-attitude->be[i+COMPASS_OFFSET];
	}
	float mag[3];
	for (i=0;i<3;i++)
	{
		mag[i] = (rates[i+COMPASS_OFFSET]-attitude->be[i+COMPASS_OFFSET])*attitude->sf[i+COMPASS_OFFSET];
	}
	
	psi = atan2(-mag[1],mag[0]);

	dbg_print("Initial yaw angle:");
	dbg_print_num(psi*100.0,10);
	dbg_print("\n");

	float cr = cosf(phi/2);
	float cp = cosf(theta/2);
	float cy = cosf(psi/2);
	float sr = sinf(phi/2);
	float sp = sinf(theta/2);
	float sy = sinf(psi/2);
	
	float* x = kf_GetX(&ins_mainKF);
	float* P = kf_GetP(&ins_mainKF);
	
	x[0] =  cr*cp*cy + sr*sp*sy;
	x[1] = -cr*sp*sy + sr*cp*cy;
	x[2] =  cr*sp*cy + sr*cp*sy;
	x[3] =  cr*cp*sy - sr*sp*cy;

	ins_out.rollAngle    =  phi*MATH_RAD_TO_DEG;
	ins_out.pitchAngle   =  theta*MATH_RAD_TO_DEG;
	ins_out.yawAngle     =  psi*MATH_RAD_TO_DEG;

	x[4] = 0;
	x[5] = 0;
	x[6] = 0;

	matf_zeros(INS_NSTATES, INS_NSTATES, P);
	P[(INS_NSTATES+1)*0] = ins_p.sigq_init;
	P[(INS_NSTATES+1)*1] = ins_p.sigq_init;
	P[(INS_NSTATES+1)*2] = ins_p.sigq_init;
	P[(INS_NSTATES+1)*3] = ins_p.sigq_init;
	P[(INS_NSTATES+1)*4] = ins_p.sigb_init;
	P[(INS_NSTATES+1)*5] = ins_p.sigb_init;
	P[(INS_NSTATES+1)*6] = ins_p.sigb_init;

	ins_p.R_var1 = 0.;
	ins_p.R_var2 = 0.;

	ins_counter = 0;
}

/*!
*	Calls all the function necessary to perform a complete loop of the Kalman filter
*/
void ins_step(Quat_Attitude_t *attitude, float *rates, float dt)
{

	// doesn't run just after initialization
	if(ins_counter == 0)
	{
		ins_bias(attitude,rates);
		ins_counter++;
		return;
	}
	ins_counter++;
	
	//-----------------
	// step 1 : PREDICT
	//-----------------
	ins_prediction(attitude,rates,dt);

	//----------------
	// step 2 : UPDATE
	//----------------
	ins_update(attitude,rates,dt);
	
	//----------------
	// step 3 : OUTPUT
	//----------------
	ins_GenerateOutput(attitude,rates);
	
}

/*!
*	Prediction step of the INS
*   The estimation of the next state value is done thanks to the quaternion rotation formular (function f)
*   Matrixes phi (jacobian of f) and Q (prediction error) are also calculated to update P
*/
void ins_prediction(Quat_Attitude_t *attitude, float *rates, float dt)
{
	int i;
	for (i=0; i<3; i++){
		attitude->om[i]  = (1.0-GYRO_LPF)*attitude->om[i]+GYRO_LPF*(((float)rates[i+GYRO_OFFSET]-attitude->be[i+GYRO_OFFSET])*attitude->sf[i]);
		attitude->a[i]   = (1.0-ACC_LPF)*attitude->a[i]+ACC_LPF*(((float)rates[i+ACC_OFFSET]-attitude->be[i+ACC_OFFSET])*attitude->sf[i+ACC_OFFSET]);
		attitude->mag[i] = (1.0-MAG_LPF)*attitude->mag[i]+MAG_LPF*(((float)rates[i+COMPASS_OFFSET]-attitude->be[i+COMPASS_OFFSET])*attitude->sf[i+COMPASS_OFFSET]);
	}
	
	float f[INS_NSTATES];
	float phi[INS_NSTATES*INS_NSTATES];
	float Q[INS_NSTATES*INS_NSTATES];
	
	float* x = kf_GetX(&ins_mainKF);
	float q0 = x[0];
	float q1 = x[1];
	float q2 = x[2];
	float q3 = x[3];
	
	float wx = rates[X+GYRO_OFFSET]*attitude->sf[X+GYRO_OFFSET] + x[4];
	float wy = rates[Y+GYRO_OFFSET]*attitude->sf[Y+GYRO_OFFSET] + x[5];
	float wz = rates[Z+GYRO_OFFSET]*attitude->sf[Z+GYRO_OFFSET] + x[6];
	
	f[0] = q0 + dt/2*(-q1*wx - q2*wy - q3*wz);
	f[1] = q1 + dt/2*( q0*wx - q3*wy + q2*wz);
	f[2] = q2 + dt/2*( q3*wx + q0*wy - q1*wz);
	f[3] = q3 + dt/2*(-q2*wx + q1*wy + q0*wz);
	f[4] = x[4];
	f[5] = x[5];
	f[6] = x[6];
	
	// calculation of phi and Q for the covariance matrix update
	
	// phi
	matf_zeros(INS_NSTATES, INS_NSTATES, phi);
	
	phi[0]  =  1.;
	phi[1]  = -0.5*dt*wx;
	phi[2]  = -0.5*dt*wy;
	phi[3]  = -0.5*dt*wz;
	phi[4]  = -0.5*dt*q1;
	phi[5]  = -0.5*dt*q2;
	phi[6]  = -0.5*dt*q3;
	phi[7]  =  0.5*dt*wx;
	phi[8]  =  1.;
	phi[9]  =  0.5*dt*wz;
	phi[10] = -0.5*dt*wy;
	phi[11] =  0.5*dt*q0;
	phi[12] = -0.5*dt*q3;
	phi[13] =  0.5*dt*q2;
	phi[14] =  0.5*dt*wy;
	phi[15] = -0.5*dt*wz;
	phi[16] =  1.;
	phi[17] =  0.5*dt*wx;
	phi[18] =  0.5*dt*q3;
	phi[19] =  0.5*dt*q0;
	phi[20] = -0.5*dt*q1;
	phi[21] =  0.5*dt*wz;
	phi[22] =  0.5*dt*wy;
	phi[23] = -0.5*dt*wx;
	phi[24] =  1.;
	phi[25] = -0.5*dt*q2;
	phi[26] =  0.5*dt*q1;
	phi[27] =  0.5*dt*q0;
	phi[32] =  1.;
	phi[40] =  1.;
	phi[48] =  1.;
	
	// Q
	matf_zeros(INS_NSTATES, INS_NSTATES, Q);
	
	float sigw = ins_p.sigw;
	
	Q[0] = 0.25*dt*dt*sigw*sigw*(q1*q1 + q2*q2 + q3*q3);
	Q[1] = -0.25*dt*dt*sigw*sigw*q0*q1;
	Q[2] = -0.25*dt*dt*sigw*sigw*q0*q2;
	Q[3] = -0.25*dt*dt*sigw*sigw*q0*q3;
	Q[7] = -0.25*dt*dt*sigw*sigw*q0*q1;
	Q[8] = 0.25*dt*dt*sigw*sigw*(q0*q0 + q2*q2 + q3*q3);
	Q[9] = -0.25*dt*dt*sigw*sigw*q1*q2;
	Q[10] = -0.25*dt*dt*sigw*sigw*q1*q3;
	Q[14] = -0.25*dt*dt*sigw*sigw*q0*q2;
	Q[15] = -0.25*dt*dt*sigw*sigw*q1*q2;
	Q[16] = 0.25*dt*dt*sigw*sigw*(q0*q0 + q1*q1 + q3*q3);
	Q[17] = -0.25*dt*dt*sigw*sigw*q2*q3;
	Q[21] = -0.25*dt*dt*sigw*sigw*q0*q3;
	Q[22] = -0.25*dt*dt*sigw*sigw*q1*q3;
	Q[23] = -0.25*dt*dt*sigw*sigw*q2*q3;
	Q[24] = 0.25*dt*dt*sigw*sigw*(q0*q0 + q1*q1 + q2*q2);
	
	
	matf_diag(INS_NSTATES, INS_NSTATES, Q, ins_p.Qb_var, 5, INS_NSTATES);     // prediction error of bias
	
	// runs the prediction step of the Kalman filter
	kf_Predict(&ins_mainKF, f, phi, Q);
	
	// normalizes the quaternions
	ins_QNorm(kf_GetX(&ins_mainKF));
}


/*!
*	Update step of the INS
*   The measurement z are provided (accelerometers), and the estimation of what
*   the accelerometer readings should be in function of the INS state is calculated (function h).
*   The measurement matrix R is calculated and adapts to the confidence we have in the measures
*   Matrix H (Jacobian of h) is also calculated.
*/
void ins_update(Quat_Attitude_t *attitude, float *rates, float dt)
{
	int i;
	float* x = kf_GetX(&ins_mainKF);
	float q0 = x[0];
	float q1 = x[1];
	float q2 = x[2];
	float q3 = x[3];
	float wx, wy, wz, v, dv, omega[3];
	
	if(dt > 0.5) dt = 0.5;
	
	float z[INS_NMEASURES];
	float H[INS_NSTATES*INS_NMEASURES];
	float h[INS_NMEASURES];
	float R[INS_NMEASURES*INS_NMEASURES];
	
	float h_g[3],h_cent[3],h_long[3];
	

	wx = rates[X+GYRO_OFFSET]*attitude->sf[X+GYRO_OFFSET] + x[4];
	wy = rates[Y+GYRO_OFFSET]*attitude->sf[Y+GYRO_OFFSET] + x[5];
	wz = rates[Z+GYRO_OFFSET]*attitude->sf[Z+GYRO_OFFSET] + x[6];
	
	omega[X] = wx;
	omega[Y] = wy;
	omega[Z] = wz;
	
	z[X] = -g*(rates[X+ACC_OFFSET]-attitude->be[X+ACC_OFFSET])*attitude->sf[X+ACC_OFFSET];
	z[Y] = -g*(rates[Y+ACC_OFFSET]-attitude->be[Y+ACC_OFFSET])*attitude->sf[Y+ACC_OFFSET];
	z[Z] = -g*(rates[Z+ACC_OFFSET]-attitude->be[Z+ACC_OFFSET])*attitude->sf[Z+ACC_OFFSET];
	
	//if(ins_p.COMPCENTR == 1)
	//{
	//    v  = ins_sensors->airSpeed;
	//    dv = ins_sensors->airAcceleration;
	//}
	//else
	//{
	v  = 0.;
	dv = 0.;
	//}
	
	// centripetal acceleration
	h_cent[0] = -dv;
	h_cent[1] = v*(-wz);
	h_cent[2] = v*( wy);
	
	//CROSS(omega,v_bf,h_cent);
	
	
	
	// temp = DCM^-1 *g
	h_g[0] = 2*g*(-(q0*q2) + q1*q3);
	h_g[1] = 2*g*(q0*q1 + q2*q3);
	h_g[2] = g*(1 - 2*(q1*q1 + q2*q2));
	
	// h = DCM*g + fcent
	matf_add(3, 1, h_cent, h_g, h);
	
	// generates H
	matf_zeros(INS_NSTATES, INS_NMEASURES, H);
	
	H[0]  = -2*g*q2;
	H[1]  =  2*g*q3;
	H[2]  = -2*g*q0;
	H[3]  =  2*g*q1;
	H[7]  =  2*g*q1;
	H[8]  =  2*g*q0;
	H[9]  =  2*g*q3;
	H[10] =  2*g*q2;
	H[13] = -v;
	H[15] = -4*g*q1;
	H[16] = -4*g*q2;
	H[19] =  v;
	
	// generates R (measurement error)
	
	// part 1 of the measurement error (R_var1) is calculated from the angular speed (ponderated by parameter R_k1)
	filter_LowPassf(&ins_p.R_var1,ins_p.R_k1*sqrtf(wx*wx + wy*wy + wz*wz),10., dt);   // low-pass filtered
	
	// part 2 of the measurement error (R_var2) is calculated from the difference between the estimated gravity norm and 9.81 (ponderated by parameter R_k2)
	matf_sub(3, 1, z, h_cent, h_g);       // temp = z - h_cent
	filter_LowPassf(&ins_p.R_var2,ins_p.R_k2*f_abs(9.81 - matf_norm(3, h_g)),10., dt);   // low-pass filtered
	
	// the measurement error variance is calculated. A constant variance (R_var0) is added to both variances calculated above
	ins_p.R_var_tot = ins_p.R_var0 + ins_p.R_var1 + ins_p.R_var2;
	
	matf_zeros(INS_NMEASURES, INS_NMEASURES, R);
	
	matf_diag(INS_NMEASURES, INS_NMEASURES, R, ins_p.R_var_tot, 1, INS_NMEASURES);       // uses variance calculated above
	
	// runs the update step of the Kalman filter
	kf_Update(&ins_mainKF, H, R, z, h,0);
	
	//for(i = 0; i<3; i++)
	//	kf_Update(&ins_mainKF, &H[i*INS_NSTATES], &R[(i+1)*INS_NMEASURES], &z[i], &h[i],1);	// incremental measures
	
	// normalizes the quaternions
	x = kf_GetX(&ins_mainKF);
	ins_QNorm(x);
	
	// saturates the biases
	int s;
	for(s=4; s<=6; s++)
	{
		// saturation
		if(x[s] >  ins_p.bias_max) x[s] =  ins_p.bias_max;
		if(x[s] < -ins_p.bias_max) x[s] = -ins_p.bias_max;
	}
}

/*!
*	Normalizes the quaternions
*/
void ins_QNorm(float * q)
{
	float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	if(norm>0.0000001)
	{
		q[0] = q[0]/norm;
		q[1] = q[1]/norm;
		q[2] = q[2]/norm;
		q[3] = q[3]/norm;
	}
}

/*!
*	Creates the output (roll pitch yaw value from the quaternions)
*/
void ins_GenerateOutput(Quat_Attitude_t *attitude, float *rates)
{
	int i;
	
	float* x = kf_GetX(&ins_mainKF);
	float q0 = x[0];
	float q1 = x[1];
	float q2 = x[2];
	float q3 = x[3];
	
	centralData->imu1.attitude.qe_kalman.s = q0;
	centralData->imu1.attitude.qe_kalman.v[0] = q1;
	centralData->imu1.attitude.qe_kalman.v[1] = q2;
	centralData->imu1.attitude.qe_kalman.v[2] = q3;
	centralData->imu1.attitude.qe = centralData->imu1.attitude.qe_kalman;
	
	ins_out.DCM[0] = 1-2*(q2*q2+q3*q3);
	ins_out.DCM[1] = 2*(q1*q2-q0*q3);
	ins_out.DCM[2] = 2*(q0*q2+q1*q3);
	ins_out.DCM[3] = 2*(q0*q3+q1*q2);
	ins_out.DCM[4] = 1-2*(q1*q1+q3*q3);
	ins_out.DCM[5] = 2*(q2*q3-q0*q1);
	ins_out.DCM[6] = 2*(q1*q3-q0*q2);
	ins_out.DCM[7] = 2*(q0*q1+q2*q3);
	ins_out.DCM[8] = 1-2*(q1*q1+q2*q2);
	
	// roll pitch yaw values
	ins_out.rollAngle   = atan2(ins_out.DCM[7], ins_out.DCM[8])*MATH_RAD_TO_DEG;
	ins_out.pitchAngle  = -asinf(ins_out.DCM[6])*MATH_RAD_TO_DEG;
	ins_out.yawAngle    = atan2(ins_out.DCM[3], ins_out.DCM[0])*MATH_RAD_TO_DEG;
	//ins_out.yawAngle    = (2*(q0 * q3 + q1 * q2)+ 1 - 2*(q2 * q2 + q3 * q3))*MATH_RAD_TO_DEG;
	ins_out.rollAngleAvion   = atan2(ins_out.DCM[7], ins_out.DCM[6])*MATH_RAD_TO_DEG;
	ins_out.pitchAngleAvion  = -asinf( -ins_out.DCM[8])*MATH_RAD_TO_DEG;
	ins_out.yawAngleAvion    = atan2(-ins_out.DCM[5], -ins_out.DCM[2])*MATH_RAD_TO_DEG;
	
	
	// computes ideal gravity vector as it should be measured by sensor frame
	ins_out.gravity[0] = ( sinf(ins_out.pitchAngle* MATH_DEG_TO_RAD))*g;
	ins_out.gravity[1] = (-cosf(ins_out.pitchAngle* MATH_DEG_TO_RAD) *sinf(ins_out.rollAngle * MATH_DEG_TO_RAD))*g;
	ins_out.gravity[2] = (-cosf(ins_out.rollAngle * MATH_DEG_TO_RAD) *cosf(ins_out.pitchAngle* MATH_DEG_TO_RAD))*g;
	
	
	// roll pitch yaw rates
	ins_out.rollRate    = rates[ROLL+GYRO_OFFSET]*attitude->sf[ROLL+GYRO_OFFSET]*MATH_RAD_TO_DEG + x[4]*MATH_RAD_TO_DEG;
	ins_out.pitchRate   = rates[PITCH+GYRO_OFFSET]*attitude->sf[PITCH+GYRO_OFFSET]*MATH_RAD_TO_DEG  + x[5]*MATH_RAD_TO_DEG;
	ins_out.yawRate     = rates[YAW+GYRO_OFFSET]*attitude->sf[YAW+GYRO_OFFSET]*MATH_RAD_TO_DEG    + x[6]*MATH_RAD_TO_DEG;
	
	centralData->imu1.attitude.om_kalman[ROLL] = ins_out.rollRate;
	centralData->imu1.attitude.om_kalman[PITCH] = ins_out.pitchRate;
	centralData->imu1.attitude.om_kalman[YAW] = ins_out.yawRate;
	
	for (i=0;i<3;i++)
	{
		centralData->imu1.attitude.om[i] = centralData->imu1.attitude.om_kalman[i];
	}
	
	UQuat_t front_vec_global = {.s=0.0, .v={1.0, 0.0, 0.0}};
	UQuat_t up;
	up.s=0; up.v[0]=UPVECTOR_X; up.v[1]=UPVECTOR_Y; up.v[2]=UPVECTOR_Z;
	attitude->up_vec = quat_global_to_local(attitude->qe, up);
	attitude->north_vec=quat_global_to_local(attitude->qe, front_vec_global);
}