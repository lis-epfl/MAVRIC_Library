/*
 * ins_kalman.h
 * Quaternion kalman attitude filter
 * 
 *  Created on: Apr 15, 2014
 *      Author: ndousse
 */


#ifndef INS_KALMAN_H__
#define INS_KALMAN_H__

#include "qfilter.h"

#define INS_NSTATES 7
#define INS_NMEASURES 6
//#define INS_NMEASURES 3

#define g 9.81

//! Type definition for flight controller output [-1,1] or [0,1]
typedef struct
{
	float rollAngle;			//!< the INS output for the roll angle [-90,90]
	float pitchAngle;			//!< the INS output for the pitch angle [-90,90]
	float yawAngle;			    //!< the INS output for the yaw angle [-180,180]
	float rollAngleAvion;		//!< the INS output for the roll angle [-90,90]
	float pitchAngleAvion;		//!< the INS output for the pitch angle [-90,90]
	float yawAngleAvion;	    //!< the INS output for the yaw angle [-180,180]
	float rollRate;		    	//!< the INS output for the roll angle rate
	float pitchRate;			//!< the INS output for the pitch angle rate
	float yawRate;			    //!< the INS output for the yaw angle rate
	float rollSigma;			//!< the INS output for the roll angle standard deviation
	float pitchSigma;			//!< the INS output for the pitch angle standard deviation
	float yawSigma;				//!< the INS output for the yaw angle standard deviation
	
	float gravity[3];
	
	float DCM[9];
}
ins_Output;

typedef struct
{
	
	float bias_max;
	
	// Kalman filter noise parameters
	float sigw;
	
	float sigq_init;
	float sigb_init;
	
	float Qb_var;
	
	float R_var0;
	float R_k1;
	float R_k2;
	float R_k3;
	float R_var;
	float R_var1;
	float R_var2;
	float R_var3;
	float R_var_tot;
	float R_var_tot2;
	
}
ins_param;

void ins_kalman_init(Quat_Attitude_t *attitude,  float *scalefactor, float *bias);
void ins_bias(Quat_Attitude_t *attitude, float *rates);

void ins_step(Quat_Attitude_t *attitude, float *rates, float dt);

void ins_prediction(Quat_Attitude_t *attitude, float *rates, float dt);
void ins_update(Quat_Attitude_t *attitude, float *rates, float dt);
void ins_QNorm(float * q);
void ins_GenerateOutput(Quat_Attitude_t *attitude, float *rates);
#endif // INS_KALMAN_H__
