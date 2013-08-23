/*
* estimator.c
*
*  Created on: April 30, 2013
*      Author: Philippe
*
*/

#include "imu.h"
#include "boardsupport.h"
#include "gps_ublox.h"
#include "bmp085.h"
#include "estimator.h"
#include "qfilter.h"
#include "coord_conventions.h"

#define INIT_X_P	10.
#define INIT_Y_P	10.
#define INIT_Z_P	10.

//#define Q_X0	0.00000026331
//#define Q_X1	0.000052662
//#define Q_X2	0
//#define Q_Y0	0.0000004396
//#define Q_Y1	0.00008793
//#define Q_Y2	0
//#define Q_Z0	0.00000148
//#define Q_Z1	0.0002979
//#define Q_Z2	0
#define Q_X0	0.001
#define Q_X1	0.0
#define Q_X2	0.0
#define Q_Y0	0.0
#define Q_Y1	0.001
#define Q_Y2	0.0
#define Q_Z0	0.0
#define Q_Z1	0.0
#define Q_Z2	0.001

#define R_X_POS		0.3895
#define R_Y_POS		0.6007
#define R_Z_POS		2.6278

#define R_X_VEL		2.
#define R_Y_VEL		2.
#define R_Z_VEL		2.

//Cross product
#define CP(u,v,out)\
out[0]=u[1]*v[2]-u[2]*v[1];\
out[1]=u[2]*v[0]-u[0]*v[2];\
out[2]=u[0]*v[1]-u[1]*v[0];

#define MUL_V_SCA(u,a)\
u[0]=u[0]*a;\
u[1]=u[1]*a;\
u[2]=u[2]*a;

#define SCP(u,v) \
(u[0]*v[0]+u[1]*v[1]+u[2]*v[2]);

#define QI(q, out) \
out.s = q.s;\
out.v[0] = -q.v[0];\
out.v[1] = -q.v[1];\
out.v[2] = -q.v[2];

#define QMUL(q1,q2,out) \
tmp[0] = q1.v[1] * q2.v[2] - q1.v[2]*q2.v[1];\
tmp[1] = q1.v[2] * q2.v[0] - q1.v[0]*q2.v[2];\
tmp[2] = q1.v[0] * q2.v[1] - q1.v[1]*q2.v[0];\
out.v[0] = q2.s*q1.v[0] + q1.s *q2.v[0] +tmp[0];\
out.v[1] = q2.s*q1.v[1] + q1.s *q2.v[1] +tmp[1];\
out.v[2] = q2.s*q1.v[2] + q1.s *q2.v[2] +tmp[2];\
out.s= q1.s*q2.s - SCP(q1.v, q2.v);

pressure_data *baro;
board_hardware_t *board;
float P[3][3][3]; // Covariance matrice for Z,X and Y
float Q[3][3];
float R[3];

float P2[3][3][3]; // Covariance matrice for Z,X and Y
float Q2[3][3];
float R2[3];

//----------------------------INITIALISATION------------------------
void e_init()
{
	board=get_board_hardware();
	e_kalman_init(X,INIT_X_P); //e stands for estimator not extended
	e_kalman_init(Y,INIT_Y_P);
	e_kalman_init(Z,INIT_Z_P);
	Q[0][0]=Q_X0;
	Q[0][1]=Q_X1;
	Q[0][2]=Q_X2;
	Q[1][0]=Q_Y0;
	Q[1][1]=Q_Y1;
	Q[1][2]=Q_Y2;
	Q[2][0]=Q_Z0;
	Q[2][1]=Q_Z1;
	Q[2][2]=Q_Z2;
	R[0]=R_X_POS;
	R[1]=R_Y_POS;
	R[2]=R_Z_POS;
	
	Q2[0][0]=Q_X0;
	Q2[0][1]=Q_X1;
	Q2[0][2]=Q_X2;
	Q2[1][0]=Q_Y0;
	Q2[1][1]=Q_Y1;
	Q2[1][2]=Q_Y2;
	Q2[2][0]=Q_Z0;
	Q2[2][1]=Q_Z1;
	Q2[2][2]=Q_Z2;
	R2[0]=R_X_POS;
	R2[1]=R_Y_POS;
	R2[2]=R_Z_POS;
	
	filter_init_delta_t = false;
	
	init_pos_gps();
}

void init_pos_gps()
{
	if (newValidGpsMsg() && (!(board->init_gps_position)))
	{
		board->init_gps_position = true;
		
		board->imu1.attitude.localPosition.origin.longitude = board->GPS_data.longitude;
		board->imu1.attitude.localPosition.origin.latitude = board->GPS_data.latitude;
		board->imu1.attitude.localPosition.origin.altitude = board->GPS_data.altitude;
		
		dbg_print("GPS initialized!\n");
	}
}

void e_kalman_init (int axis,float init_p) // axis = Z, X or Y
{
	int i, j;
	
	board->estimation.state[axis][POSITION] = 0; // Differential par rapport au point de depart
	board->estimation.state[axis][SPEED] = 0;
	
	board->estimation.state[axis][BIAIS] = board->imu1.attitude.be[axis + ACC_OFFSET];
	
	for (i=0; i<3; i++)
	{
		for (j=0; j<3; j++)
		{
			P[axis][i][j] = 0.0;
			P2[axis][i][j] = 0.0;
		}
		P[axis][i][i] = init_p;
		P2[axis][i][i] = init_p;
	}
}

//------------------------------PREDICTION--------------------------
void e_predict (UQuat_t *qe, float a[], float dt)
{
	
	UQuat_t qtmp1,qtmp2,qtmp3,qe_tmp;
	float tmp[3];
	//UQuat_t inv_qe;
	float acc_glo[3];
	
	//quat_rot(qe,x); // get the x vector of the quad in NED
	//quat_rot(qe,y);
	//quat_rot(qe,z);
	
	//MUL_V_SCA(x,a[1]) // get the right norm
	//MUL_V_SCA(y,a[0])
	//MUL_V_SCA(z,-a[2])
	//e_kalman_predict(X,(x[0]*x[0]+y[0]*y[0]+z[0]*z[0]),dt);//final x (in NED) acc
	//e_kalman_predict(Y,(x[1]*x[1]+y[1]*y[1]+z[1]*z[1]),dt);
	//e_kalman_predict(Z,(x[2]*x[2]+y[2]*y[2]+z[2]*z[2]),dt);
	
	qe_tmp = *qe;
	
	// compute acceleration in global frame
	// acc_glo = qe * acc_bf * qe-1
	qtmp1.s=0.0; qtmp1.v[0]=a[0]; qtmp1.v[1]=a[1];qtmp1.v[2]=a[2];
	QMUL(qe_tmp,qtmp1,qtmp2);
	QI(qe_tmp,qtmp1);
	QMUL(qtmp2,qtmp1,qtmp3);
	acc_glo[0]=qtmp3.v[0];acc_glo[1]=qtmp3.v[1];acc_glo[2]=qtmp3.v[2];

	//dbg_print("Acceleration:");
	//dbg_print_num(a[X]*1000,10);
	//dbg_print_num(a[Y]*1000,10);
	//dbg_print_num(a[Z]*1000,10);
	//dbg_print("\n");
	//dbg_print("Acceleration2:");
	//dbg_print_num(board->imu1.attitude.a[X]*1000,10);
	//dbg_print_num(board->imu1.attitude.a[Y]*1000,10);
	//dbg_print_num(board->imu1.attitude.a[Z]*1000,10);
	//dbg_print("\n");

	e_kalman_predict(X,acc_glo[X],dt);//final x (in NED) acc
	e_kalman_predict(Y,acc_glo[Y],dt);
	//e_kalman_predict_hf(X,acc_glo[X],dt);//final x (in NED) acc
	//e_kalman_predict_hf(Y,acc_glo[Y],dt);
	e_kalman_predict(Z,acc_glo[Z],dt);
}

//Rotation of vector vect with the quaternion quat
void quat_rot(UQuat_t *quat,float *vect)
{
	float temp1[3],temp2[3];
	CP((*quat).v,vect,temp1);
	temp1[0]=temp1[0]+(*quat).s*vect[0];
	temp1[1]=temp1[1]+(*quat).s*vect[1];
	temp1[2]=temp1[2]+(*quat).s*vect[2];
	CP((*quat).v,temp1,temp2);
	vect[0]= vect[0]+temp2[0]+temp2[0];
	vect[1]= vect[1]+temp2[1]+temp2[1];
	vect[2]= vect[2]+temp2[2]+temp2[2];
}

/*

state vector : X = [ x x_speed x_biais ]';

F = [ 1 dt -dt^2/2
0  1 -dt
0  0   1     ];

B = [ dt^2/2 dt 0]';

Q = [ 0.01  0     0
0     0.01  0
0     0     0.001 ];

Xk1 = F * Xk0 + B * accel;

Pk1 = F * Pk0 * F' + Q;

*/
void e_kalman_predict (int axis,float accel_meas, float dt)
{
	float accel_corr;
	float FPF00,FPF01,FPF02,FPF10,FPF11,FPF12,FPF20,FPF21,FPF22;
	
	
	/* update state */

	//board->estimation.state[axis][SPEED] = board->estimation.state[axis][SPEED]*(1.0-(VEL_DECAY*dt)) + dt * accel_meas;
	//board->estimation.state[axis][POSITION] = board->estimation.state[axis][POSITION]*(1.0-(POS_DECAY*dt)) + dt * board->estimation.state[axis][SPEED];
	
	board->estimation.state[axis][SPEED] = board->estimation.state[axis][SPEED] + dt * accel_meas;
	board->estimation.state[axis][POSITION] = board->estimation.state[axis][POSITION] + dt * board->estimation.state[axis][SPEED];
	
	//board->estimation.state[axis][POSITION] = board->imu1.attitude.localPosition.pos[axis];
	//board->estimation.state[axis][SPEED] = board->imu1.attitude.vel[axis];
	
	
	/* update covariance */
	// F*P*F' calculation
	FPF00 = P[axis][0][0] + dt * ( P[axis][1][0] + P[axis][0][1] + dt * P[axis][1][1] );
	FPF01 = P[axis][0][1] + dt * ( P[axis][1][1] - P[axis][0][2] - dt * P[axis][1][2] );
	FPF02 = P[axis][0][2] + dt * ( P[axis][1][2] );
	FPF10 = P[axis][1][0] + dt * (-P[axis][2][0] + P[axis][1][1] - dt * P[axis][2][1] );
	FPF11 = P[axis][1][1] + dt * (-P[axis][2][1] - P[axis][1][2] + dt * P[axis][2][2] );
	FPF12 = P[axis][1][2] + dt * (-P[axis][2][2] );
	FPF20 = P[axis][2][0] + dt * ( P[axis][2][1] );
	FPF21 = P[axis][2][1] + dt * (-P[axis][2][2] );
	FPF22 = P[axis][2][2];
	// P = F*P*F' + Q
	P[axis][0][0] = FPF00 + Q[axis][POSITION];
	P[axis][0][1] = FPF01;
	P[axis][0][2] = FPF02;
	P[axis][1][0] = FPF10;
	P[axis][1][1] = FPF11 + Q[axis][SPEED];
	P[axis][1][2] = FPF12;
	P[axis][2][0] = FPF20;
	P[axis][2][1] = FPF21;
	P[axis][2][2] = FPF22 + Q[axis][BIAIS];
	
	
	/*************************************************************************************/
	FPF00 = P2[axis][0][0] + dt * ( P2[axis][1][0] + P2[axis][0][1] + dt * P2[axis][1][1] );
	FPF01 = P2[axis][0][1] + dt * ( P2[axis][1][1] - P2[axis][0][2] - dt * P2[axis][1][2] );
	FPF02 = P2[axis][0][2] + dt * ( P2[axis][1][2] );
	FPF10 = P2[axis][1][0] + dt * (-P2[axis][2][0] + P2[axis][1][1] - dt * P2[axis][2][1] );
	FPF11 = P2[axis][1][1] + dt * (-P2[axis][2][1] - P2[axis][1][2] + dt * P2[axis][2][2] );
	FPF12 = P2[axis][1][2] + dt * (-P2[axis][2][2] );
	FPF20 = P2[axis][2][0] + dt * ( P2[axis][2][1] );
	FPF21 = P2[axis][2][1] + dt * (-P2[axis][2][2] );
	FPF22 = P2[axis][2][2];
	// P = F*P*F' + Q
	P2[axis][0][0] = FPF00 + Q2[axis][POSITION];
	P2[axis][0][1] = FPF01;
	P2[axis][0][2] = FPF02;
	P2[axis][1][0] = FPF10;
	P2[axis][1][1] = FPF11 + Q2[axis][SPEED];
	P2[axis][1][2] = FPF12;
	P2[axis][2][0] = FPF20;
	P[axis][2][1] = FPF21;
	P[axis][2][2] = FPF22 + Q2[axis][BIAIS];
}


void e_kalman_predict_hf(int axis,float accel_meas, float dt)
{
	float FPF00,FPF01,FPF10,FPF11;
	
	board->estimation.state[axis][POSITION] = board->estimation.state[axis][POSITION] + dt * board->estimation.state[axis][SPEED] + dt*dt*accel_meas;
	board->estimation.state[axis][POSITION] = board->estimation.state[axis][POSITION] + dt * board->estimation.state[axis][SPEED];
	
	FPF00 = P[axis][0][0] + dt * (P[axis][1][0] + P[axis][0][1] + dt * P[axis][1][1]);
	FPF01 = P[axis][0][1] + dt * P[axis][1][1];
	FPF10 = P[axis][1][0] + dt * P[axis][1][1];
	FPF11 = P[axis][1][1];
	
	P[axis][0][0] = FPF00 + Q[axis][POSITION];
	P[axis][0][1] = FPF01;
	P[axis][1][0] = FPF10;
	P[axis][1][1] = FPF11 + Q[axis][BIAIS];
	
	/*************************************************************************************/
	FPF00 = P2[axis][0][0] + dt * (P2[axis][1][0] + P2[axis][0][1] + dt * P2[axis][1][1]);
	FPF01 = P2[axis][0][1] + dt * P2[axis][1][1];
	FPF10 = P2[axis][1][0] + dt * P2[axis][1][1];
	FPF11 = P2[axis][1][1];
	
	P2[axis][0][0] = FPF00 + Q2[axis][POSITION];
	P2[axis][0][1] = FPF01;
	P2[axis][1][0] = FPF10;
	P2[axis][1][1] = FPF11 + Q2[axis][BIAIS];
}

//--------------------------------UPDATE----------------------------

/*
H = [1 0 0];
R = 0.1;

// state residual
y = rangemeter - H * Xm;

// covariance residual
S = H*Pm*H' + R;

// kalman gain
K = Pm*H'*inv(S);

// update state
Xp = Xm + K*y;

// update covariance
Pp = Pm - K*H*Pm;
*/
void e_kalman_update_position (int axis,float position_meas)
{
	float y,y2,S,K1,K2,K3;
	float P11,P12,P13,P21,P22,P23,P31,P32,P33;

	y = position_meas - board->estimation.state[axis][POSITION];
	
	
	S = P[axis][0][0] + R[axis];
	K1 = P[axis][0][0] * 1/S;
	K2 = P[axis][1][0] * 1/S;
	K3 = P[axis][2][0] * 1/S;

	board->estimation.state[axis][POSITION] = board->estimation.state[axis][POSITION] + K1 * y;
	board->estimation.state[axis][SPEED] = board->estimation.state[axis][SPEED] + K2 * y;
	board->estimation.state[axis][BIAIS] = board->estimation.state[axis][BIAIS] + K3 * y;

	P11 = (1. - K1) * P[axis][0][0];
	P12 = (1. - K1) * P[axis][0][1];
	P13 = (1. - K1) * P[axis][0][2];
	P21 = -K2 * P[axis][0][0] + P[axis][1][0];
	P22 = -K2 * P[axis][0][1] + P[axis][1][1];
	P23 = -K2 * P[axis][0][2] + P[axis][1][2];
	P31 = -K3 * P[axis][0][0] + P[axis][2][0];
	P32 = -K3 * P[axis][0][1] + P[axis][2][1];
	P33 = -K3 * P[axis][0][2] + P[axis][2][2];

	P[axis][0][0] = P11;
	P[axis][0][1] = P12;
	P[axis][0][2] = P13;
	P[axis][1][0] = P21;
	P[axis][1][1] = P22;
	P[axis][1][2] = P23;
	P[axis][2][0] = P31;
	P[axis][2][1] = P32;
	P[axis][2][2] = P33;
	
	/*************************************************************************************/
	y = position_meas - board->imu1.attitude.localPosition.pos[axis];
	
	S = P2[axis][0][0] + R2[axis];
	K1 = P2[axis][0][0] * 1/S;
	K2 = P2[axis][1][0] * 1/S;
	K3 = P2[axis][2][0] * 1/S;
	
	board->imu1.attitude.localPosition.pos[axis] = board->imu1.attitude.localPosition.pos[axis] + K1 * y;
	board->imu1.attitude.vel[axis] = board->imu1.attitude.vel[axis] + K2 * y;
	board->imu1.attitude.be[axis+ACC_OFFSET] = board->imu1.attitude.be[axis+ACC_OFFSET] + K3 * y;
	
	P11 = (1. - K1) * P2[axis][0][0];
	P12 = (1. - K1) * P2[axis][0][1];
	P13 = (1. - K1) * P2[axis][0][2];
	P21 = -K2 * P2[axis][0][0] + P2[axis][1][0];
	P22 = -K2 * P2[axis][0][1] + P2[axis][1][1];
	P23 = -K2 * P2[axis][0][2] + P2[axis][1][2];
	P31 = -K3 * P2[axis][0][0] + P2[axis][2][0];
	P32 = -K3 * P2[axis][0][1] + P2[axis][2][1];
	P33 = -K3 * P2[axis][0][2] + P2[axis][2][2];

	P2[axis][0][0] = P11;
	P2[axis][0][1] = P12;
	P2[axis][0][2] = P13;
	P2[axis][1][0] = P21;
	P2[axis][1][1] = P22;
	P2[axis][1][2] = P23;
	P2[axis][2][0] = P31;
	P2[axis][2][1] = P32;
	P2[axis][2][2] = P33;
}


/*
H = [0 1 0];
R = 0.1;

// state residual
yd = vz - H * Xm;

// covariance residual
S = H*Pm*H' + R;

// kalman gain
K = Pm*H'*inv(S);

// update state
Xp = Xm + K*yd;

// update covariance
Pp = Pm - K*H*Pm;
*/
void e_kalman_update_speed(int axis,float speed_meas)
{
	
	float yd,S,K1,K2,K3;
	float P11,P12,P13,P21,P22,P23,P31,P32,P33;
	
	yd = speed_meas - board->estimation.state[axis][SPEED];
	
	
	S = P[axis][1][1] + R[axis];
	K1 = P[axis][0][1] * 1/S;
	K2 = P[axis][1][1] * 1/S;
	K3 = P[axis][2][1] * 1/S;

	board->estimation.state[axis][POSITION] = board->estimation.state[axis][POSITION] + K1 * yd;
	board->estimation.state[axis][SPEED] = board->estimation.state[axis][SPEED] + K2 * yd;
	board->estimation.state[axis][BIAIS] = board->estimation.state[axis][BIAIS] + K3 * yd;

	P11 = -K1 * P[axis][1][0] + P[axis][0][0];
	P12 = -K1 * P[axis][1][1] + P[axis][0][1];
	P13 = -K1 * P[axis][1][2] + P[axis][0][2];
	P21 = (1. - K2) * P[axis][1][0];
	P22 = (1. - K2) * P[axis][1][1];
	P23 = (1. - K2) * P[axis][1][2];
	P31 = -K3 * P[axis][1][0] + P[axis][2][0];
	P32 = -K3 * P[axis][1][1] + P[axis][2][1];
	P33 = -K3 * P[axis][1][2] + P[axis][2][2];

	P[axis][0][0] = P11;
	P[axis][0][1] = P12;
	P[axis][0][2] = P13;
	P[axis][1][0] = P21;
	P[axis][1][1] = P22;
	P[axis][1][2] = P23;
	P[axis][2][0] = P31;
	P[axis][2][1] = P32;
	P[axis][2][2] = P33;
	
	/*************************************************************************************/
	yd = speed_meas - board->imu1.attitude.vel[axis];
	
	S = P2[axis][1][1] + R2[axis];
	K1 = P2[axis][0][1] * 1/S;
	K2 = P2[axis][1][1] * 1/S;
	K3 = P2[axis][2][1] * 1/S;
	
	board->imu1.attitude.localPosition.pos[axis] = board->imu1.attitude.localPosition.pos[axis] + K1 * yd;
	board->imu1.attitude.vel[axis] = board->imu1.attitude.vel[axis] + K2 * yd;
	board->imu1.attitude.be[axis+ACC_OFFSET] = board->imu1.attitude.be[axis+ACC_OFFSET] + K3 * yd;
	
	P11 = -K1 * P2[axis][1][0] + P2[axis][0][0];
	P12 = -K1 * P2[axis][1][1] + P2[axis][0][1];
	P13 = -K1 * P2[axis][1][2] + P2[axis][0][2];
	P21 = (1. - K2) * P2[axis][1][0];
	P22 = (1. - K2) * P2[axis][1][1];
	P23 = (1. - K2) * P2[axis][1][2];
	P31 = -K3 * P2[axis][1][0] + P2[axis][2][0];
	P32 = -K3 * P2[axis][1][1] + P2[axis][2][1];
	P33 = -K3 * P2[axis][1][2] + P2[axis][2][2];

	P2[axis][0][0] = P11;
	P2[axis][0][1] = P12;
	P2[axis][0][2] = P13;
	P2[axis][1][0] = P21;
	P2[axis][1][1] = P22;
	P2[axis][1][2] = P23;
	P2[axis][2][0] = P31;
	P2[axis][2][1] = P32;
	P2[axis][2][2] = P33;
}


/*
  * Update horizontal position
  H = [1 0];
  R = 0.1;
  // state residual
  y = pos_measurement - H * Xm;
  // covariance residual
  S = H*Pm*H' + R;
  // kalman gain
  K = Pm*H'*inv(S);
  // update state
  Xp = Xm + K*y;
  // update covariance
  Pp = Pm - K*H*Pm;
*/
void e_kalman_update_position_hf(int axis,float position_meas)
{
	float posxy,S,K1,K2;
	float P11,P12,P21,P22;
	
	posxy = position_meas - board->estimation.state[axis][POSITION];
	
	
	S = P[axis][0][0] + R[axis];
	K1 = P[axis][0][0] * 1/S;
	K2 = P[axis][1][0] * 1/S;
	
	board->estimation.state[axis][POSITION] = board->estimation.state[axis][POSITION] + K1 * posxy;
	board->estimation.state[axis][SPEED] = board->estimation.state[axis][SPEED] + K2 * posxy;
	
	P11 = (1. - K1) * P[axis][0][0];
	P12 = (1. - K1) * P[axis][0][1];
	P21 = -K2 * P[axis][0][0] + P[axis][1][0];
	P22 = -K2 * P[axis][0][1] + P[axis][1][1];

	P[axis][0][0] = P11;
	P[axis][0][1] = P12;
	P[axis][1][0] = P21;
	P[axis][1][1] = P22;
	
	/*************************************************************************************/
	posxy = position_meas - board->imu1.attitude.localPosition.pos[axis];
	
	S = P2[axis][0][0] + R2[axis];
	K1 = P2[axis][0][0] * 1/S;
	K2 = P2[axis][1][0] * 1/S;
	
	board->imu1.attitude.localPosition.pos[axis] = board->imu1.attitude.localPosition.pos[axis] + K1 * posxy;
	board->imu1.attitude.vel[axis] = board->imu1.attitude.vel[axis] + K2 * posxy;
	
	P11 = (1. - K1) * P2[axis][0][0];
	P12 = (1. - K1) * P2[axis][0][1];
	P21 = -K2 * P2[axis][0][0] + P2[axis][1][0];
	P22 = -K2 * P2[axis][0][1] + P2[axis][1][1];

	P2[axis][0][0] = P11;
	P2[axis][0][1] = P12;
	P2[axis][1][0] = P21;
	P2[axis][1][1] = P22;
}

/*
 * Update horizontal velocity

  H = [0 1];
  R = 0.1;
  
  // state residual
  yd = vx - H * Xm;
  
  // covariance residual
  S = H*Pm*H' + R;
  
  // kalman gain
  K = Pm*H'*inv(S);
  
  // update state
  Xp = Xm + K*yd;
  
  // update covariance
  Pp = Pm - K*H*Pm;
*/
void e_kalman_update_speed_hf(int axis,float speed_meas)
{
		float velxy,S,K1,K2;
		float P11,P12,P21,P22;
		
		velxy = speed_meas - board->estimation.state[axis][SPEED];
		
		S = P[axis][1][1] + R[axis];
		K1 = P[axis][0][1] * 1/S;
		K2 = P[axis][1][1] * 1/S;

		board->estimation.state[axis][POSITION] = board->estimation.state[axis][POSITION] + K1 * velxy;
		board->estimation.state[axis][SPEED] = board->estimation.state[axis][SPEED] + K2 * velxy;

		P11 = -K1 * P[axis][1][0] + P[axis][0][0];
		P12 = -K1 * P[axis][1][1] + P[axis][0][1];
		P21 = (1.0 - K2) * P[axis][1][0];
		P22 = (1.0 - K2) * P[axis][1][1];

		P[axis][0][0] = P11;
		P[axis][0][1] = P12;
		P[axis][1][0] = P21;
		P[axis][1][1] = P22;
		
		/*************************************************************************************/
		velxy = speed_meas - board->imu1.attitude.vel[axis];
		
		S = P2[axis][1][1] + R2[axis];
		K1 = P2[axis][0][1] * 1/S;
		K2 = P2[axis][1][1] * 1/S;
		
		board->imu1.attitude.localPosition.pos[axis] = board->imu1.attitude.localPosition.pos[axis] + K1 * velxy;
		board->imu1.attitude.vel[axis] = board->imu1.attitude.vel[axis] + K2 * velxy;
		
		P11 = -K1 * P2[axis][1][0] + P2[axis][0][0];
		P12 = -K1 * P2[axis][1][1] + P2[axis][0][1];
		P21 = (1.0 - K2) * P2[axis][1][0];
		P22 = (1.0 - K2) * P2[axis][1][1];

		P2[axis][0][0] = P11;
		P2[axis][0][1] = P12;
		P2[axis][1][0] = P21;
		P2[axis][1][1] = P22;
}

//--------------------------------GLOBAL--------------------------
void estimator_loop()
{
	float pos_x,pos_y,pos_z;
	double	latitude_rad;
	float time_before_baro;
	uint32_t actual_time;
	
	global_position_t global_gps_position;
	local_coordinates_t local_coordinates;
	//static uint32_t dt_baro,time_before_baro;
	
	if (!board->init_gps_position)
	{
		init_pos_gps();
	}

	if(!filter_init_delta_t)
	{
		filter_init_delta_t = true;
		prev_time = get_micros();
	}else{
		actual_time = get_micros();
		board->estimation.delta_t_filter = (float)(actual_time-prev_time);
		prev_time = actual_time;
		board->estimation.delta_t_filter /= 1000000.0;
		
		e_predict(&(board->imu1.attitude.qe),board->imu1.attitude.acc_bf,board->estimation.delta_t_filter);
		
		//Check new values from GPS/Baro, if yes, update
		if (newValidGpsMsg() && board->init_gps_position && (board->simulation_mode == 0))
		{
			
			//dbg_print("Update step\n");
			
			//longitude latitude to x,y position
			//latitude_rad= ((double) (board->GPS_data.latitude-init_lat))*DEGREE_TO_RADIAN; //in rad E+7
			//pos_y= (float) (((double) (board->GPS_data.longitude-init_long)*EARTH_RADIUS)*DEGREE_TO_RADIAN*(COS_PI_4-COS_PI_4*(board->GPS_data.latitude*DEGREE_TO_RADIAN*0.0000001-PI_4)-COS_PI_4*0.5*(board->GPS_data.latitude*DEGREE_TO_RADIAN*0.0000001-PI_4)*(board->GPS_data.latitude*DEGREE_TO_RADIAN*0.0000001-PI_4)));//Taylor 2nd order cos() approx
			//pos_x= (float) (latitude_rad*EARTH_RADIUS);
			//pos_z= -board->GPS_data.altitude+init_alt;
			
			global_gps_position.longitude = board->GPS_data.longitude;
			global_gps_position.latitude = board->GPS_data.latitude;
			global_gps_position.altitude = board->GPS_data.altitude;
			
			local_coordinates = global_to_local_position(global_gps_position,board->imu1.attitude.localPosition.origin);
			
			//get delay of GPS measure
			//do prediction up to the corresponding delay
			
			e_kalman_update_position(X,local_coordinates.pos[X]);
			e_kalman_update_position(Y,local_coordinates.pos[Y]);
			//e_kalman_update_position_hf(X,local_coordinates.pos[X]);
			//e_kalman_update_position_hf(Y,local_coordinates.pos[Y]);
			e_kalman_update_position(Z,local_coordinates.pos[Z]);
			
			e_kalman_update_speed(X,board->GPS_data.northSpeed);
			e_kalman_update_speed(Y,board->GPS_data.eastSpeed);
			//e_kalman_update_speed_hf(X,board->GPS_data.northSpeed);
			//e_kalman_update_speed_hf(Y,board->GPS_data.eastSpeed);
			e_kalman_update_speed(Z,board->GPS_data.verticalSpeed);
			
			//Continue the prediction until actual time
		}
		
		/*	if (newBaroValue())
		{
		dt_baro=get_millis()-time_before_baro;
		baro=get_pressure_data_slow();
		e_kalman_update_position(Z,baro->altitude,dt_baro);
		time_before_baro=get_millis();
		}	*/
		//}
	}
}