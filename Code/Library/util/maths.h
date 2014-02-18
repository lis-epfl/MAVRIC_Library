#ifndef MATHS_H
#define MATHS_H

#include "compiler.h"
#include <math.h>
#define PI 3.141592653589793

typedef struct UQuat {
	float s;
	float v[3];
} UQuat_t;


#define CROSS(u,v,out) \
	out[0] = u[1] * v[2] - u[2]*v[1];\
	out[1] = u[2] * v[0] - u[0]*v[2];\
	out[2] = u[0] * v[1] - u[1]*v[0];

#define SCP(u,v) \
	(u[0]*v[0]+u[1]*v[1]+u[2]*v[2])

#define QI(q, out) \
	out.s = q.s;\
	out.v[0] = -q.v[0];\
	out.v[1] = -q.v[1];\
	out.v[2] = -q.v[2];

#define QUAT(q, s, v0, v1, v2) \
	q.s=s; q.v[0]=v0; q.v[1]=v1; q.v[2]=v2;

UQuat_t static inline quat_from_vector(float *v) {
	UQuat_t q;	
	q.s=0; 
	q.v[0]=v[0]; 
	q.v[1]=v[1]; 
	q.v[2]=v[2];
	return q;
}

#define QMUL(q1,q2,out) \
	tmp[0] = q1.v[1] * q2.v[2] - q1.v[2]*q2.v[1];\
	tmp[1] = q1.v[2] * q2.v[0] - q1.v[0]*q2.v[2];\
	tmp[2] = q1.v[0] * q2.v[1] - q1.v[1]*q2.v[0];\
	out.v[0] = q2.s*q1.v[0] + q1.s *q2.v[0] +tmp[0];\
	out.v[1] = q2.s*q1.v[1] + q1.s *q2.v[1] +tmp[1];\
	out.v[2] = q2.s*q1.v[2] + q1.s *q2.v[2] +tmp[2];\
	out.s= q1.s*q2.s - SCP(q1.v, q2.v);

#define SQR(in) \
		((in)*(in))



float static inline calc_smaller_angle(float angle) {
	float out=angle;
	while (out<-PI) out+=2.0*PI;
	while (out>=PI) out-=2.0*PI;
	return out;
}


float static inline scalar_product(const float u[], const float v[])
{
	float scp = (u[0]*v[0]+u[1]*v[1]+u[2]*v[2]);
	return scp;
}


void static inline cross_product(const float u[], const float v[], float out[])
{
	out[0] = u[1] * v[2] - u[2] * v[1];
	out[1] = u[2] * v[0] - u[0] * v[2];
	out[2] = u[0] * v[1] - u[1] * v[0];
}


UQuat_t static inline quat_multi(const UQuat_t q1, const UQuat_t q2)
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

UQuat_t static inline quat_inv(const UQuat_t q)
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

UQuat_t static inline quat_global_to_local(const UQuat_t qe, const UQuat_t qvect)
{
	UQuat_t qinv, qtmp;
	
	qinv = quat_inv(qe);
	qtmp = quat_multi(qinv,qvect);
	qtmp = quat_multi(qtmp,qe);

	return qtmp;
}

UQuat_t static inline quat_local_to_global(const UQuat_t qe, const UQuat_t qvect)
{
	UQuat_t qinv, qtmp;
	
	qinv = quat_inv(qe);
	qtmp = quat_multi(qe, qvect);
	qtmp = quat_multi(qtmp, qinv);
	
	return qtmp;
}


// fast newton iteration for approximate square root of numbers close to 1 (for re-normalisation)
float static inline fast_sqrt(float number) {
	long i;
	float x, y;
	const float f = 1.5F;

	x = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;
	i  = 0x5f3759df - ( i >> 1 );
	y  = * ( float * ) &i;
	y  = y * ( f - ( x * y * y ) );
	y  = y * ( f - ( x * y * y ) ); // repeat newton iteration for more accuracy
	return number * y;
}


// fast newton iteration for approximate square root of numbers close to 1 (for re-normalisation)
float static inline fast_sqrt_1(float input) {
	if (input<0) {
		//dbg_print("negative root");
		return 0.0F;
	}
	float result=1.0F;
	result=0.5F*(result+(input/result));
	result=0.5F*(result+(input/result));
	//result=0.5*(result+(input/result));
	//result=0.5*(result+(input/result));
	return result;
}

float static inline vector_norm_sqr(float u[])
{
	float norm = scalar_product(u,u);
	return norm;
}

float static inline vector_norm(float u[])
{
	return fast_sqrt(vector_norm_sqr(u));
}

void static inline vector_normalize(float v[], float u[])
{
	int i;
	float norm = vector_norm(v);
	for (i = 0; i < 3; ++i)
	{
		u[i] = v[i] / norm;
	}
}

static inline UQuat_t quat_normalise(const UQuat_t q) {
	UQuat_t result={.s=1.0, .v={0.0, 0.0, 0.0} };
	float snorm= SQR(q.s) + SQR(q.v[0]) + SQR(q.v[1]) + SQR(q.v[2]);
	if (snorm >0.0000001) {
		float norm=fast_sqrt(snorm);
		result.s=q.s/norm;
		result.v[0]=q.v[0]/norm;		result.v[1]=q.v[1]/norm;		result.v[2]=q.v[2]/norm;

	}
	return result;
}

static inline float f_abs(const float a){
	if (a >= 0.0)
	{
		return a;
	}else{
		return -a;
	}
}

static inline float f_min(const float a, const float b){
	if (a <= b)
	{
		return a;
	}else{
		return b;
	}
}

static inline float f_max(const float a, const float b){
	if (a >= b)
	{
		return a;
	}else{
		return b;
	}
}


static float inline clip(float input_value, float clip_value) {
	
	if (input_value>clip_value)  return clip_value;     
	if (input_value<-clip_value) return -clip_value; 
	return input_value;
}


static float inline soft_zone(float x, float soft_zone_width) {
	if (soft_zone_width<0.0000001) return x; 
	else {
		return (x*x*x/(SQR(soft_zone_width) + SQR(x)));
	}
};

static float inline sigmoid(float x) {
	return (x/fast_sqrt(1+SQR(x)));
};

static float inline center_window_2(float x) {
	return 1.0/(1+SQR(x));
}

static float inline center_window_4(float x) {
	return 1.0/(1+SQR(SQR(x)));
}


static float inline median_filter_3x(float a, float b, float c) {
	float middle;
	if ((a <= b) && (a <= c)) {
		middle = (b <= c) ? b : c;
	}
	else if ((b <= a) && (b <= c))
	{
		middle = (a <= c) ? a : c;
	} else {
	   middle = (a <= b) ? a : b;
	}
	return middle;

}


static float interpolate(float x, float x1, float x2, float y1, float y2)
{
	if (x1 == x2)
	{
		return y1;
	}
	else
	{
		float y = y1 + (y2 - y1) * (x - x1) / (x2 - x1); 
		return y;
	}
}

#endif
