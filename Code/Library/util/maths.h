#ifndef MATHS_H
#define MATHS_H

#include "compiler.h"


typedef struct UQuat {
	float s;
	float v[3];
} UQuat_t;


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

UQuat_t static inline quat_global_to_local(UQuat_t qe, UQuat_t qvect)
{
	UQuat_t qinv, qtmp;
	
	qinv = quat_inv(qe);
	qtmp = quat_multi(qinv,qvect);
	qtmp = quat_multi(qtmp,qe);

	return qtmp;
}

UQuat_t static inline quat_local_to_global(UQuat_t qe, UQuat_t qvect)
{
	UQuat_t qinv, qtmp;
	
	qinv = quat_inv(qe);
	qtmp = quat_multi(qe, qvect);
	qtmp = quat_multi(qtmp, qinv);
	
	return qtmp;
}

// fast newton iteration for approximate square root
float static inline fast_sqrt(float input) {
	if (input<0) {
		//dbg_print("negative root");
		return 0.0;
	}
	float result=1.0;
	result=0.5*(result+(input/result));
	result=0.5*(result+(input/result));
	result=0.5*(result+(input/result));
	result=0.5*(result+(input/result));
	return result;
}

static inline UQuat_t quat_normalise(UQuat_t q) {
	UQuat_t result={.s=1.0, .v={0.0, 0.0, 0.0} };
	float snorm= SQR(q.s) + SQR(q.v[0]) + SQR(q.v[1]) + SQR(q.v[2]);
	if (snorm >0.000000001) {
		float norm=fast_sqrt(SQR(q.v[0]) + SQR(q.v[1]) + SQR(q.v[2]) );
		result.s=q.s/norm;
		result.v[0]=q.v[0]/norm;		result.v[1]=q.v[1]/norm;		result.v[2]=q.v[2]/norm;

	}
	return result;
}


#endif
