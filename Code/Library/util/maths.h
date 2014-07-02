/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file maths.h
 *
 * Useful math functions
 */

#ifndef MATHS_H
#define MATHS_H

#ifdef __cplusplus
extern "C" 
{
#endif

#include "compiler.h"
#include <math.h>

#define PI 3.141592653589793f

/**
 * \brief 		Unit quaternion
 * 
 * \details  	The quaternions are in the form q = [s, v_1, v_2, v_3]
 */
typedef struct UQuat {
	float s;			///<	Scalar component
	float v[3];			///<	Vector component
} UQuat_t;


/**
 * \brief Conversion from degrees to radians
 */
#define MATH_DEG_TO_RAD (PI/180.)


/**
 * \brief Conversion from radians to degrees
 */
#define MATH_RAD_TO_DEG (180./PI)


/**
 * \brief 			Cross product between two 3-vectors
 * 
 * \param 	u 		Input vector (dim 3)
 * \param 	v  		Input vector (dim 3)
 * \param 	out 	Output vector (dim 3)
 */
#define CROSS(u,v,out) \
	out[0] = u[1] * v[2] - u[2] * v[1];\
	out[1] = u[2] * v[0] - u[0] * v[2];\
	out[2] = u[0] * v[1] - u[1] * v[0];


/**
 * \brief 			Scalar product between two 3-vectors
 * 
 * \param 	v 		Input vector (dim 3)
 * \param 	u 		Input vector (dim 3)
 * 
 * \return 			Scalar product
 */
#define SCP(u,v) \
	(u[0] * v[0] + u[1] * v[1] + u[2] * v[2])


/**
 * @brief 		Quaternion inverse
 * 
 * @param 	q  	Input quaternion
 * 
 * @return  	Inversed quaternion
 */
#define QI(q, out) \
	out.s = q.s;\
	out.v[0] = -q.v[0];\
	out.v[1] = -q.v[1];\
	out.v[2] = -q.v[2];


/**
 * @brief 		Creates a quaternion from a vector of dimension 4, MACRO version
 * 
 * @param 	q 	Output quaternion
 * @param 	s 	Scalar part of the quaternion
 * @param 	v0 	First component of the vector part of the quaternion
 * @param 	v1 	Second component of the vector part of the quaternion
 * @param 	v2 	Third component of the vector part of the quaternion
 */
#define QUAT(q, s, v0, v1, v2) \
	q.s = s;\
	q.v[0] = v0;\
	q.v[1] = v1;\
	q.v[2] = v2;


/**
 * @brief 		Creates a quaternion from a vector of dimension 4
 * 
 * @param 	v 	Array of floats of dimension 4 containing the vector components
 * 
 * @return 		Unit quaternion
 */
UQuat_t static inline quat_from_vector(float v[4]) {
	UQuat_t q;	
	q.s = 0; 
	q.v[0] = v[0]; 
	q.v[1] = v[1]; 
	q.v[2] = v[2];
	return q;
}


/**
 * @brief 			Quaternion multiplication
 * 
 * @param 	q1 		Input quaternion
 * @param 	q2 		Input quaternion
 * @param 	out 	Output quaternion
 */
#define QMUL(q1,q2,out) \
	tmp[0] = q1.v[1] * q2.v[2] - q1.v[2] * q2.v[1];\
	tmp[1] = q1.v[2] * q2.v[0] - q1.v[0] * q2.v[2];\
	tmp[2] = q1.v[0] * q2.v[1] - q1.v[1] * q2.v[0];\
	out.v[0] = q2.s* q1.v[0] + q1.s * q2.v[0] + tmp[0];\
	out.v[1] = q2.s* q1.v[1] + q1.s * q2.v[1] + tmp[1];\
	out.v[2] = q2.s* q1.v[2] + q1.s * q2.v[2] + tmp[2];\
	out.s= q1.s * q2.s - SCP(q1.v, q2.v);


/**
 * @brief 			Square function
 * 
 * @param  	in 		Input variable
 * @return 			Squared variable
 */
#define SQR(in) \
		((in)*(in))


/**
 * @brief 			For any given angle, computes an equivalent angle between -2pi and 2pi 
 * 
 * @param 	angle 	Input angle
 * 
 * @return 			Output angle
 */
float static inline calc_smaller_angle(float angle) {
	float out=angle;
	while (out<-PI) out += 2.0 * PI;
	while (out>=PI) out -= 2.0 * PI;
	return out;
}


/**
 * @brief 		Computes the scalar product of two vectors of dimension 3
 * 
 * @param 	u 	Input vector (dim 3)
 * @param 	v 	Input vector (dim 3)
 * 
 * @return 		Scalar product
 */
float static inline scalar_product(const float u[3], const float v[3])
{
	float scp = (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]);
	return scp;
}


/**
 * @brief 			Computes the cross product of two vectors of dimension 3
 * 
 * @param 	u 		Input vector (dim 3)
 * @param 	v 		Input vector (dim 3)
 * @param 	out 	Output vector (dim 3)
 */
void static inline cross_product(const float u[3], const float v[3], float out[3])
{
	out[0] = u[1] * v[2] - u[2] * v[1];
	out[1] = u[2] * v[0] - u[0] * v[2];
	out[2] = u[0] * v[1] - u[1] * v[0];
}


/**
 * @brief 			Multiplies two unit quaternions
 * 
 * @param 	q1 		Input quaternion
 * @param 	q2 		Input quaternion
 * 
 * @return 			Output quaternion
 */
UQuat_t static inline quat_multi(const UQuat_t q1, const UQuat_t q2)
{
	float tmp[3];
	UQuat_t out;

	tmp[0] = q1.v[1] * q2.v[2] - q1.v[2] * q2.v[1];
	tmp[1] = q1.v[2] * q2.v[0] - q1.v[0] * q2.v[2];
	tmp[2] = q1.v[0] * q2.v[1] - q1.v[1] * q2.v[0];
	
	out.v[0] = q2.s * q1.v[0] + q1.s * q2.v[0] + tmp[0];
	out.v[1] = q2.s * q1.v[1] + q1.s * q2.v[1] + tmp[1];
	out.v[2] = q2.s * q1.v[2] + q1.s * q2.v[2] + tmp[2];
	out.s= q1.s * q2.s - scalar_product(q1.v, q2.v);
	
	return out;
}


/**
 * @brief 		Inverse quaternion
 * 
 * @param 	q 	Input quaternion
 * @return 		Output quaternion
 */
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


/**
 * @brief 			Rotates a vector from global frame to local frame according to an attitude quaternion
 * 
 * @details 		The vector is given in the form of a quaternion with the scalar term equal to 0
 * 
 * @param 	qe 		Attitude quaternion
 * @param 	qvect 	Quaternion containing the vector to be rotated
 * 
 * @return 			Output quaternion
 */
UQuat_t static inline quat_global_to_local(const UQuat_t qe, const UQuat_t qvect)
{
	UQuat_t qinv, qtmp;
	
	qinv = quat_inv(qe);
	qtmp = quat_multi(qinv,qvect);
	qtmp = quat_multi(qtmp,qe);

	return qtmp;
}


/**
 * @brief 			Rotates a vector from local frame to global frame according to an attitude quaternion
 * 
 * @details 		The vector is given in the form of a quaternion with the scalar term equal to 0
 * 
 * @param 	qe 		Attitude quaternion
 * @param 	qvect 	Quaternion containing the vector to be rotated
 * 
 * @return 			Output quaternion
 */
UQuat_t static inline quat_local_to_global(const UQuat_t qe, const UQuat_t qvect)
{
	UQuat_t qinv, qtmp;
	
	qinv = quat_inv(qe);
	qtmp = quat_multi(qe, qvect);
	qtmp = quat_multi(qtmp, qinv);
	
	return qtmp;
}


/**
 * @brief 			Rotates a vector according to a unit quaternion
 * 
 * @details 		This is an optimized implementation that does not require quaternion multiplications
 * 					It should run more than 2 times faster than the standard implementation
 * 
 * @param 	q 		unit quaternion
 * @param 	u 		input vector
 * @param 	v 		rotated vector (output)
 * 
 */
void static inline quat_rotate_vector(const UQuat_t q, const float u[3], float v[3])
{
	float tmp1[3], tmp2[3];

	cross_product(q.v, u, tmp1);
	tmp1[0] = 2 * tmp1[0];
	tmp1[1] = 2 * tmp1[1];
	tmp1[2] = 2 * tmp1[2];

	cross_product(q.v, tmp1, tmp2);
	
	v[0] = u[0] + q.s * tmp1[0] + tmp2[0];
	v[1] = u[1] + q.s * tmp1[1] + tmp2[1];
	v[2] = u[2] + q.s * tmp1[2] + tmp2[2];
}


/**
 * @brief 			Fast newton iteration for approximate square root of numbers close to 1 (for re-normalisation)
 * 
 * @param 	number 	Input value
 * 
 * @return 			Output value
 */
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


/**
 * @brief 			Fast newton iteration for approximate square root of numbers close to 1 (for re-normalisation)
 * 
 * @param 	input 	Input value
 * @return 			Output value
 */
float static inline fast_sqrt_1(float input) {
	if (input<0) 
	{
		return 0.0f;
	}

	float result = 1.0f;

	result = 0.5f * (result + (input / result));
	result = 0.5f * (result + (input / result));

	return result;
}


/**
 * @brief 		Computes the squared norm of a vector (dim 3)
 * 
 * @param 	u 	Input vector
 * @return 		Squared norm
 */
float static inline vector_norm_sqr(float u[3])
{
	float norm = scalar_product(u, u);
	return norm;
}


/**
 * @brief 		Computes the norm of a vector of dimension 3
 * 
 * @param 	u 	Input vector (dim 3)
 * @return 		Norm of the vector
 */
float static inline vector_norm(float u[3])
{
	return fast_sqrt(vector_norm_sqr(u));
}


/**
 * @brief 		Normalizes a vector of dimension 3
 * 
 * @param 	v 	Input vector (dim 3)
 * @param 	u 	Output vector (dim 3)
 */
void static inline vector_normalize(float v[3], float u[3])
{
	int i;
	float norm = vector_norm(v);
	for (i = 0; i < 3; ++i)
	{
		u[i] = v[i] / norm;
	}
}


/**
 * @brief 		Normalises a quaternion
 * 
 * @param 	q 	Input quaternion
 * @return 		Unit quaternion
 */
static inline UQuat_t quat_normalise(const UQuat_t q) 
{
	UQuat_t result;
	
	float snorm= SQR(q.s) + SQR(q.v[0]) + SQR(q.v[1]) + SQR(q.v[2]);

	if (snorm >0.0000001) 
	{
		float norm = fast_sqrt(snorm);
		result.s = q.s / norm;
		result.v[0] = q.v[0] / norm;		
		result.v[1] = q.v[1] / norm;		
		result.v[2] = q.v[2] / norm;
	}
	else
	{
		result.s = 1.0;
		result.v[0] = 0.0;
		result.v[1] = 0.0;
		result.v[2] = 0.0;
	}

	return result;
}


/**
 * @brief 		Returns the absolute value of a floating point value
 * 
 * @param 	a 	Input value
 * 
 * @return 		Absolute value
 */
static inline float f_abs(const float a)
{
	if (a >= 0.0)
	{
		return a;
	}
	else
	{
		return -a;
	}
}


/**
 * @brief 		Returns the minimum value of two floating point values
 * 
 * @param 	a 	Input value
 * @param 	b 	Input value
 * 
 * @return 		Minimum value
 */
static inline float f_min(const float a, const float b)
{
	if (a <= b)
	{
		return a;
	}
	else
	{
		return b;
	}
}


/**
 * @brief 		Returns the maximum value of two floating point values
 * 
 * @param 	a 	Input value
 * @param 	b 	Input value
 * 
 * @return 		Maximum value
 */
static inline float f_max(const float a, const float b){
	if (a >= b)
	{
		return a;
	}
	else
	{
		return b;
	}
}


/**
 * @brief 					Clip a variable
 * 
 * @details 				Acts like a saturation between -clip_value and clip_value
 * 
 * @param 	input_value 	Input value
 * @param 	clip_value 		Clip value
 * 
 * @return 					Clipped value
 */
static float inline clip(float input_value, float clip_value) {
	
	if (input_value>clip_value)  return clip_value;     
	if (input_value<-clip_value) return -clip_value; 
	return input_value;
}


/**
 * @brief 	 				A function to attenuate values bellow a certain threshold
 * 
 * @param 	x 				Input value
 * @param 	soft_zone_width Width of the soft zone
 * 
 * @return 					Output value
 */
static float inline soft_zone(float x, float soft_zone_width) 
{
	if (soft_zone_width < 0.0000001) 
	{	
		return x;
	} 
	else 
	{
		return x * x * x / ( SQR(soft_zone_width) + SQR(x) );
	}
};


/**
 * @brief 		Sigmoid function
 * 
 * @param 	x 	Input value
 * @return 		Output value
 */
static float inline sigmoid(float x) 
{
	return (x / fast_sqrt(1 + SQR(x)));
};


/**
 * @brief 		Center window 2 (?)
 * 
 * @param 	x 	Input value
 * @return 		Output value
 */
static float inline center_window_2(float x) 
{
	return 1.0 / (1 + SQR(x));
}


/**
 * @brief 		Center window 4 (?)
 * 
 * @param 	x 	Input value
 * @return 		Output value
 */
static float inline center_window_4(float x) 
{
	return 1.0 / (1 + SQR(SQR(x)));
}


/**
 * @brief 		Median filter
 * 
 * @details  	Returns the median of 3 floating point values
 * 
 * @param 	a 	Input value
 * @param 	b 	Input value
 * @param 	c 	Input value
 * 
 * @return 		Output value
 */
static float inline median_filter_3x(float a, float b, float c) {
	float middle;
	
	if ((a <= b) && (a <= c)) 
	{
		middle = (b <= c) ? b : c;
	}
	else if ((b <= a) && (b <= c))
	{
		middle = (a <= c) ? a : c;
	} 
	else 
	{
	   middle = (a <= b) ? a : b;
	}

	return middle;

}


/**
 * @brief 		Interpolation
 * 
 * @details 	With known y1 = f(x1) and y2 = f(x2), this function will interpolate f to compute f(x)
 * 
 * @param 	x 	Target abscissa
 * @param 	x1  First known abscissa
 * @param 	x2 	Second known abscissa
 * @param 	y1 	First known ordinate
 * @param 	y2 	Second known ordinate
 * 
 * @return 		Interpolated value
 */
static inline float interpolate(float x, float x1, float x2, float y1, float y2)
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


#ifdef __cplusplus
}
#endif


#endif	/*	MATHS_H	 */
