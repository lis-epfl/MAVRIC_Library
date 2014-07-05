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
 * Useful functions for vectors
 */


#ifndef VECTORS_H_
#define VECTORS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "compiler.h"
#include "maths.h"


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
 * \brief 		Computes the scalar product of two vectors of dimension 3
 * 
 * \param 	u 	Input vector (dim 3)
 * \param 	v 	Input vector (dim 3)
 * 
 * \return 		Scalar product
 */
float static inline vectors_scalar_product(const float u[3], const float v[3]) //maths_scalar_product(const float u[3], const float v[3])
{
	float scp = (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]);
	return scp;
}


/**
 * \brief 			Computes the cross product of two vectors of dimension 3
 * 
 * \param 	u 		Input vector (dim 3)
 * \param 	v 		Input vector (dim 3)
 * \param 	out 	Output vector (dim 3)
 */
void static inline vectors_cross_product(const float u[3], const float v[3], float out[3]) //maths_cross_product(const float u[3], const float v[3], float out[3])
{
	out[0] = u[1] * v[2] - u[2] * v[1];
	out[1] = u[2] * v[0] - u[0] * v[2];
	out[2] = u[0] * v[1] - u[1] * v[0];
}


/**
 * \brief 		Computes the squared norm of a vector (dim 3)
 * 
 * \param 	u 	Input vector
 * \return 		Squared norm
 */
float static inline vectors_norm_sqr(float u[3]) //maths_vector_norm_sqr(float u[3])
{
	float norm = vectors_scalar_product(u, u);
	return norm;
}


/**
 * \brief 		Computes the norm of a vector of dimension 3
 * 
 * \param 	u 	Input vector (dim 3)
 * \return 		Norm of the vector
 */
float static inline vectors_norm(float u[3]) //maths_vector_norm(float u[3])
{
	return maths_fast_sqrt(vectors_norm_sqr(u));
}


/**
 * \brief 		Normalizes a vector of dimension 3
 * 
 * \param 	v 	Input vector (dim 3)
 * \param 	u 	Output vector (dim 3)
 */
void static inline vectors_normalize(float v[3], float u[3]) //maths_vector_normalize(float v[3], float u[3])
{
	int i;
	float norm = vectors_norm(v);
	for (i = 0; i < 3; ++i)
	{
		u[i] = v[i] / norm;
	}
}


#ifdef __cplusplus
}
#endif

#endif /* VECTORS_H_ */
