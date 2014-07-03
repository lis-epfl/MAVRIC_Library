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
 * \file maths_utils.h
 *
 * Useful math functions
 */


#ifndef MATH_UTIL_H_
#define MATH_UTIL_H_

#ifdef __cplusplus
extern "C" 
{
#endif


// TODO: move to maths


/**
 * \brief 		Macro to compute the maximum of two variables
 * 
 * \param 	a 	Input variable
 * \param 	b 	Input variable
 * 
 * \return 	Maximum
 */
#define math_util_fmax(a,b) ((a>b)?a:b)		


/**
 * \brief 		Macro to compute the minimum of two variables
 * 
 * \param 	a 	Input variable
 * \param 	b 	Input variable
 * 
 * \return 	Minimum
 */
#define math_util_fmin(a,b) ((a<b)?a:b)


#ifdef __cplusplus
}
#endif

#endif /* MATH_UTIL_H_ */