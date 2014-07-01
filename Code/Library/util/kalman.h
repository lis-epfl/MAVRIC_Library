/**
 * 2D kalman filter
 *
 * The MAV'RIC Framework
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the MAV'RIC Framework.
 */

#ifndef KALMAN_H_
#define KALMAN_H_


#ifdef __cplusplus
extern "C" 
{
#endif


#include "small_matrix.h"
#include "linear_algebra.h"

/**
 * @brief Kalman filter 
 */
typedef struct kalman_filter_2D_t 
{
	matrix_2x2_t system_model;			///<	Model matrix
	matrix_2x2_t control_model;			///<	Control matrix
	matrix_2x2_t observation_model;		///<	Observation matrix
	matrix_2x2_t noise_prediction;		///<	Model noise matrix
	matrix_2x2_t noise_measurement;		///<	Measurement noise  matrix
	matrix_2x2_t covariance;			///<	Covariance matrix
	vector_2_t   state;					///<	State vector
} kalman_filter_2D_t;


/**
 * \brief 	Kalman prediction step
 * 
 * \param 	kalman 		Pointer to kalman structure
 * \param 	control 	Control vector
 */
void kalman_2D_prediction(kalman_filter_2D_t *kalman, vector_2_t control);


/**
 * \brief 	Kalman update step
 * 
 * \param 	kalman 			Pointer to kalman structure
 * \param 	measurement 	Measurement vector
 */
void kalman_2D_update(kalman_filter_2D_t *kalman, vector_2_t measurement);


#ifdef __cplusplus
}
#endif


#endif /* KALMAN_H_ */