/*
 * kalman.h
 *
 * Created: 16/02/2014 20:30:24
 *  Author: sfx
 */ 


#ifndef KALMAN_H_
#define KALMAN_H_


#include "small_matrix.h"
#include "linear_algebra.h"

typedef struct kalman_filter_2D_t {
	matrix_2x2_t system_model, control_model, observation_model;
	matrix_2x2_t noise_prediction, noise_measurement;
	matrix_2x2_t covariance;
	vector_2_t   state;
} kalman_filter_2D_t;

void kalman_2D_prediction(kalman_filter_2D_t *kalman, vector_2_t control);

void kalman_2D_update(kalman_filter_2D_t *kalman, vector_2_t measurement);



#endif /* KALMAN_H_ */