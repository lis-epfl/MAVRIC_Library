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
 * \file kalman.c
 * 
 * 2D kalman filter
 */


#include "kalman.h"


void kalman_2D_prediction(kalman_filter_2D_t *kalman, vector_2_t control) 
{
	kalman->state =     vadd2( 	mvmul2(kalman->system_model, kalman->state), 
								mvmul2(kalman->control_model, control));

	kalman->covariance= madd2(  mmul2( 	mmul2(	kalman->system_model, 
												kalman->covariance), 
										trans2(kalman->system_model)), 
								kalman->noise_prediction);
}

void kalman_2D_update(kalman_filter_2D_t *kalman, vector_2_t measurement)
{
	vector_2_t innovation = vsub2(	measurement, 
									mvmul2(kalman->observation_model, kalman->state));
	
	matrix_2x2_t innovation_covariance = madd2(	mmul2( 	mmul2(kalman->observation_model, kalman->covariance),
											   			trans2(kalman->observation_model)), 
											  	kalman->noise_measurement);
	
	matrix_2x2_t kalman_gain = mmul2(	mmul2(kalman->covariance, trans2(kalman->observation_model)), 
										inv2(innovation_covariance));
	
	kalman->state = vadd2(kalman->state, mvmul2(kalman_gain, innovation));

	kalman->covariance = mmul2(	msub2(ident_2x2, mmul2(kalman_gain, kalman->observation_model)), 
								kalman->covariance);
}
