/*
 * linear_algebra.h
 *
 * Created: 13/02/2014 14:44:09
 *  Author: sfx
 */ 


#ifndef LINEAR_ALGEBRA_H_
#define LINEAR_ALGEBRA_H_

#include "small_matrix.h"


matrix_2x2_t inv2(matrix_2x2_t m);

matrix_3x3_t inv3(matrix_3x3_t m);

matrix_4x4_t inv4(matrix_4x4_t m);

matrix_5x5_t inv5(matrix_5x5_t m);

matrix_6x6_t inv6(matrix_6x6_t m);



#endif /* LINEAR_ALGEBRA_H_ */