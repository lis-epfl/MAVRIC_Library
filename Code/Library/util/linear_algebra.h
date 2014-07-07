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
 * \file linear_algebra.h
 * 
 * Matrix inversion functions
 */


#ifndef LINEAR_ALGEBRA_H_
#define LINEAR_ALGEBRA_H_

#include "small_matrix.h"

/**
 * \brief Compute the invert of a 2 by 2 Matrix
 *
 * \param m the 2 by 2 matrix
 *
 * \return the inverted 2 by 2 matrix
 */
matrix_2x2_t inv2(matrix_2x2_t m);

/**
 * \brief Compute the invert of a 3 by 3 Matrix
 *
 * \param m the 3 by 3 matrix
 *
 * \return the inverted 3 by 3 matrix
 */
matrix_3x3_t inv3(matrix_3x3_t m);

/**
 * \brief Compute the invert of a 4 by 4 Matrix
 *
 * \param m the 4 by 4 matrix
 *
 * \return the inverted 4 by 4 matrix
 */
matrix_4x4_t inv4(matrix_4x4_t m);

/**
 * \brief Compute the invert of a 5 by 5 Matrix
 *
 * \param m the 5 by 5 matrix
 *
 * \return the inverted 5 by 5 matrix
 */
matrix_5x5_t inv5(matrix_5x5_t m);

/**
 * \brief Compute the invert of a 6 by 6 Matrix
 *
 * \param m the 6 by 6 matrix
 *
 * \return the inverted 6 by 6 matrix
 */
matrix_6x6_t inv6(matrix_6x6_t m);

#endif /* LINEAR_ALGEBRA_H_ */