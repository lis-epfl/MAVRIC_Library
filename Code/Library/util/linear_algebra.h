/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file linear_algebra.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief Matrix inversion functions
 *
 ******************************************************************************/


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