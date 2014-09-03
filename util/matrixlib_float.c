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
 * \file matrixlib_float.c
 * 
 * \author MAV'RIC Team
 *   
 * \brief Source file for the floating point versions of some matrix operation 
 * functions
 *
 ******************************************************************************/


#include <math.h>
#include "matrixlib_float.h"

//---------------------------------
// Public functions implementation
//---------------------------------

/*!
*	Initializes a matrix with zeros
*   sizes : A(n1 x n2)
*/
float* matf_zeros(int32_t n1, int32_t n2, float* A)
{
    int32_t i;
    for(i=0; i < n1*n2; i++) 
        A[i] = 0.0f;
    
    return A;
}

/*!
*	Sets the "start"th to "stop"th diagonal elements of a matrix to the value "val"   (indices for this function start at 1)
*   sizes : A(n1 x n2)
*/
float* matf_diag(int32_t n1, int32_t n2, float* A, float val, int32_t start, int32_t stop)
{
    int32_t i;
    if(start < 1) start = 1;
    for(i=start-1; i < stop; i++) 
        A[(n2+1)*i] = val;
    
    return A;
}

/*!
*	Copies a matrix B = A
*   sizes : A(n1 x n2), B(n1 x n2)
*/
float* matf_copy(int32_t n1, int32_t n2, float* A, float* B)
{
    int32_t i;
    for(i=0; i < n1*n2; i++) 
        B[i] = A[i];
    
    return B;
}

/*!
*	3D vector cross product: c = a x b
*/
float* matf_cross(float* a, float* b, float* c)
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
    
    return c;
}

/*!
*	Gets the vector of standard deviations from the matrix A's diagonal
*   sizes : A(n x n), v(n x 1)
*/
float* matf_std(int32_t n, float* A, float* v)
{
    int32_t i;
    for(i=0; i < n; i++) 
        v[i] = sqrtf(A[i*(n+1)]);
    
    return v;
}

/*!
*	Copies a submatrix p1xp2 of matrix A at position (ra,ca) to matrix B at position (rb,cb)
*       B(rb,cb,p1 x p2) = A(ra,ca,p1 x p2)
*   sizes : A(a1 x a2), B(b1 x b2)
*/
float* matf_copy_part(float* A, int32_t a1, int32_t a2, int32_t ra, int32_t ca, int32_t p1, int32_t p2, float* B, int32_t b1, int32_t b2, int32_t rb, int32_t cb)
{
    int32_t row;
    int32_t col;
    
    for(row = 0; row < p2; row ++)
    for(col = 0; col < p1; col ++)
         B[(rb+row)*b1+col+cb] = A[(ra+row)*a1+col+ca];

    return B;
}

/*!
*	Gets the norm of a vector A of length n
*/
float matf_norm(int32_t n, float* A)
{
    int32_t i;
    float norm = 0.0f;
    for(i=0; i < n; i++) 
        norm += A[i]*A[i];
    
    return sqrtf(norm);
}

/*!
*	Gets the sum of a vector A of length n
*/
float matf_sum(int32_t n, float* A)
{
    int32_t i;
    float sum = 0.0f;
    for(i=0; i < n; i++) 
        sum += A[i];
    
    return sum;
}


/*!
*	Adds two matrices A+B = C
*   sizes : A(n1 x n2), B(n1 x n2), C(n1 x n2)
*/
float* matf_add(int32_t n1, int32_t n2, float* A, float* B, float* C)
{
    int32_t i;
    for(i=0; i < n1*n2; i++) 
        C[i] = A[i] + B[i];
    
    return C;
}


/*!
*	Multiply matrix by factor B = c * A
*   sizes : A(n1 x n2), B(n1 x n2)
*/
float* matf_multiply_factor(int32_t n1, int32_t n2, float* A, float* B, float c)
{
    int32_t i;
    for(i=0; i < n1*n2; i++) 
        B[i] = c*A[i];
    
    return B;
}

/*!
*	Subtracts two matrices A-B = C
*   sizes : A(n1 x n2), B(n1 x n2), C(n1 x n2)
*/
float* matf_sub(int32_t n1, int32_t n2, float* A, float* B, float* C)
{
    int32_t i;
    for(i=0; i < n1*n2; i++) 
        C[i] = A[i] - B[i];
    
    return C;
}


/*!
*	Transposes a matrix B = A'
*   sizes : A(n1 x n2), B(n2 x n1)
*/
float* matf_tr(int32_t n1, int32_t n2, float* A, float* B)
{
    int32_t a, col, row;
    int32_t b = 0;
    
    for(row=0; row < n2; row++) 
    {
        a = row;
        for(col=0; col < n1; col++)
        {
            B[b] = A[a];
            a += n2;
            b++;
        }
    }   
    return B;
}

/*!
*	Multiplies two matrices A.B = C
*   sizes : A(n1 x n12), B(n12 x n2), C(n1 x n2)
*/
float* matf_multiply(int32_t n1, int32_t n12, int32_t n2, float* A, float* B, float* C)
{
    int32_t a, b, col, row, el;
    int32_t c = 0;
    
    for(row=0; row < n1; row++) 
    for(col=0; col < n2; col++)
    {
        a = row*n12;
        b = col;
        C[c] = 0.0f;
        for(el=0; el < n12; el++)  
        {
            C[c] = C[c] + A[a]*B[b];
            a++;
            b += n2;
        }
        c++;
    }
    
    return C;
}


/*!
*	Multiplies two matrices A.B' = C, but uses the transpose of the second matrix
*   sizes : A(n1 x n12), B(n2 x n12), C(n1 x n2)
*/
float* matf_multiply_Bt(int32_t n1, int32_t n12, int32_t n2, float* A, float* B, float* C)
{
    int32_t a, b, col, row, el;
    int32_t c = 0;
    
    for(row=0; row < n1; row++) 
    for(col=0; col < n2; col++)
    {
        a = row*n12;
        b = col*n12;
        C[c] = 0.0f;
        for(el=0; el < n12; el++)  
        {
            C[c] = C[c] + A[a]*B[b];
            a++;
            b++;
        }
        c++;
    }
    
    return C;
}

//float* matf_invert(int32_t numRowsCols, float* dstM, float* srcM)
//{
	//arm_matrix_instance_f32 in = {numRowsCols, numRowsCols, srcM};
	//arm_matrix_instance_f32 out = {numRowsCols, numRowsCols, dstM};
//
	//arm_mat_inverse_f32(&in, &out);
//
	//return NULL;
//}

