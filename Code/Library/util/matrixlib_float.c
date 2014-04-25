/*
  Copyright (C) 2009. LIS Laboratory, EPFL, Lausanne

  This file is part of Aeropic.

  Aeropic is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 2.1 of the License, or
  (at your option) any later version.

  Aeropic is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with Aeropic.  If not, see <http://www.gnu.org/licenses/>.
*/
/*!
*	\file matrixlib_float.c
*	\brief Source file for the floating point versions of some matrix operation functions
*
*/

#include <math.h>
#include "matrixlib_float.h"

//---------------------------------
// Public functions implementation
//---------------------------------

/*!
*	Initializes a matrix with zeros
*   sizes : A(n1 x n2)
*/
float* matf_zeros(int n1, int n2, float* A)
{
    int i;
    for(i=0; i < n1*n2; i++) 
        A[i] = 0.0;
    
    return A;
}

/*!
*	Sets the "start"th to "stop"th diagonal elements of a matrix to the value "val"   (indices for this function start at 1)
*   sizes : A(n1 x n2)
*/
float* matf_diag(int n1, int n2, float* A, float val, int start, int stop)
{
    int i;
    if(start < 1) start = 1;
    for(i=start-1; i < stop; i++) 
        A[(n2+1)*i] = val;
    
    return A;
}

/*!
*	Copies a matrix B = A
*   sizes : A(n1 x n2), B(n1 x n2)
*/
float* matf_copy(int n1, int n2, float* A, float* B)
{
    int i;
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
float* matf_std(int n, float* A, float* v)
{
    int i;
    for(i=0; i < n; i++) 
        v[i] = sqrtf(A[i*(n+1)]);
    
    return v;
}

/*!
*	Copies a submatrix p1xp2 of matrix A at position (ra,ca) to matrix B at position (rb,cb)
*       B(rb,cb,p1 x p2) = A(ra,ca,p1 x p2)
*   sizes : A(a1 x a2), B(b1 x b2)
*/
float* matf_copy_part(float* A, int a1, int a2, int ra, int ca, int p1, int p2, float* B, int b1, int b2, int rb, int cb)
{
    int row;
    int col;
    
    for(row = 0; row < p2; row ++)
    for(col = 0; col < p1; col ++)
         B[(rb+row)*b1+col+cb] = A[(ra+row)*a1+col+ca];

    return B;
}

/*!
*	Gets the norm of a vector A of length n
*/
float matf_norm(int n, float* A)
{
    int i;
    float norm = 0.0;
    for(i=0; i < n; i++) 
        norm += A[i]*A[i];
    
    return sqrtf(norm);
}

/*!
*	Gets the sum of a vector A of length n
*/
float matf_sum(int n, float* A)
{
    int i;
    float sum = 0.0;
    for(i=0; i < n; i++) 
        sum += A[i];
    
    return sum;
}


/*!
*	Adds two matrices A+B = C
*   sizes : A(n1 x n2), B(n1 x n2), C(n1 x n2)
*/
float* matf_add(int n1, int n2, float* A, float* B, float* C)
{
    int i;
    for(i=0; i < n1*n2; i++) 
        C[i] = A[i] + B[i];
    
    return C;
}


/*!
*	Multiply matrix by factor B = c * A
*   sizes : A(n1 x n2), B(n1 x n2)
*/
float* matf_multiply_factor(int n1, int n2, float* A, float* B, float c)
{
    int i;
    for(i=0; i < n1*n2; i++) 
        B[i] = c*A[i];
    
    return B;
}

/*!
*	Subtracts two matrices A-B = C
*   sizes : A(n1 x n2), B(n1 x n2), C(n1 x n2)
*/
float* matf_sub(int n1, int n2, float* A, float* B, float* C)
{
    int i;
    for(i=0; i < n1*n2; i++) 
        C[i] = A[i] - B[i];
    
    return C;
}


/*!
*	Transposes a matrix B = A'
*   sizes : A(n1 x n2), B(n2 x n1)
*/
float* matf_tr(int n1, int n2, float* A, float* B)
{
    int a, col, row;
    int b = 0;
    
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
float* matf_multiply(int n1, int n12, int n2, float* A, float* B, float* C)
{
    int a, b, col, row, el;
    int c = 0;
    
    for(row=0; row < n1; row++) 
    for(col=0; col < n2; col++)
    {
        a = row*n12;
        b = col;
        C[c] = 0.0;
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
float* matf_multiply_Bt(int n1, int n12, int n2, float* A, float* B, float* C)
{
    int a, b, col, row, el;
    int c = 0;
    
    for(row=0; row < n1; row++) 
    for(col=0; col < n2; col++)
    {
        a = row*n12;
        b = col*n12;
        C[c] = 0.0;
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

//float* matf_invert(int numRowsCols, float* dstM, float* srcM)
//{
	//arm_matrix_instance_f32 in = {numRowsCols, numRowsCols, srcM};
	//arm_matrix_instance_f32 out = {numRowsCols, numRowsCols, dstM};
//
	//arm_mat_inverse_f32(&in, &out);
//
	//return NULL;
//}

