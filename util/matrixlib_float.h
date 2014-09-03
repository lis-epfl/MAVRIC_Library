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
 * \file matrixlib_float.h
 * 
 * \author MAV'RIC Team
 *   
 * \brief Source file for the floating point versions of some matrix operation 
 * functions
 *
 ******************************************************************************/


#ifndef __MF_H__
#define __MF_H__

#include <stdint.h>

#ifndef USE_MATF
	#define USE_MATF			0
#endif

/** Errors module can throw */
enum matf_errors
{
	MATF_ERROR_BASE = 0x3F00,
};

//----------------------------
// Public function prototypes
//----------------------------

float* matf_zeros(int32_t, int32_t, float *);
float* matf_diag(int32_t, int32_t, float *, float, int32_t, int32_t);
float* matf_std(int32_t, float*, float*);
float* matf_copy(int32_t, int32_t, float *, float *);
float* matf_cross(float* a, float* b, float* c);
float* matf_copy_part(float *, int32_t, int32_t, int32_t, int32_t, int32_t, int32_t, float *, int32_t, int32_t, int32_t, int32_t);
float  matf_norm(int32_t, float *);
float  matf_sum (int32_t, float *);
float* matf_add(int32_t, int32_t, float*, float*, float*);
float* matf_multiply_factor(int32_t n1, int32_t n2, float* A, float* B, float c);
float* matf_sub(int32_t, int32_t, float*, float*, float*);
float* matf_tr (int32_t, int32_t, float*, float*);
float* matf_multiply(int32_t, int32_t, int32_t, float*, float*, float*);
float* matf_multiply_Bt(int32_t, int32_t, int32_t, float*, float*, float*);
// float* matf_invert(int32_t numRowsCols, float* dstM, float* srcM);

#endif
