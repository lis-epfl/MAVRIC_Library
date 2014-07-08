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
*	\file ins.h
*	\brief Header file for the INS module
*/

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
