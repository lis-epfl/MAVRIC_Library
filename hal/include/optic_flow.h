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
 * \file optic_flow.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Defines types for generic optic flow sensors
 *
 ******************************************************************************/


#ifndef OPTIC_FLOW_H_
#define OPTIC_FLOW_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

/**
 * \brief Raw optic flaw vector
 */
typedef struct 
{
	int16_t x;			///< x component of the raw optic flaw vector
	int16_t y;			///< y component of the raw optic flaw vector
} raw_of_vector_t;


/**
 * \brief Optic flaw vector
 */
typedef struct
{
	float x;			///< x component of the optic flaw vector
	float y;			///< y component of the optic flaw vector
} of_vector_t;


/**
 * \brief Direction at which a ROI is pointed
 */
typedef struct
{
	float azimuth;		///< azimuth in radians
	float elevation;	///< elevation in radians
} viewing_direction_t;


/**
 * \brief Link between gyro rates and rotational optic flow
 * 
 * OF_rotation = A * gyro
 * 
 * [ OF_x ] = 	[Arx	Apx		Ayx] * 	[ roll_rate ]
 * [ OF_y ]	[Ary	Apy		Ayy]	[ pitch_rate]
 * 						[ yaw_rate  ]
 */
typedef struct
{
	float Arx;	///< Links roll rate with horizontal OF (see formula above)
	float Apx;	///< Links pitch rate with horizontal OF (see formula above)
	float Ayx;	///< Links yaw rate with horizontal OF (see formula above)
	float Ary;	///< Links roll rate with vertical OF (see formula above)
	float Apy;	///< Links pitch rate with vertical OF (see formula above)
	float Ayy;	///< Links yaw rate with vertical OF (see formula above)
} derotation_matrix_t;


#ifdef __cplusplus
	}
#endif

#endif /* OPTIC_FLOW_H_ */
