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
 * \file curvace.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Driver for the cylindrical curvace
 *
 ******************************************************************************/


#ifndef CURVACE_H_
#define CURVACE_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include "quaternions.h"
#include "ahrs.h"
#include "mavlink_stream.h"

#define CURVACE_NB_OF 108


typedef struct 
{
	int16_t x;
	int16_t y;	
} curvace_raw_of_vector_t;


typedef union
{
	int16_t data[2 * CURVACE_NB_OF];
	struct
	{
		curvace_raw_of_vector_t left_hemisphere[ CURVACE_NB_OF / 2 ];
		curvace_raw_of_vector_t right_hemisphere[ CURVACE_NB_OF / 2 ];
	};
	struct
	{
		curvace_raw_of_vector_t all[ CURVACE_NB_OF ];
	};
} curvace_raw_data_t;


typedef struct
{
	float x;
	float y;
} curvace_of_vector_t;


typedef union
{
	float data[ 2 * CURVACE_NB_OF ];
	struct
	{
		curvace_of_vector_t left_hemisphere[ CURVACE_NB_OF / 2 ];
		curvace_of_vector_t right_hemisphere[ CURVACE_NB_OF / 2 ];
	};
	struct
	{
		curvace_of_vector_t all[ CURVACE_NB_OF ];
	};
} curvace_data_t;


/**
 * \brief Pixels coordinates top left is (0, 0), bottom right is (41, 14)
 */
typedef struct
{
	uint8_t x;		///< x coordinate
	uint8_t y;		///< y coordinate
} curvace_pixel_coordinates_t;


/**
 * \brief Direction at which a ROI is pointed
 */
typedef struct
{
	float azimuth;		///< azimuth in radians
	float elevation;	///< elevation in radians
} curvace_viewing_direction_t;


/**
 * \brief List of viewing direction for each Region of interest (ROI)
 */
 typedef union
 {
 	float data[ CURVACE_NB_OF * 2 ];
 	struct
 	{
 		curvace_viewing_direction_t left_hemisphere[ CURVACE_NB_OF / 2 ];
 		curvace_viewing_direction_t right_hemisphere[ CURVACE_NB_OF / 2 ];
 	};
 	curvace_viewing_direction_t all[ CURVACE_NB_OF ];
 } curvace_roi_coord_t;


/**
 * \brief Link between gyro rates and rotational optic flow
 * 
 * OF_rotation = A * gyro
 * 
 * [ OF_x ] = 	[Arx	Apx		Ayx] * 	[ roll_rate ]
 * [ OF_y ]		[Ary	Apy		Ayy]	[ pitch_rate]
 * 										[ yaw_rate 	]
 */
typedef struct
{
	float Arx;
	float Apx;
	float Ayx;
	float Ary;
	float Apy;
	float Ayy;
} derotation_matrix_t;


typedef struct
{
	float elevation;
	float azimuth;
} curvace_scale_factor_t;

typedef union
{
	float data[ 2 * CURVACE_NB_OF ];
	curvace_scale_factor_t  scale[ CURVACE_NB_OF ];
} curvace_calibration_factor_t;


typedef union
{
	float data[ CURVACE_NB_OF * 6 ];
	struct
	{
		derotation_matrix_t left_hemisphere[ CURVACE_NB_OF / 2 ];
		derotation_matrix_t right_hemisphere[ CURVACE_NB_OF / 2 ];
	};
	derotation_matrix_t all[ CURVACE_NB_OF ];
} curvace_calibration_matrix_t;


typedef struct
{
	curvace_data_t 					of;
	curvace_raw_data_t 				raw_of;
	curvace_roi_coord_t				roi_coord;
	curvace_calibration_matrix_t 	calib_matrix;
	curvace_calibration_factor_t	calib_factor;
	quat_t 							orientation; 	///< unused
	const ahrs_t* 					ahrs;
	const mavlink_stream_t* 		mavlink_stream;
} curvace_t;



void curvace_init(curvace_t* curvace, const ahrs_t* ahrs, const mavlink_stream_t* mavlink_stream);


void curvace_update(curvace_t* curvace);


void curvace_send_telemetry(const curvace_t* curvace);

#ifdef __cplusplus
	}
#endif

#endif