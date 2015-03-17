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
 * \author Geraud L'Eplattenier
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
#include "optic_flow.h"
#include "constants.h"	

#define CURVACE_NB_OF 108


/**
 * \brief 	Curvace raw data
 */
typedef union
{
	int16_t data[2 * CURVACE_NB_OF];								///< Raw data as int16
	struct
	{
		raw_of_vector_t left_hemisphere[ CURVACE_NB_OF / 2 ];		///< Left optic flow vectors
		raw_of_vector_t right_hemisphere[ CURVACE_NB_OF / 2 ];		///< Right optic flow vectors
	};																
	struct
	{
		raw_of_vector_t all[ CURVACE_NB_OF ];						///< All optic flow vectors
	};
} curvace_raw_data_t;


/**
 * \brief 	Curvace data
 */
typedef union
{
	float data[ 2 * CURVACE_NB_OF ];							///< Raw data as float
	struct
	{
		of_vector_t left_hemisphere[ CURVACE_NB_OF / 2 ];		///< Left optic flow vectors
		of_vector_t right_hemisphere[ CURVACE_NB_OF / 2 ];		///< Right optic flow vectors
	};
	struct
	{
		of_vector_t all[ CURVACE_NB_OF ];						///< All optic flow vectors
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
 * \brief List of viewing direction for each Region of interest (ROI)
 */
 typedef union
 {
 	float data[ CURVACE_NB_OF * 2 ];								///< Raw data as float
 	struct
 	{
 		viewing_direction_t left_hemisphere[ CURVACE_NB_OF / 2 ];	///< Left viewing directions
 		viewing_direction_t right_hemisphere[ CURVACE_NB_OF / 2 ];	///< Right viewing directions
 	};
 	viewing_direction_t all[ CURVACE_NB_OF ];						///< All viewing directions
 } curvace_roi_coord_t;


/**
 * \brief	Scale factor on horizontal and vertical axis
 */
typedef struct
{
	float elevation;		///< Scale factor along elevation
	float azimuth;			///< Scale factor along azimuth
} curvace_scale_factor_t;


/**
 * \brief 	Calibration, scale factors
 */
typedef union
{
	float data[ 2 * CURVACE_NB_OF ];					///< Raw data as float
	curvace_scale_factor_t  scale[ CURVACE_NB_OF ];		///< Array of scale factors
} curvace_calibration_factor_t;


/**
 * \brief Calibration, derotation matrix
 */
typedef union
{
	float data[ CURVACE_NB_OF * 6 ];									///< Raw data as float
	struct
	{
		derotation_matrix_t left_hemisphere[ CURVACE_NB_OF / 2 ];		///< Derotation matrices for left hemisphere
		derotation_matrix_t right_hemisphere[ CURVACE_NB_OF / 2 ];		///< Derotation matrices for right hemisphere
	};
	derotation_matrix_t all[ CURVACE_NB_OF ];							///< All derotation matrices 
} curvace_calibration_matrix_t;


/**
 * \brief Curvace data structure
 */
typedef struct
{
	curvace_data_t 					of;						///< Array of processed optic flow
	curvace_raw_data_t 				raw_of;					///< Array of raw optic flow values
	curvace_roi_coord_t				roi_coord;				///< Coordinates of regions of interests
	curvace_calibration_matrix_t 	calib_matrix;			///< Calibration matrices
	curvace_calibration_factor_t	calib_factor;			///< Calibration factors
	float							scale_factor_simple;	///< Temporary replacement for calib_factor with a single value for all OF vectors
	constants_on_off_t				do_derotation;			///< Indicates whether derotation should be performed (ON/ OFF)
	float							LPF;					///< Low pass filter
	float							derot_factor;			///< Derotation factor
	quat_t 							orientation; 			///< unused
	const ahrs_t* 					ahrs;					///< Pointer to attitude estimation
} curvace_t;


/**
 * \brief 	Initialisation
 * 
 * \param 	curvace 	Pointer to data structure
 * \param 	ahrs 		Pointer to attitude estimation
 */
void curvace_init(curvace_t* curvace, const ahrs_t* ahrs);


/**
 * \brief 	Main update function
 * 
 * \param 	curvace 	Pointer to data structure
 */
void curvace_update(curvace_t* curvace);


#ifdef __cplusplus
	}
#endif

#endif