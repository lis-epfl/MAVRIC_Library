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
 * \file orca.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This file computes a collision-free trajectory for the ORCA algorithm
 *
 ******************************************************************************/


#ifndef ORCA_H__
#define ORCA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "neighbor_selection.h"
#include "imu.h"
#include "position_estimation.h"
#include <stdbool.h>
#include <stdint.h>

#define ORCA_TIME_STEP_MILLIS 10.0f
#define TIME_HORIZON 12.0f
#define MAXSPEED 4.5f
#define RVO_EPSILON 0.0001f

/**
 * \brief The 3D plane structure
 */
typedef struct{
	float normal[3];									///< The normal vector to the plane
	float point[3];										///< A point of the plane
} plane_t;

/**
 * \brief The 3D line structure
 */
typedef struct{
	float direction[3];									///< The direction vector of a line
	float point[3];										///< A point of the line
} line_t;

typedef struct
{
	neighbors_t* 				neighbors;				///< The pointer to the neighbor structure
	const position_estimator_t* position_estimator;		///< The pointer to the position estimation structure
	const imu_t* 				imu;					///< The pointer to the IMU structure
	const ahrs_t* 				ahrs;					///< The pointer to the attitude estimation structure
} orca_t;

/**
 * \brief	Initialize the ORCA module
 *
 * \param	neighbors					The pointer to the neighbor data structure
 * \param	position_estimator			The pointer to the position structure
 * \param	imu							The pointer to the IMU structure
 * \param	ahrs						The pointer to the attitude estimation structure
 */
void orca_init(orca_t *orca, neighbors_t *neighbors, const position_estimator_t *position_estimator, const imu_t *imu, const ahrs_t *ahrs);

/**
 * \brief	Computes the new collision-free velocity
 *
 * \param	orca				pointer to the orca_t struct
 * \param	optimal_velocity	a 3D array
 * \param	new_velocity		the 3D output array
 */
void orca_compute_new_velocity(orca_t* orca, float optimal_velocity[], float new_velocity[]);

/**
 * \brief	computes the solution of a 1D linear program
 * 
 * \param	planes				the array of all planes
 * \param	index				the starting index of the plane
 * \param	line				the intersecting line between the two conflicting planes
 * \param	optimal_velocity	a 3D array
 * \param	new_velocity		the 3D output array
 * \param	direction_opt		whether we are solving a 4D or a 3D linear program
 *
 * \return	whether the linear program has a solution or not
 */
bool orca_linear_program1(plane_t planes[], uint8_t index, line_t line, float max_speed, float optimal_velocity[], float new_velocity[], bool direction_opt);

/**
 * \brief	computes the solution of a 2D linear program
 *
 * \param	planes				the array of all planes
 * \param	ind					the index of the plane
 * \param	max_speed			the norm of the max velocity
 * \param	optimal_velocity	a 3D array
 * \param	new_velocity		the 3D output array
 * \param	direction_opt		whether we are solving a 4D or a 3D linear program
 *
 * \return	whether the linear program has a solution or not
 */
bool orca_linear_program2(plane_t planes[], uint8_t ind, float max_speed, float optimal_velocity[], float new_velocity[], bool direction_opt);

/**
 * \brief	computes the solution of a 3D linear program
 *
 * \param	planes				the array of all planes
 * \param	plane_size			the number of planes
 * \param	optimal_velocity	a 3D array
 * \param	max_speed			the norm of the max velocity
 * \param	new_velocity		the 3D output array
 * \param	direction_opt		whether we are solving a 4D or a 3D linear program
 *
 * \return the number of the last plane to be evaluated, if smaller than the number of planes, linear program is infeasible
 */
float orca_linear_program3(plane_t planes[], uint8_t plane_size, float optimal_velocity[], float max_speed, float new_velocity[], bool direction_opt);

/**
 * \brief	computes the solution of a 3D linear program
 *
 * \param	planes				the array of all planes
 * \param	plane_size			the number of planes
 * \param	ind					the index of the plane for which the 3D linear program is infeasible
 * \param	max_speed			the norm of the max velocity
 * \param	new_velocity		the 3D output array
 */
void orca_linearProgram4(plane_t planes[], uint8_t plane_size, uint8_t ind, float max_speed, float new_velocity[]);

#ifdef __cplusplus
}
#endif

#endif // ORCA_H__