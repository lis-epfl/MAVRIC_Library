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
 * \file coord_conventions.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Coordinate conventions
 *
 ******************************************************************************/


#ifndef COORD_CONVENTIONS_H_
#define COORD_CONVENTIONS_H_

#ifdef __cplusplus
extern "C" 
{
#endif

#include <stdint.h>
#include "quaternions.h"

#define EARTH_RADIUS 6378137.0f   // radius of the earth in meters

#define rad_to_deg(input) (input*180.0f/PI)
#define deg_to_rad(input) (input*PI/180.0f)


/**
 * \brief 		Global position
 */
typedef struct 
{
	double longitude;			///<	Current longitude
	double latitude;			///<	Current latitude
	float altitude;				///<	Current altitude
	float heading;				///<	Current heading
	uint32_t timestamp_ms;		///<	Timestamp (milliseconds)
} global_position_t;


/**
 * \brief Local coordinates
 */
typedef struct 
{
	float pos[3];				///<	Current position x, y and z
	float heading;				///<	Current heading (equal to heading in global frame)
	global_position_t origin;	///<	Global coordinates of the local frame's origin (ie. local (0, 0, 0) expressed in the global frame)
	uint32_t timestamp_ms;		///<	Timestamp (milliseconds)
} local_coordinates_t;


/*
 * \brief 		Attitude with aeronautics convention
 * 
 * \details 	Expressed with the NED frame: X front, Y right, Z down. 
 * 				The rotations are done in the following order: 
 * 				- first around the local yaw axis, 
 * 				- then around new local pitch axis, 
 * 				- and finally around the new roll axis
*/
typedef struct 
{
	float rpy[3];	///<	Roll pitch and yaw angles in radians
} aero_attitude_t;


/**
 * \brief 			Convert local NED coordinates to global GPS coordinates 
 * \details 		(relative to origin given in local coordinate frame)
 * 
 * \param input 	Local position
 * 
 * \return 			Global position
 */
global_position_t coord_conventions_local_to_global_position(local_coordinates_t input);


/**
 * \brief 			Convert global GPS coordinates to local NED coordinate
 * 
 * \param position 	Global position
 * \param origin 	Global coordinates of the local frame's origin
 * 
 * \return 			Local position
 */
local_coordinates_t coord_conventions_global_to_local_position(global_position_t position, global_position_t origin);


/**
 * \brief      Converts an attitude quaternion to roll, pitch and yaw angles with the aeronautics conventions
 * 
 * \param qe   Attitude quaternion
 *
 * \return     Aero attitude
 */
aero_attitude_t coord_conventions_quat_to_aero(quat_t qe);


/**
 * \brief      	Converts roll, pitch and yaw angles (with the aeronautics conventions) to an attitude quaternion 
 * 
 * \param aero 	Aero attitude
 * 
 * \return     	Attitude quaternion
 */
quat_t coord_conventions_quaternion_from_aero(aero_attitude_t aero);


/**
 * \brief      Computes the yaw angle from an attitude quaternion
 * 
 * \param qe   Attitude quaternion
 * 
 * \return     Yaw angle in radians
 */
float coord_conventions_get_yaw(quat_t qe);


#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_H_ */