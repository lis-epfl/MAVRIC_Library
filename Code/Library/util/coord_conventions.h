/**
 * Coordinate conventions
 *
 * The MAV'RIC Framework
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the MAV'RIC Framework.
 */

#ifndef COORD_CONVENTIONS_H_
#define COORD_CONVENTIONS_H_

#ifdef __cplusplus
extern "C" 
{
#endif

#include "compiler.h"
#include "maths.h"

#define EARTH_RADIUS 6378137.0   // radius of the earth in meters

#define rad_to_deg(input) (input*180.0/PI)
#define deg_to_rad(input) (input*PI/180.0)


/**
 * \brief 					Global position
 * 
 * \param longitude    		Current longitude
 * \param latitude     		Current latitude
 * \param altitude     		Current altitude
 * \param heading      		Current heading
 * \param timestamp_ms 		Timestamp (milliseconds)
 */
typedef struct 
{
	double longitude;
	double latitude;
	float altitude;
	float heading;
	uint32_t timestamp_ms;
} global_position_t;


/**
 * \brief Local coordinates
 * 
 * \param pos          	Current position x, y and z
 * \param heading      	Current heading (equal to heading in global frame)
 * \param origin       	Global coordinates of the local frame's origin (ie. local (0, 0, 0) expressed in the global frame)
 * \param timestamp_ms 	Timestamp (milliseconds)
 */
typedef struct 
{
	float pos[3];
	float heading;
	global_position_t origin;
	uint32_t timestamp_ms;
} local_coordinates_t;


/*
 * \brief 		Attitude with aeronautics convention
 * \details 	Expressed with the NED frame: X front, Y right, Z down. 
 * 				The rotations are done in the following order: 
 * 				- first around the local yaw axis, 
 * 				- then around new local pitch axis, 
 * 				- and finally around the new roll axis
 *							
 * \param rpy	Roll pitch and yaw angles in radians
*/
typedef struct 
{
	float rpy[3];
} Aero_Attitude_t;


/**
 * \brief 			Convert local NED coordinates to global GPS coordinates 
 * \details 		(relative to origin given in local coordinate frame)
 * 
 * \param input 	Local position
 * 
 * \return 			Global position
 */
global_position_t local_to_global_position(local_coordinates_t input);


/**
 * \brief 			Convert global GPS coordinates to local NED coordinate
 * 
 * \param position 	Global position
 * \param origin 	Global coordinates of the local frame's origin
 * 
 * \return 			Local position
 */
local_coordinates_t global_to_local_position(global_position_t position, global_position_t origin);


/**
 * \brief      Converts an attitude quaternion to roll, pitch and yaw angles with the aeronautics conventions
 * 
 * \param qe   Attitude quaternion
 *
 * \return     Aero attitude
 */
Aero_Attitude_t Quat_to_Aero(UQuat_t qe);


/**
 * \brief      	Converts roll, pitch and yaw angles (with the aeronautics conventions) to an attitude quaternion 
 * 
 * \param aero 	Aero attitude
 * 
 * \return     	Attitude quaternion
 */
UQuat_t quaternion_from_aero(Aero_Attitude_t aero);


/**
 * \brief      Computes the yaw angle from an attitude quaternion
 * 
 * \param qe   Attitude quaternion
 * 
 * \return     Yaw angle in radians
 */
float get_yaw(UQuat_t qe);


#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_H_ */