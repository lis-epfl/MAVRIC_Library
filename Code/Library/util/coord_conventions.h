/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file coord_conventions.h
 * 
 * Coordinate conventions
 */ 


#ifndef COORD_CONVENTIONS_H_
#define COORD_CONVENTIONS_H_

#ifdef __cplusplus
extern "C" 
{
#endif

#include "compiler.h"
#include "maths.h"

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
} Aero_Attitude_t;


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
Aero_Attitude_t coord_conventions_quat_to_aero(UQuat_t qe);


/**
 * \brief      	Converts roll, pitch and yaw angles (with the aeronautics conventions) to an attitude quaternion 
 * 
 * \param aero 	Aero attitude
 * 
 * \return     	Attitude quaternion
 */
UQuat_t coord_conventions_quaternion_from_aero(Aero_Attitude_t aero);


/**
 * \brief      Computes the yaw angle from an attitude quaternion
 * 
 * \param qe   Attitude quaternion
 * 
 * \return     Yaw angle in radians
 */
float coord_conventions_get_yaw(UQuat_t qe);


#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_H_ */