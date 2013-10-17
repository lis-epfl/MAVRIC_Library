/*
 * coord_conventions.h
 *
 * Created: 13/02/2013 16:47:26
 *  Author: Julien
 */ 

#ifndef COORD_CONVENTIONS_H_
#define COORD_CONVENTIONS_H_

#include "compiler.h"
#include <math.h>
#include "maths.h"

#define EARTH_RADIUS 6378137.0   // radius of the earth in meters


#define rad_to_deg(input) (input*180.0/PI)
#define deg_to_rad(input) (input*PI/180.0)

typedef struct {
	double longitude;
	double latitude;
	float altitude;
	float heading;
	uint32_t timestamp_ms;
} global_position_t;

typedef struct {
	float pos[3];
	float heading;
	global_position_t origin;
	uint32_t timestamp_ms;
} local_coordinates_t;


// convert local NED coordinates to global GPS coordinates (relative to origin given in local coordinate frame)
global_position_t local_to_global_position(local_coordinates_t input);

// convert a global position into a local coordinate frame around the given global origin
local_coordinates_t global_to_local_position(global_position_t position, global_position_t origin);

/*
 * Aeronautics convention : X front
 *							Y right
 *							Z down
*/
typedef struct {
	float rpy[3];
} Aero_Attitude_t;



Aero_Attitude_t Quat_to_Aero(UQuat_t qe);

float get_yaw(UQuat_t qe);

#endif /* STABILISATION_H_ */