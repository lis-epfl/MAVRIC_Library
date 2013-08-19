/*
 * coord_conventions.c
 *
 * Created: 13/02/2013 16:47:26
 *  Author: Julien, Felix
 */ 

#include "coord_conventions.h"
#include <math.h>

// convert local NED coordinates to global GPS coordinates (relative to origin given in local coordinate frame)
global_position_t local_to_global_position(local_coordinates_t input){
	global_position_t output;
	output.latitude = input.origin.latitude  + rad_to_deg( input.pos[0] *2.0 / (PI * EARTH_RADIUS));
	output.longitude= input.origin.longitude + rad_to_deg( input.pos[1] *2.0 / (PI * EARTH_RADIUS*cos(deg_to_rad(output.latitude))));
	output.altitude = input.origin.altitude  - input.pos[2]; 
	return output;
}

// convert a global position into a local coordinate frame around the given global origin
local_coordinates_t global_to_local_position(global_position_t position, global_position_t origin) {
	local_coordinates_t output;
	output.origin=origin;
	double small_radius=cos(deg_to_rad(position.latitude))*EARTH_RADIUS;
	output.pos[0]=  sin(deg_to_rad(position.latitude-origin.latitude))*EARTH_RADIUS;
	output.pos[1]=  sin(deg_to_rad(position.longitude-origin.longitude))*small_radius;
	output.pos[2]= -(position.altitude - origin.altitude);
	return output;
}


Aero_Attitude_t Quat_to_Aero(UQuat_t qe) {
	Aero_Attitude_t aero;

	aero.rpy[0]= atan2(2*(qe.s*qe.v[0] + qe.v[1]*qe.v[2]) , (qe.s*qe.s - qe.v[0]*qe.v[0] - qe.v[1]*qe.v[1] + qe.v[2]*qe.v[2])); 
	aero.rpy[1]=-asin(2*(qe.v[0]*qe.v[2] - qe.s*qe.v[1]));
	aero.rpy[2]= atan2(2*(qe.s*qe.v[2] + qe.v[0]*qe.v[1]) , (qe.s*qe.s + qe.v[0]*qe.v[0] - qe.v[1]*qe.v[1] - qe.v[2]*qe.v[2]));
	
	return aero;
}

Schill_Attitude_t Quat_to_Schill(UQuat_t qe) {
	Schill_Attitude_t schill;
	Aero_Attitude_t aero;
	
	aero=Quat_to_Aero(qe);
	
	schill.rpy[0]=aero.rpy[1];
	schill.rpy[1]=aero.rpy[0];
	schill.rpy[2]=aero.rpy[2];

	return schill;
}