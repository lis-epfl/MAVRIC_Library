/*
 * coord_conventions.c
 *
 * Created: 13/02/2013 16:47:26
 *  Author: Julien, Felix
 */ 

#include "coord_conventions.h"
#include <math.h>
#include "print_util.h"
#include "conf_platform.h"

// convert local NED coordinates to global GPS coordinates (relative to origin given in local coordinate frame)
global_position_t local_to_global_position(local_coordinates_t input){
	global_position_t output;
	//output.latitude = input.origin.latitude  + rad_to_deg( input.pos[0] *2.0 / (PI * EARTH_RADIUS));
	//output.longitude= input.origin.longitude + rad_to_deg( input.pos[1] *2.0 / (PI * EARTH_RADIUS*cos(deg_to_rad(output.latitude))));
	output.latitude = input.origin.latitude  + rad_to_deg( input.pos[0] / EARTH_RADIUS);
	output.longitude= input.origin.longitude + rad_to_deg( input.pos[1] / ( EARTH_RADIUS*cos(deg_to_rad(output.latitude))));
	output.altitude = -input.pos[2] + input.origin.altitude;
	output.heading=input.heading;

	return output;
}

// convert a global position into a local coordinate frame around the given global origin
local_coordinates_t global_to_local_position(global_position_t position, global_position_t origin) {
	local_coordinates_t output;
	output.origin=origin;
	double small_radius=cos(deg_to_rad(position.latitude))*EARTH_RADIUS;
	output.pos[X]=  (float)(sin(deg_to_rad((position.latitude-origin.latitude)))*EARTH_RADIUS);
	output.pos[Y]=  (float)(sin(deg_to_rad((position.longitude-origin.longitude)))*small_radius);
	output.pos[Z]=  (float)(-(position.altitude - origin.altitude));
	output.heading=position.heading;
	
	//dbg_print("global2local: (x1e7): ");
	//dbg_print("lat:(");
	//dbg_print_num(position.latitude*10000000,10);
	//dbg_print(", ");
	//dbg_print_num(origin.latitude*10000000,10);
	//dbg_print(")");
	//dbg_print(", long: (");
	//dbg_print_num(position.longitude*100000000,10);
	//dbg_print(", ");
	//dbg_print_num(origin.longitude*10000000,10);
	//dbg_print(")");
	//dbg_print(", small rad:");
	//dbg_print_num(small_radius*10000000, 10);
	//dbg_print(", d2r_lat:");
	//dbg_print_num(deg_to_rad((position.latitude-origin.latitude))*10000000,10);
	//dbg_print(", sin_lat:");
	//dbg_print_num(sin(deg_to_rad((position.latitude-origin.latitude)))*EARTH_RADIUS*10000000,10);
	//dbg_print(", d2r_long:");
	//dbg_print_num(deg_to_rad((position.longitude-origin.longitude))*10000000,10);
	//dbg_print(", sin_long:");
	//dbg_print_num(sin(deg_to_rad((position.longitude-origin.longitude)))*small_radius*10000000,10);
	//dbg_print("\n");
	
	return output;
}

Aero_Attitude_t Quat_to_Aero(UQuat_t qe) {
	Aero_Attitude_t aero;

	aero.rpy[0]= atan2(2*(qe.s*qe.v[0] + qe.v[1]*qe.v[2]) , (qe.s*qe.s - qe.v[0]*qe.v[0] - qe.v[1]*qe.v[1] + qe.v[2]*qe.v[2])); 
	aero.rpy[1]=-asin(2*(qe.v[0]*qe.v[2] - qe.s*qe.v[1]));
	aero.rpy[2]= atan2(2*(qe.s*qe.v[2] + qe.v[0]*qe.v[1]) , (qe.s*qe.s + qe.v[0]*qe.v[0] - qe.v[1]*qe.v[1] - qe.v[2]*qe.v[2]));
	
	return aero;
}

float get_yaw(UQuat_t qe) {
	return  atan2(2*(qe.s*qe.v[2] + qe.v[0]*qe.v[1]) , (qe.s*qe.s + qe.v[0]*qe.v[0] - qe.v[1]*qe.v[1] - qe.v[2]*qe.v[2]));
}

