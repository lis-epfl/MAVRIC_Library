/*
 * coord_conventions.c
 *
 * Created: 13/02/2013 16:47:26
 *  Author: Julien
 */ 

#include "coord_conventions.h"

Aero_Attitude_t Quat_to_Aero(UQuat_t qe) {
	Aero_Attitude_t aero;

	aero.rpy[0]= atan(2*(qe.s*qe.v[0] + qe.v[1]*qe.v[2]) / (qe.s*qe.s - qe.v[0]*qe.v[0] - qe.v[1]*qe.v[1] + qe.v[2]*qe.v[2])); 
	aero.rpy[1]=-asin(2*(qe.v[0]*qe.v[2] - qe.s*qe.v[1]));
	aero.rpy[2]= atan(2*(qe.s*qe.v[2] + qe.v[0]*qe.v[1]) / (qe.s*qe.s - qe.v[0]*qe.v[0] - qe.v[1]*qe.v[1] + qe.v[2]*qe.v[2]));

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