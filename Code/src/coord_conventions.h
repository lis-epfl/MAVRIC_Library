/*
 * coord_conventions.h
 *
 * Created: 13/02/2013 16:47:26
 *  Author: Julien
 */ 

#ifndef COORD_CONVENTIONS_H_
#define COORD_CONVENTIONS_H_

#include "compiler.h"
#include "imu.h"
#include <math.h>

/*
 * Aeronautics convention : X front
 *							Y right
 *							Z down
*/
typedef struct {
	float rpy[3];
} Aero_Attitude_t;


/*
 * Schill convention :	X right
 *						Y front
 *						Z up
*/
typedef struct {
	float rpy[3];
} Schill_Attitude_t;


Aero_Attitude_t Quat_to_Aero(UQuat_t qe);
Schill_Attitude_t Quat_to_Schill(UQuat_t qe);

#endif /* STABILISATION_H_ */