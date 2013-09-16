/*
* position_estimation.h
*
* Created: 05.09.2013 17:48:32
*  Author: ndousse
*/


#ifndef POSITION_ESTIMATION_H__
#define POSITION_ESTIMATION_H__

#include "qfilter.h"

void init_pos_integration();
void init_pos_gps();

void position_integration(Quat_Attitude_t *attitude, float dt);
void position_correction();

#endif // POSITION_ESTIMATION_H__