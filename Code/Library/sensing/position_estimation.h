/*
* position_estimation.h
*
* Created: 05.09.2013 17:48:32
*  Author: ndousse
*/


#ifndef POSITION_ESTIMATION_H__
#define POSITION_ESTIMATION_H__

#include "qfilter.h"
// leaky velocity integration as a simple trick to emulate drag and avoid too large deviations (loss per 1 second)
#define VEL_DECAY 0.05
#define POS_DECAY 0.0

void init_pos_integration();
void init_pos_gps();

void position_integration(Quat_Attitude_t *attitude, float dt);
void position_correction();

#endif // POSITION_ESTIMATION_H__