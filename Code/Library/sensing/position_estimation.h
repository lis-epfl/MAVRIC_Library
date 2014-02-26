/*
* position_estimation.h
*
* Created: 05.09.2013 17:48:32
*  Author: ndousse
*/


#ifndef POSITION_ESTIMATION_H__
#define POSITION_ESTIMATION_H__

#include "qfilter.h"
#include "bmp085.h"
#include "gps_ublox.h"
#include "coord_conventions.h"

// leaky velocity integration as a simple trick to emulate drag and avoid too large deviations (loss per 1 second)
#define VEL_DECAY 0.0
#define POS_DECAY 0.0


typedef struct position_estimator_t {
	float kp_vel[3], kp_pos[3], kp_alt, kp_vel_baro;

	uint32_t timeLastGpsMsg;
	uint32_t timeLastBarometerMsg;
	bool init_gps_position;
	bool init_barometer;
	
	float vel_bf[3], vel[3];

	float pos_correction[3];
	float last_alt, last_vel[3];

	local_coordinates_t localPosition;
	local_coordinates_t lastGpsPos;

} position_estimator_t;

void init_pos_integration(position_estimator_t *pos_est, pressure_data *barometer,gps_Data_type *gps );

void position_reset_home_altitude(position_estimator_t *pos_est, pressure_data *barometer, gps_Data_type *gps, local_coordinates_t *simLocalPos);

void position_integration(position_estimator_t *pos_est, Quat_Attitude_t *attitude, float dt);
void position_correction(position_estimator_t *pos_est, pressure_data *barometer, gps_Data_type *gps, float dt);

#endif // POSITION_ESTIMATION_H__