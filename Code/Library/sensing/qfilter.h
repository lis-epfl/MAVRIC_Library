/*
 * qfilter.h
 *
 *  Created on: Apr 13, 2010
 *      Author: felix
 */

#ifndef QFILTER_H_
#define QFILTER_H_
#include "compiler.h"

#define GYRO_LPF 0.1
#define ACC_LPF 0.05
#define MAG_LPF 0.1

// leaky velocity integration as a simple trick to emulate drag and avoid too large deviations (loss per 1 second)
#define VEL_DECAY 0.01
#define POS_DECAY 0.0

#define GRAVITY 9.81

typedef struct {
	double longitude;
	double latitude;
	float altitude;
	uint32_t timestamp_ms;
} global_position_t;

typedef struct {
	double pos[3];
	global_position_t origin;
	uint32_t timestamp_ms;
} local_coordinates_t;

typedef struct UQuat {
	float s;
	float v[3];
} UQuat_t;

enum calibration_mode {OFF, LEVELING, LEVEL_PLUS_ACCEL};

typedef struct {
	UQuat_t qe;
	UQuat_t up_vec;
	
	float be[9], sf[9];
	float om[3], a[3], mag[3];
	float kp;
	float ki;
	float kp_mag;
	float ki_mag;
	float raw_mag_mean[3];
	uint8_t calibration_level;
	
	float acc_bf[3], vel_bf[3], vel[3];//, pos[3];

	float pos_correction[3];
	float last_alt, last_vel[3];
	float baro_alt_error;
	local_coordinates_t localPosition;
	local_coordinates_t lastGpsPos;

} Quat_Attitude_t;	

//float dt;

UQuat_t quat_local_to_global(UQuat_t qe, UQuat_t qvect);
UQuat_t quat_global_to_local(UQuat_t qe, UQuat_t qvect);

void qfInit(Quat_Attitude_t *attitude, float *scalefactor, float *bias);

void qfilter(Quat_Attitude_t *attitude, float *rates, float dt, bool simu_mode);

#endif /* QFILTER_H_ */
