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

typedef struct UQuat {
	float s;
	float v[3];
} UQuat_t;

enum calibration_mode {OFF, LEVELING, LEVEL_PLUS_ACCEL};

typedef struct {
	UQuat_t qe;
	UQuat_t up_vec;
	
	float be[6], sf[6];
	float om[3], a[3], mag[3];
	float kp;
	float ki;
	uint8_t calibration_level;
	
	float acc[3], vel[3], pos[3];

} Quat_Attitude_t;	

//float dt;

void qfInit(Quat_Attitude_t *attitude, float *scalefactor, float *bias);

void qfilter(Quat_Attitude_t *attitude, float *rates, float dt);


#endif /* QFILTER_H_ */
