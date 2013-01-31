/*
 * imu.h
 *
 *  Created on: Mar 7, 2010
 *      Author: felix
 */

#ifndef IMU_H_
#define IMU_H_
#include "compiler.h"
#include "qfilter.h"

#define IMU_AXES 6

#include "conf_platform.h"

typedef struct {
	Quat_Attitude_t attitude;
	float raw_channels[9];
	float raw_bias[9];
	float raw_scale[9];
	uint32_t last_update;
	uint8_t valid;
	float dt;
	int8_t ready;
} Imu_Data_t;


void init_imu(Imu_Data_t *imu1);

void calibrate_Gyros(Imu_Data_t *imu1);

void imu_update(Imu_Data_t *imu1);

#endif /* IMU_H_ */
