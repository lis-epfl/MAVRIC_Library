/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file conf_imu_rev4.h
 *
 * This file configures the imu for the rev 4 of the maveric autopilot
 */


#ifndef CONF_IMU_REV4_H_
#define CONF_IMU_REV4_H_

#ifdef __cplusplus
	extern "C" {
#endif

#define RAW_GYRO_X 0				///< Define the index for the gyroscope raw values array
#define RAW_GYRO_Y 1				///< Define the index for the gyroscope raw values array
#define RAW_GYRO_Z 2				///< Define the index for the gyroscope raw values array

#define RAW_ACC_X 0					///< Define the index for the accelerometer raw values array
#define RAW_ACC_Y 1					///< Define the index for the accelerometer raw values array
#define RAW_ACC_Z 2					///< Define the index for the accelerometer raw values array

#define RAW_MAG_X 2					///< Define the index for the magnetometer raw values array
#define RAW_MAG_Y 0					///< Define the index for the magnetometer raw values array
#define RAW_MAG_Z 1					///< Define the index for the magnetometer raw values array

// from datasheet: FS 2000dps --> 70 mdps/digit
// scale = 1/(0.07 * PI / 180.0) = 818.5111f
#define RAW_GYRO_X_SCALE 818.5111f		///< Define the scale factor for the gyroscope raw values
#define RAW_GYRO_Y_SCALE 818.5111f		///< Define the scale factor for the gyroscope raw values
#define RAW_GYRO_Z_SCALE 818.5111f		///< Define the scale factor for the gyroscope raw values

#define GYRO_BIAIS_X 0.0f								///< Gyroscope x biais
#define GYRO_BIAIS_Y 0.0f								///< Gyroscope y biais
#define GYRO_BIAIS_Z 0.0f								///< Gyroscope z biais

#define GYRO_AXIS_X  1.0f				///< Define the gyroscope axis direction (according to sensor-autopilot orientation)
#define GYRO_AXIS_Y -1.0f				///< Define the gyroscope axis direction (according to sensor-autopilot orientation)
#define GYRO_AXIS_Z -1.0f				///< Define the gyroscope axis direction (according to sensor-autopilot orientation)

#define RAW_ACC_X_SCALE  3924.0f		///< Define the scale factor for the accelerometer raw values
#define RAW_ACC_Y_SCALE  3844.8f		///< Define the scale factor for the accelerometer raw values
#define RAW_ACC_Z_SCALE  4119.6f		///< Define the scale factor for the accelerometer raw values

#define ACC_BIAIS_X   0.0f				///< Define the bias for the accelerometer raw values
#define ACC_BIAIS_Y   64.0f				///< Define the bias for the accelerometer raw values
#define ACC_BIAIS_Z   90.0				///< Define the bias for the accelerometer raw values

#define ACC_AXIS_X  1.0f				///< Define the accelerometer axis direction (according to sensor-autopilot orientation)
#define ACC_AXIS_Y -1.0f				///< Define the accelerometer axis direction (according to sensor-autopilot orientation)
#define ACC_AXIS_Z -1.0f				///< Define the accelerometer axis direction (according to sensor-autopilot orientation)

#define RAW_MAG_X_SCALE 614.51f			///< Define the scale factor for the magnetometer raw values
#define RAW_MAG_Y_SCALE 584.28f			///< Define the scale factor for the magnetometer raw values
#define RAW_MAG_Z_SCALE 531.95f			///< Define the scale factor for the magnetometer raw values

#define MAG_BIAIS_X   42.38f			///< Define the bias for the magnetometer raw values
#define MAG_BIAIS_Y -107.51f			///< Define the bias for the magnetometer raw values
#define MAG_BIAIS_Z    2.47f			///< Define the bias for the magnetometer raw values

#define MAG_AXIS_X -1.0f				///< Define the magnetometer axis direction (according to sensor-autopilot orientation)
#define MAG_AXIS_Y -1.0f				///< Define the magnetometer axis direction (according to sensor-autopilot orientation)
#define MAG_AXIS_Z -1.0f				///< Define the magnetometer axis direction (according to sensor-autopilot orientation)

#ifdef __cplusplus
	}
#endif

#endif /* CONF_IMU_REV4_H_ */