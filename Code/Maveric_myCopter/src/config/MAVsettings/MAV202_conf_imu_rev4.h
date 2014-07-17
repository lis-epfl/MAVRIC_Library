/* The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file MAV202_conf_imu_rev4.h
 *
 *  This file defines the mapping between the IMU and the compass and the frames of the vehicles 
 * as well as the scales and the biaises. The NED frame is used. 
 */


#ifndef CONF_IMU_REV4_H_
#define CONF_IMU_REV4_H_

#define RAW_GYRO_X 0									///< Gyroscope x axis
#define RAW_GYRO_Y 1									///< Gyroscope y axis
#define RAW_GYRO_Z 2									///< Gyroscope z axis

#define RAW_ACC_X 0										///< Accelerometer x axis
#define RAW_ACC_Y 1										///< Accelerometer y axis
#define RAW_ACC_Z 2										///< Accelerometer z axis

#define RAW_MAG_X 2										///< Compass x axis
#define RAW_MAG_Y 0										///< Compass y axis
#define RAW_MAG_Z 1										///< Compass z axis

// from datasheet: FS 2000dps --> 70 mdps/digit
// scale = 1/(0.07 * PI / 180.0) = 818.5111f
#define RAW_GYRO_X_SCALE 818.5111f						///< Gyroscope x axis scale
#define RAW_GYRO_Y_SCALE 818.5111f						///< Gyroscope y axis scale
#define RAW_GYRO_Z_SCALE 818.5111f						///< Gyroscope z axis scale

#define GYRO_AXIS_X  1.0f								///< Gyroscope x axis direction
#define GYRO_AXIS_Y -1.0f								///< Gyroscope y axis direction
#define GYRO_AXIS_Z -1.0f								///< Gyroscope z axis direction

#define RAW_ACC_X_SCALE 4145.7462f						///< Accelerometer x axis scale
#define RAW_ACC_Y_SCALE 4071.6653f						///< Accelerometer y axis scale
#define RAW_ACC_Z_SCALE 4141.9266f						///< Accelerometer z axis scale

#define ACC_BIAIS_X -128.0f								///< Accelerometer x axis biais
#define ACC_BIAIS_Y  130.0f								///< Accelerometer y axis biais
#define ACC_BIAIS_Z  230.0f								///< Accelerometer z axis biais

#define ACC_AXIS_X  1.0f									///< Accelerometer x axis direction
#define ACC_AXIS_Y -1.0f								///< Accelerometer y axis direction
#define ACC_AXIS_Z -1.0f									///< Accelerometer z axis direction

#define RAW_MAG_X_SCALE 671.1324f						///< Compass x axis scale
#define RAW_MAG_Y_SCALE 699.7155f						///< Compass y axis scale
#define RAW_MAG_Z_SCALE 600.8004f						///< Compass z axis scale

#define MAG_BIAIS_X   22.6809f							///< Compass x axis biais
#define MAG_BIAIS_Y   53.48f								///< Compass y axis biais
#define MAG_BIAIS_Z -357.72f								///< Compass z axis biais

#define MAG_AXIS_X -1.0f									///< Compass x axis direction
#define MAG_AXIS_Y -1.0f									///< Compass y axis direction
#define MAG_AXIS_Z -1.0f									///< Compass z axis direction

#endif /* CONF_IMU_REV4_H_ */