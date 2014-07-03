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
// scale = 1/(0.07 * PI / 180.0) = 818.5111
#define RAW_GYRO_X_SCALE 818.5111						///< Gyroscope x axis scale
#define RAW_GYRO_Y_SCALE 818.5111						///< Gyroscope y axis scale
#define RAW_GYRO_Z_SCALE 818.5111						///< Gyroscope z axis scale

#define GYRO_AXIS_X  1.0								///< Gyroscope x axis direction
#define GYRO_AXIS_Y -1.0								///< Gyroscope y axis direction
#define GYRO_AXIS_Z -1.0								///< Gyroscope z axis direction

#define RAW_ACC_X_SCALE 4114.96							///< Accelerometer x axis scale
#define RAW_ACC_Y_SCALE 4052.16							///< Accelerometer y axis scale
#define RAW_ACC_Z_SCALE 4007.06							///< Accelerometer z axis scale

#define ACC_BIAIS_X  85.00								///< Accelerometer x axis biais
#define ACC_BIAIS_Y 250.00								///< Accelerometer y axis biais
#define ACC_BIAIS_Z 130									///< Accelerometer z axis biais

#define ACC_AXIS_X  1.0									///< Accelerometer x axis direction
#define ACC_AXIS_Y -1.0									///< Accelerometer y axis direction
#define ACC_AXIS_Z -1.0									///< Accelerometer z axis direction

#define RAW_MAG_X_SCALE 490.57							///< Compass x axis scale
#define RAW_MAG_Y_SCALE 501.79							///< Compass y axis scale
#define RAW_MAG_Z_SCALE 436.09							///< Compass z axis scale

#define MAG_BIAIS_X -315.89								///< Compass x axis biais
#define MAG_BIAIS_Y   37.87								///< Compass y axis biais
#define MAG_BIAIS_Z -518.49								///< Compass z axis biais

#define MAG_AXIS_X -1.0									///< Compass x axis direction
#define MAG_AXIS_Y -1.0									///< Compass y axis direction
#define MAG_AXIS_Z -1.0									///< Compass z axis direction

#endif /* CONF_IMU_REV4_H_ */