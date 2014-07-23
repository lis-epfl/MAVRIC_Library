/*
 * conf_imu_rev4.h
 *
 * Created: 20/11/2013 22:21:49
 *  Author: sfx
 */ 


#ifndef CONF_IMU_REV4_H_
#define CONF_IMU_REV4_H_


#define GYRO_AXIS_X 0
#define GYRO_AXIS_Y 1
#define GYRO_AXIS_Z 2

#define ACC_AXIS_X 0
#define ACC_AXIS_Y 1
#define ACC_AXIS_Z 2

#define MAG_AXIS_X 2
#define MAG_AXIS_Y 0
#define MAG_AXIS_Z 1

// from datasheet: FS 2000dps --> 70 mdps/digit
// scale = 1/(0.07 * PI / 180.0) = 818.5111
#define RAW_GYRO_X_SCALE  1637.02223
#define RAW_GYRO_Y_SCALE  -1637.02223
#define RAW_GYRO_Z_SCALE  -1637.02223

#define RAW_ACC_X_SCALE  3960.7
#define RAW_ACC_Y_SCALE  4005.7
#define RAW_ACC_Z_SCALE  4026.8

#define ACC_BIAIS_X 64
#define ACC_BIAIS_Y  128
#define ACC_BIAIS_Z  210

#define ACC_ORIENTATION_X  1.0
#define ACC_ORIENTATION_Y -1.0
#define ACC_ORIENTATION_Z -1.0

#define RAW_MAG_X_SCALE 624.01
#define RAW_MAG_Y_SCALE 590.35
#define RAW_MAG_Z_SCALE 523.8

#define MAG_BIAIS_X  -50				
#define MAG_BIAIS_Y -150
#define MAG_BIAIS_Z   7.17

#define MAG_ORIENTATION_X -1.0
#define MAG_ORIENTATION_Y -1.0
#define MAG_ORIENTATION_Z  1.0

#endif /* CONF_IMU_REV4_H_ */
