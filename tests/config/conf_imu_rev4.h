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

#define RAW_ACC_X_SCALE  4116.1
#define RAW_ACC_Y_SCALE  4029.3
#define RAW_ACC_Z_SCALE  4152.3

#define ACC_BIAIS_X  -131.6
#define ACC_BIAIS_Y  338.28
#define ACC_BIAIS_Z  223.65

#define ACC_ORIENTATION_X  1.0
#define ACC_ORIENTATION_Y -1.0
#define ACC_ORIENTATION_Z -1.0

#define RAW_MAG_X_SCALE 576.74
#define RAW_MAG_Y_SCALE 556.33
#define RAW_MAG_Z_SCALE 521.03

#define MAG_BIAIS_X   84.23
#define MAG_BIAIS_Y   54.21
#define MAG_BIAIS_Z  112.59

#define MAG_ORIENTATION_X -1.0
#define MAG_ORIENTATION_Y -1.0
#define MAG_ORIENTATION_Z  1.0

//#define RAW_ACC_X_SCALE  4096.0
//#define RAW_ACC_Y_SCALE  4096.0
//#define RAW_ACC_Z_SCALE  4096.0
//
//#define ACC_BIAIS_X  170
//#define ACC_BIAIS_Y  -16
//#define ACC_BIAIS_Z  350
//
//#define ACC_ORIENTATION_X  1.0
//#define ACC_ORIENTATION_Y -1.0
//#define ACC_ORIENTATION_Z -1.0
//
//#define RAW_MAG_X_SCALE 530.27
//#define RAW_MAG_Y_SCALE 525.29
//#define RAW_MAG_Z_SCALE 498.44
//
//#define MAG_BIAIS_X   131.76
//#define MAG_BIAIS_Y   -26.13
//#define MAG_BIAIS_Z    61.16
//
//#define MAG_ORIENTATION_X -1.0
//#define MAG_ORIENTATION_Y -1.0
//#define MAG_ORIENTATION_Z  1.0

#endif /* CONF_IMU_REV4_H_ */