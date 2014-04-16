/*
 * conf_imu_rev4.h
 *
 * Created: 20/11/2013 22:21:49
 *  Author: sfx
 */ 


#ifndef CONF_IMU_REV4_H_
#define CONF_IMU_REV4_H_


#define RAW_GYRO_X 0
#define RAW_GYRO_Y 1
#define RAW_GYRO_Z 2

#define RAW_ACC_X 0
#define RAW_ACC_Y 1
#define RAW_ACC_Z 2

#define RAW_COMPASS_X 2
#define RAW_COMPASS_Y 0
#define RAW_COMPASS_Z 1

// from datasheet: FS 2000dps --> 70 mdps/digit
// scale = 1/(0.07 * PI / 180.0) = 818.5111
#define RAW_GYRO_X_SCALE  1637.02223
#define RAW_GYRO_Y_SCALE  1637.02223
#define RAW_GYRO_Z_SCALE  1637.02223

#define GYRO_AXIS_X  1.0
#define GYRO_AXIS_Y -1.0
#define GYRO_AXIS_Z -1.0

#define RAW_ACC_X_SCALE  3929.0
#define RAW_ACC_Y_SCALE  4080.4
#define RAW_ACC_Z_SCALE  4056.5

#define ACC_BIAIS_X   15
#define ACC_BIAIS_Y   30
#define ACC_BIAIS_Z  440

#define ACC_AXIS_X  1.0
#define ACC_AXIS_Y -1.0
#define ACC_AXIS_Z -1.0

#define RAW_MAG_X_SCALE 593.4
#define RAW_MAG_Y_SCALE 574.76
#define RAW_MAG_Z_SCALE 548.33

#define MAG_BIAIS_X -150
#define MAG_BIAIS_Y -160
#define MAG_BIAIS_Z  -46.51

#define MAG_AXIS_X -1.0
#define MAG_AXIS_Y -1.0
#define MAG_AXIS_Z  1.0

#endif /* CONF_IMU_REV4_H_ */