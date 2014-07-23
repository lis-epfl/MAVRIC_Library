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
#define RAW_GYRO_X_SCALE  818.5111
#define RAW_GYRO_Y_SCALE  818.5111
#define RAW_GYRO_Z_SCALE  818.5111

#define GYRO_ORIENTATION_X  1.0
#define GYRO_ORIENTATION_Y -1.0
#define GYRO_ORIENTATION_Z -1.0

#define RAW_ACC_X_SCALE  4114.96
#define RAW_ACC_Y_SCALE  4052.16
#define RAW_ACC_Z_SCALE  4007.06

#define ACC_BIAIS_X  85.00//-97.20;
#define ACC_BIAIS_Y  250.00//124.35;
#define ACC_BIAIS_Z  130//-40.73;

#define ACC_ORIENTATION_X  1.0
#define ACC_ORIENTATION_Y -1.0
#define ACC_ORIENTATION_Z -1.0

#define RAW_MAG_X_SCALE 490.57//624.11
#define RAW_MAG_Y_SCALE 501.79//590.77
#define RAW_MAG_Z_SCALE 436.09//512.37

#define MAG_BIAIS_X -315.89//-127.86
#define MAG_BIAIS_Y 37.87//-138.32
#define MAG_BIAIS_Z -518.49//-79.38

#define MAG_ORIENTATION_X -1.0
#define MAG_ORIENTATION_Y -1.0
#define MAG_ORIENTATION_Z -1.0

#endif /* CONF_IMU_REV4_H_ */