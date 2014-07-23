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

#define RAW_ACC_X_SCALE  4015.2
#define RAW_ACC_Y_SCALE  3908.1
#define RAW_ACC_Z_SCALE  4220.2

#define ACC_BIAIS_X  60
#define ACC_BIAIS_Y  160
#define ACC_BIAIS_Z  220

#define ACC_ORIENTATION_X  1.0
#define ACC_ORIENTATION_Y -1.0
#define ACC_ORIENTATION_Z -1.0

#define RAW_MAG_X_SCALE 572.21
#define RAW_MAG_Y_SCALE 556.70
#define RAW_MAG_Z_SCALE 560.70

#define MAG_BIAIS_X  150
#define MAG_BIAIS_Y -120
#define MAG_BIAIS_Z  60

#define MAG_ORIENTATION_X -1.0
#define MAG_ORIENTATION_Y -1.0
#define MAG_ORIENTATION_Z  1.0

#endif /* CONF_IMU_REV4_H_ */
