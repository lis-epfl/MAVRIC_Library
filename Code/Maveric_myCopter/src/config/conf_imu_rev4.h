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
// scale = 1/(0.07f * PI / 180.0f) = 818.5111f
#define RAW_GYRO_X_SCALE 818.5111f
#define RAW_GYRO_Y_SCALE 818.5111f
#define RAW_GYRO_Z_SCALE 818.5111f

#define GYRO_AXIS_X  1.0f
#define GYRO_AXIS_Y -1.0f
#define GYRO_AXIS_Z -1.0f

#define RAW_ACC_X_SCALE  3924.0f
#define RAW_ACC_Y_SCALE  3844.8f
#define RAW_ACC_Z_SCALE  4119.6f

#define ACC_BIAIS_X   0.0f
#define ACC_BIAIS_Y   64.0f
#define ACC_BIAIS_Z   90.0f

#define ACC_AXIS_X  1.0f
#define ACC_AXIS_Y -1.0f
#define ACC_AXIS_Z -1.0f

#define RAW_MAG_X_SCALE 614.51f
#define RAW_MAG_Y_SCALE 584.28f
#define RAW_MAG_Z_SCALE 531.95f

#define MAG_BIAIS_X   42.38f
#define MAG_BIAIS_Y -107.51f
#define MAG_BIAIS_Z    2.47f

#define MAG_AXIS_X -1.0f
#define MAG_AXIS_Y -1.0f
#define MAG_AXIS_Z -1.0f

#endif /* CONF_IMU_REV4_H_ */