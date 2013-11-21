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

// from datasheet: FS 2000dps --> 70 mdps/digit
// scale = 1/(0.07 * PI / 180.0) = 818.5111
#define RAW_GYRO_X_SCALE  1637.02223
#define RAW_GYRO_Y_SCALE  -1637.02223
#define RAW_GYRO_Z_SCALE  -1637.02223

#define RAW_ACC_X 0
#define RAW_ACC_Y 1
#define RAW_ACC_Z 2


#define RAW_ACC_X_SCALE   4000.0
#define RAW_ACC_Y_SCALE  -4000.0
#define RAW_ACC_Z_SCALE  -4000.0

#define RAW_COMPASS_X 2
#define RAW_COMPASS_Y 0
#define RAW_COMPASS_Z 1


#define RAW_MAG_X_SCALE 530.2771
#define RAW_MAG_Y_SCALE 525.2934
#define RAW_MAG_Z_SCALE 498.4476

#define ACC_BIAIS_X 0;
#define ACC_BIAIS_Y 0;
#define ACC_BIAIS_Z 0;


#define MAG_BIAIS_X  131.7582
#define MAG_BIAIS_Y -26.1298
#define MAG_BIAIS_Z  61.1646





#endif /* CONF_IMU_REV4_H_ */