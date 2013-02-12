/*
 * conf_platform.h
 *
 * Created: 07/06/2012 22:02:26
 *  Author: sfx
 */ 


#ifndef CONF_PLATFORM_H_
#define CONF_PLATFORM_H_

//#define CONF_DIAG
#define CONF_CROSS

#define IMU_X 0
#define IMU_Y 1
#define IMU_Z 2

#define RAW_GYRO_X 0
#define RAW_GYRO_Y 1
#define RAW_GYRO_Z 2

#define RAW_ACC_X 1
#define RAW_ACC_Y 0
#define RAW_ACC_Z 2

#define RAW_COMPASS_X 0
#define RAW_COMPASS_Y 1
#define RAW_COMPASS_Z 2

#define GYRO_OFFSET 0
#define ACC_OFFSET 3

#define M_FRONT_LEFT 0
#define M_FRONT_RIGHT 1
#define M_REAR_LEFT 2
#define M_REAR_RIGHT 3

#define M_FR_DIR ( 1)
#define M_FL_DIR (-1)
#define M_RR_DIR (-1)
#define M_RL_DIR ( 1)

#define M_FRONT 0
#define M_RIGHT 1
#define M_REAR 2
#define M_LEFT 3

#define M_FRONT_DIR ( 1)
#define M_RIGHT_DIR (-1)
#define M_REAR_DIR  ( 1)
#define M_LEFT_DIR  (-1)

#define MIN_THRUST -0.9
#define MAX_THRUST 1.0
#define SERVO_SCALE 500

#endif /* CONF_PLATFORM_H_ */