/*
 * conf_platform.h
 *
 * Created: 07/06/2012 22:02:26
 *  Author: sfx
 */ 


#ifndef CONF_PLATFORM_H_
#define CONF_PLATFORM_H_

#define CONF_DIAG
//#define CONF_CROSS

// needs to be set for AVR32 target
//#define NATIVE_BIG_ENDIAN  

#define IMU_X 0
#define IMU_Y 1
#define IMU_Z 2

#define ROLL 0
#define PITCH 1
#define YAW 2

#define X			0
#define Y			1
#define Z			2

#define RAW_GYRO_X 1
#define RAW_GYRO_Y 0
#define RAW_GYRO_Z 2

#define RAW_GYRO_X_SCALE   12600.0
#define RAW_GYRO_Y_SCALE  -12600.0
#define RAW_GYRO_Z_SCALE   12600.0

#define RAW_ACC_X 0
#define RAW_ACC_Y 1
#define RAW_ACC_Z 2

#define RAW_ACC_X_SCALE  261.5
#define RAW_ACC_Y_SCALE  262.5
#define RAW_ACC_Z_SCALE  255.0


#define RAW_COMPASS_X 2
#define RAW_COMPASS_Y 0
#define RAW_COMPASS_Z 1

#define RAW_MAG_X_SCALE 290
#define RAW_MAG_Y_SCALE 237.5
#define RAW_MAG_Z_SCALE 488.5

#define GYRO_OFFSET 0
#define ACC_OFFSET 3
#define COMPASS_OFFSET 6

#define UPVECTOR_X  0
#define UPVECTOR_Y  0
#define UPVECTOR_Z -1

#define FRONTVECTOR_X 1
#define FRONTVECTOR_Y 0
#define FRONTVECTOR_Z -0.8558


// Definitions of Platform configuration

#define ROTORCOUNT 4

#define M_REAR_RIGHT 0
#define M_FRONT_RIGHT 1
#define M_FRONT_LEFT 2
#define M_REAR_LEFT 3

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


// define type of GPS
#define GPS_TYPE_UBX

#endif /* CONF_PLATFORM_H_ */
