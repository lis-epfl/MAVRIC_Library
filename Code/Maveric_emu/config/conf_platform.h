/*
 * conf_platform.h
 *
 * Created: 07/06/2012 22:02:26
 *  Author: sfx
 */ 


/*
 * conf_platform.h
 *
 * Created: 07/06/2012 22:02:26
 *  Author: sfx
 */ 


#ifndef CONF_PLATFORM_H_
#define CONF_PLATFORM_H_


//#define NATIVE_BIG_ENDIAN  

#define CONF_DIAG
//#define CONF_CROSS

#define RC_INPUT_SCALE 0.5

#define JOYSTICK_REMOTE

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

//#define RAW_ACC_X_SCALE  261.5
//#define RAW_ACC_Y_SCALE  262.5
//#define RAW_ACC_Z_SCALE  255.0
#define RAW_ACC_X_SCALE  259.67
#define RAW_ACC_Y_SCALE  261.324
#define RAW_ACC_Z_SCALE  256.724

#define RAW_COMPASS_X 2
#define RAW_COMPASS_Y 0
#define RAW_COMPASS_Z 1

// Inside values
//#define RAW_MAG_X_SCALE 579.41
//#define RAW_MAG_Y_SCALE 540.3
//#define RAW_MAG_Z_SCALE 525.59

// Outside values
#define RAW_MAG_X_SCALE 496.38
#define RAW_MAG_Y_SCALE 504.85
#define RAW_MAG_Z_SCALE 448.42

// #define ACC_BIAIS_X 21.0;
// #define ACC_BIAIS_Y 9.0;
// #define ACC_BIAIS_Z -19.0;
#define ACC_BIAIS_X 5.92
#define ACC_BIAIS_Y 4.47
#define ACC_BIAIS_Z -23.13

// Inside values
//#define MAG_BIAIS_X 34.20
//#define MAG_BIAIS_Y -47.07
//#define MAG_BIAIS_Z -76.93

// Outside values
#define MAG_BIAIS_X 45.67
#define MAG_BIAIS_Y -58.41
#define MAG_BIAIS_Z -77.71

#define GYRO_OFFSET 0
#define ACC_OFFSET 3
#define COMPASS_OFFSET 6

#define UPVECTOR_X  0
#define UPVECTOR_Y  0
#define UPVECTOR_Z -1

#define FRONTVECTOR_X 1
#define FRONTVECTOR_Y 0
// Inside value
//#define FRONTVECTOR_Z -1.3846
// Outside value
#define FRONTVECTOR_Z -0.9067


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
