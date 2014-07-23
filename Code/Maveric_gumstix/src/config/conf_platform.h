/*
 * conf_platform.h
 *
 * Created: 07/06/2012 22:02:26
 *  Author: sfx
 */ 


#ifndef CONF_PLATFORM_H_
#define CONF_PLATFORM_H_


#define NATIVE_BIG_ENDIAN  

#define MAVLINK_SYS_ID 1

#define MAVLINK_BASE_STATION_ID 255

#define CONF_DIAG
//#define CONF_CROSS

#define RC_INPUT_SCALE 0.8
// Thrust compensation for hover (relative to center position)
#define THRUST_HOVER_POINT (-0.3)


#define IMU_X 0
#define IMU_Y 1
#define IMU_Z 2

#define ROLL 0
#define PITCH 1
#define YAW 2

#define X			0
#define Y			1
#define Z			2

#define GYRO_AXIS_X 1
#define GYRO_AXIS_Y 0
#define GYRO_AXIS_Z 2

#define RAW_GYRO_X_SCALE   12600.0
#define RAW_GYRO_Y_SCALE  -12600.0
#define RAW_GYRO_Z_SCALE   12600.0

#define ACC_AXIS_X 0
#define ACC_AXIS_Y 1
#define ACC_AXIS_Z 2

//#define RAW_ACC_X_SCALE  261.5
//#define RAW_ACC_Y_SCALE  262.5
//#define RAW_ACC_Z_SCALE  255.0
//#define RAW_ACC_X_SCALE  259.67
//#define RAW_ACC_Y_SCALE  261.324
//#define RAW_ACC_Z_SCALE  256.724
// Felix outside
//#define RAW_ACC_X_SCALE  264.9173
#define RAW_ACC_X_SCALE  258.9173
#define RAW_ACC_Y_SCALE  258.9853
#define RAW_ACC_Z_SCALE  258.0829

#define MAG_AXIS_X 2
#define MAG_AXIS_Y 0
#define MAG_AXIS_Z 1

// Inside values
//#define RAW_MAG_X_SCALE 579.41
//#define RAW_MAG_Y_SCALE 540.3
//#define RAW_MAG_Z_SCALE 525.59

// Outside values
//#define RAW_MAG_X_SCALE 534.90
//#define RAW_MAG_Y_SCALE 514.85
//#define RAW_MAG_Z_SCALE 478.57

// Felix Outside values
#define RAW_MAG_X_SCALE 530.2771
#define RAW_MAG_Y_SCALE 525.2934
#define RAW_MAG_Z_SCALE 498.4476

 #define ACC_BIAIS_X 18.0;
 #define ACC_BIAIS_Y 9.0;
 #define ACC_BIAIS_Z -16.0;
//#define ACC_BIAIS_X 4.685
//#define ACC_BIAIS_Y 4.376
//#define ACC_BIAIS_Z -16.26

// Felix Outside values
// #define ACC_BIAIS_X  21.5871
// #define ACC_BIAIS_Y  10.0884
// #define ACC_BIAIS_Z  -14.9891


// Inside values
//#define MAG_BIAIS_X 34.20
//#define MAG_BIAIS_Y -47.07
//#define MAG_BIAIS_Z -76.93

// Outside values
//#define MAG_BIAIS_X 47.62
//#define MAG_BIAIS_Y -47.29
//#define MAG_BIAIS_Z -74.38

// Felix Outside values
#define MAG_BIAIS_X  131.7582
#define MAG_BIAIS_Y -26.1298
#define MAG_BIAIS_Z  61.1646



#define GYRO_OFFSET 0
#define ACC_OFFSET 3
#define MAG_OFFSET 6

#define UPVECTOR_X  0
#define UPVECTOR_Y  0
#define UPVECTOR_Z -1

#define FRONTVECTOR_X 1
#define FRONTVECTOR_Y 0
#define FRONTVECTOR_Z 0
// Inside value
//#define FRONTVECTOR_Z -1.3846
// Outside value
//#define FRONTVECTOR_Z -0.8985


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


// Definitions of Hybrid platform
#define MAIN_ENGINE 4 
#define FLAP_FRONT 1 
#define FLAP_RIGHT 2 
#define FLAP_REAR 3 
#define FLAP_LEFT 0

#define FLAP_FRONT_DIR (1) // 1 if positive value gives positive roll, -1 else
#define FLAP_RIGHT_DIR (1) 
#define FLAP_REAR_DIR (1) 
#define FLAP_LEFT_DIR (1)

#define MIN_DEFLECTION -1.0
#define MAX_DEFLECTION 1.0
#define SERVO_AMPLITUDE 500
#define SERVO_NEUTRAL 0


// define type of GPS
#define GPS_TYPE_UBX

// define type of remote controller
//#define SPEKTRUM_REMOTE
#define TURNIGY_REMOTE

//define type Gumstix
#define SF_GUMSTIX_API 1
#define SF_ADVANCED_GUMSTIX_API 1

#endif /* CONF_PLATFORM_H_ */
