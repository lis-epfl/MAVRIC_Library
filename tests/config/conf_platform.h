/*
 * conf_platform.h
 *
 * Created: 07/06/2012 22:02:26
 *  Author: sfx
 */ 


#ifndef CONF_PLATFORM_H_
#define CONF_PLATFORM_H_

//#include "conf_imu_rev3.h"
#include "conf_imu_rev4.h"

#define NATIVE_BIG_ENDIAN  

#define MAVLINK_SYS_ID 9

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

#define M_REAR_LEFT 0
#define M_FRONT_LEFT 1
#define M_FRONT_RIGHT 2
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
//#define GPS_ON_SERVO_1_2

// define type of remote controller
//#define SPEKTRUM_REMOTE
//#define TURNIGY_REMOTE
#define JOYSTICK_REMOTE

#endif /* CONF_PLATFORM_H_ */
