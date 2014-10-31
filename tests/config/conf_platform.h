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

#define MAVLINK_SYS_ID 9


// Definitions of Platform configuration
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

#endif /* CONF_PLATFORM_H_ */