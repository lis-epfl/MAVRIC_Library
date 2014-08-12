/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file mav_mode.h
 *
 *  Place where the mav modes and flags aredefined
 */

#ifndef MAV_MODE_H
#define MAV_MODE_H

#include <stdint.h>


typedef enum
{
	ARMED_OFF = 0,
	ARMED_ON  = 1,
} mode_flag_armed_t;


typedef enum
{
	MANUAL_OFF = 0,
	MANUAL_ON  = 1,
} mode_flag_manual_t;


typedef enum
{
	HIL_OFF = 0,			
	HIL_ON  = 1,
} mode_flag_hil_t;


typedef enum
{
	STABILISE_OFF = 0,
	STABILISE_ON  = 1,
} mode_flag_stabilise_t;


typedef enum
{
	GUIDED_OFF = 0,
	GUIDED_ON  = 1,
} mode_flag_guided_t;


typedef enum
{
	AUTO_OFF = 0,
	AUTO_ON  = 1,
} mode_flag_auto_t;


typedef enum
{
	TEST_OFF = 0,
	TEST_ON  = 1,
} mode_flag_test_t;


typedef enum
{
	CUSTOM_OFF = 0,
	CUSTOM_ON  = 1,
} mode_flag_custom_t;


typedef struct 
{
    mode_flag_custom_t 	CUSTOM       : 1;
    mode_flag_test_t 	TEST         : 1;
    mode_flag_auto_t 	AUTO         : 1;
    mode_flag_guided_t 	GUIDED       : 1;
    mode_flag_stabilise_t STABILISE  : 1;
    mode_flag_hil_t 	HIL          : 1;
    mode_flag_manual_t 	MANUAL       : 1;
    mode_flag_armed_t 	ARMED        : 1;
} mav_mode_bitfield_t; 


typedef union
{
	uint8_t byte;
	mav_mode_bitfield_t flags;
} mav_mode_t;


// typedef uint8_t mav_mode_t;


typedef enum
{
	MODE_FLAG_CUSTOM    = 0,
	MODE_FLAG_TEST      = 1,
	MODE_FLAG_AUTO      = 2,
	MODE_FLAG_GUIDED    = 3,
	MODE_FLAG_STABILISE = 4,
	MODE_FLAG_HIL       = 5,
	MODE_FLAG_MANUAL    = 6,
	MODE_FLAG_ARMED     = 7,
} mode_flags_t;


typedef enum MAV_MODE_FLAG mav_flag_mask_t;


typedef enum
{
	MAV_MODE_PRE = 0,
	MAV_MODE_SAFE = 64,
	MAV_MODE_ATTITUDE_CONTROL = 192,
	MAV_MODE_VELOCITY_CONTROL = 208,
	MAV_MODE_POSITION_HOLD = 216,
	MAV_MODE_GPS_NAVIGATION = 148
} mav_mode_predefined_t;

#endif